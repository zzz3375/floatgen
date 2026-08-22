#!/usr/bin/env bash
# One-shot bringup: PX4 SITL (wind_farm world) + MicroXRCEAgent + gz<->ROS2
# bridges (clock, odom, mid360 lidar, camera) + RViz + inspection planner.
#
# Usage:  bash scripts/wind_farm_bringup.sh [HEADLESS]
#   HEADLESS=1 (or any non-empty arg) runs gz without the GUI.
#
# Launches, in order:
#   1. MicroXRCEAgent (uXRCE-DDS bridge, PX4 <-> ROS2)
#   2. PX4 SITL with the wind_farm world (x500_mid360 at config/drone_home:
#      front camera + top Mid-360 lidar, see PX4 Tools/simulation/gz/models/x500_mid360)
#   3. Spawn turbines via xacro + ros_gz_sim create (target with blade animation;
#      decorations static; velocity from config/wind_farm.yaml)
#   4. gz -> ROS2 bridges (/clock, odometry, /mid360/points, /mid360, /camera)
#   5. TF broadcaster (world -> drone frames) + RViz (config/rviz/wind_farm.rviz)
#   6. flight_path_publisher (nav_msgs/Path in world frame for RViz trail)
#   7. wind_farm_simulator (offboard orbit inspection, then RTL + land)
#
# Requires: PX4-Autopilot at ~/PX4-Autopilot, ROS2 Humble workspace sourced,
# px4_msgs/px4_ros_com built in the workspace.
# NOTE: do not use `set -u` here: sourcing /opt/ros/*/setup.bash reads
# AMENT_TRACE_SETUP_FILES which is unset by default and kills nounset shells.
set +u

HEADLESS="${1:-${HEADLESS:-false}}"
# normalise: "false"/"0"/"" → empty (GUI on); anything else → "1"
case "${HEADLESS,,}" in false|0|"") HEADLESS="";; esac
WORKSPACE="${ROS2_WS:-$HOME/ros2_ws}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
FLOATGEN_SRC="$WORKSPACE/src/floatgen"
SIMULATOR="$FLOATGEN_SRC/scripts/wind_farm_simulator.py"
CONFIG="$FLOATGEN_SRC/config/wind_farm.yaml"

# WSL2: FastDDS discovery is unreliable here; CycloneDDS interoperates with the
# agent's FastDDS and discovers over default interfaces without a profile.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$FLOATGEN_SRC/gz/dds/loopback_fastdds.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"

# drone home must match config/wind_farm.yaml -> drone_home
DRONE_HOME_X="$(python3 -c "import yaml;print(yaml.safe_load(open('$CONFIG'))['drone_home']['x'])")"
DRONE_HOME_Y="$(python3 -c "import yaml;print(yaml.safe_load(open('$CONFIG'))['drone_home']['y'])")"
DRONE_HOME_Z="$(python3 -c "import yaml;print(yaml.safe_load(open('$CONFIG'))['drone_home']['z'])")"
MODEL_POSE="$DRONE_HOME_X,$DRONE_HOME_Y,$DRONE_HOME_Z,0,0,0"

# turbine blade rotation (rad/s); 0 = stopped, -2 = spinning (default)
TURBINE_VELOCITY="$(python3 -c "import yaml;print(yaml.safe_load(open('$CONFIG')).get('turbine_velocity', -2.0))")"

_kill_tree() {
    # Recursively terminate a process and all its descendants (depth-first).
    local pid=$1
    local child
    for child in $(pgrep -P "$pid" 2>/dev/null); do
        _kill_tree "$child"
    done
    kill "$pid" 2>/dev/null || true
}

cleanup() {
    echo "=== shutting down ==="
    # Without setsid we cannot use kill -- -$PGID in a non-interactive shell.
    # _kill_tree walks the make→px4 tree while make is still alive, and the
    # explicit pkill -f calls catch gz/px4 processes that make already exited
    # and were reparented to init before cleanup ran.
    [ -n "${PX4_PID:-}" ]   && _kill_tree "$PX4_PID"
    pkill -f "gz sim"                       2>/dev/null || true
    pkill -f "parameter_bridge"              2>/dev/null || true
    [ -n "${AGENT_PID:-}" ]  && kill "$AGENT_PID"  2>/dev/null || true
    [ -n "${BRIDGE_PID:-}" ] && kill "$BRIDGE_PID" 2>/dev/null || true
    [ -n "${TF_PID:-}" ]     && kill "$TF_PID"     2>/dev/null || true
    [ -n "${RVIZ_PID:-}" ]   && kill "$RVIZ_PID"   2>/dev/null || true
    [ -n "${PATH_PID:-}" ]   && kill "$PATH_PID"   2>/dev/null || true
    [ -n "${TURBINE_PID:-}" ] && kill "$TURBINE_PID" 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT
# non-interactive bash ignores SIGINT; TERM is the reliable shutdown signal
trap 'exit 130' TERM INT

echo "=== [1/7] MicroXRCEAgent ==="
pkill -x MicroXRCEAgent 2>/dev/null || true
# stale gz instances make PX4 attach to an old world (drone GPS/height never
# converges, arming denied); always start from a fresh wind_farm world
pkill -f "gz sim" 2>/dev/null || true
pkill -f "ros_gz_bridge parameter_bridge" 2>/dev/null || true
sleep 1
MicroXRCEAgent udp4 -p 8888 -v 0 &
AGENT_PID=$!

echo "=== [2/7] PX4 SITL (world=wind_farm, model=gz_x500_mid360, pose=$MODEL_POSE) ==="
cd "$PX4_DIR"
if [ -n "$HEADLESS" ]; then export HEADLESS=1; fi
# UXRCE_DDS_SYNCT=0
# NOTE: do NOT use setsid — it detaches from the X11 session and prevents
# gz-sim's GUI window from connecting to the display.  Launch make directly in
# the background; non-interactive bash places it in the script's own process
# group, so cleanup uses pkill -P to walk the process tree by parent PID.
PX4_GZ_WORLD=wind_farm PX4_GZ_MODEL_POSE="$MODEL_POSE" make px4_sitl gz_x500_mid360 &
PX4_PID=$!

# wait for the uXRCE-DDS topics to appear
echo "=== waiting for /fmu topics ==="
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
timeout 15 ros2 daemon start >/dev/null 2>&1 || true
for i in $(seq 1 90); do
    if timeout -k 2 3 ros2 topic list 2>/dev/null | grep -q '^/fmu/out/vehicle_status'; then
        break
    fi
    sleep 1
done
if ! timeout -k 2 3 ros2 topic list 2>/dev/null | grep -q '^/fmu/out/vehicle_status'; then
    echo "ERROR: /fmu topics not available after 90s" >&2
    exit 1
fi
echo "fmu topics up"

echo "=== [3/7] spawn target turbine (velocity=$TURBINE_VELOCITY rad/s) ==="
# The three decoration turbines (2_1, 1_2, 2_2) are static models in the
# world SDF — they load when PX4 starts Gazebo.  Only the target turbine
# (1_1 at origin) is spawned dynamically here so its blade rotation velocity
# can be configured via config/wind_farm.yaml.
ros2 run xacro xacro "$FLOATGEN_SRC/urdf/farm.xacro" \
    nx:=1 ny:=1 x:=0.0 y:=0.0 scale:=200.0 yaw:=0.0 \
    "velocity:=$TURBINE_VELOCITY" \
    nacelle_yaw:=0.0 blade_pitch:=0.0 hub_position:=0.0 \
    > /tmp/tw_target.urdf
ros2 run ros_gz_sim create -name turbine_target -file /tmp/tw_target.urdf &
TURBINE_PID=$!
sleep 3
rm -f /tmp/tw_target.urdf

# bridge the gz sensor/odom/clock topics to ROS2 (rviz + tf broadcaster).
# Sensor topics are not model-scoped, so the drone's camera/lidar publish at
# /camera and /mid360 (single-drone world). The drone odometry is on a custom
# topic (the model's OdometryPublisher plugin redirects it away from the
# /model/.../odometry_with_covariance topic that PX4's gz_bridge consumes).
echo "=== [4/7] gz -> ROS2 bridges (clock, odom, lidar, camera) ==="
ros2 run ros_gz_bridge parameter_bridge \
    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
    /x500_mid360/odom_with_cov@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance \
    /mid360/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
    /mid360@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
    /camera@sensor_msgs/msg/Image[gz.msgs.Image \
    /camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo &
BRIDGE_PID=$!
# Wait for /clock to stabilise before launching nodes with use_sim_time:=true.
# Without this, rviz2 and tf2_buffer initialise against system time, then
# reset on the first /clock message → repeated "Detected jump back in time".
for i in $(seq 1 20); do
    if timeout -k 1 2 ros2 topic hz /clock 2>/dev/null | grep -q 'average rate'; then
        break
    fi
    sleep 0.5
done
sleep 1

echo "=== [5/7] TF broadcaster + RViz ==="
python3 -u "$FLOATGEN_SRC/scripts/gz_tf_broadcaster.py" --ros-args -p use_sim_time:=true -p filter_alpha:=0.25 &
TF_PID=$!
if [ -z "$HEADLESS" ]; then
    rviz2 -d "$FLOATGEN_SRC/config/rviz/wind_farm.rviz" --ros-args -p use_sim_time:=true &
    RVIZ_PID=$!
fi
sleep 3

echo "=== [6/7] flight_path_publisher ==="
python3 -u "$FLOATGEN_SRC/scripts/flight_path_publisher.py" "$CONFIG" &
PATH_PID=$!

echo "=== [7/7] wind_farm_simulator ==="
python3 -u "$SIMULATOR" "$CONFIG"

sleep 3
echo "=== mission finished ==="
