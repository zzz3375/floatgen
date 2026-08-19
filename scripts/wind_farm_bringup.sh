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
#   3. gz -> ROS2 bridges (/clock, odometry, /mid360/points, /mid360, /camera)
#   4. TF broadcaster (world -> drone frames) + RViz (config/rviz/wind_farm.rviz)
#   5. wind_farm_planner (offboard orbit inspection, then RTL + land)
#
# Requires: PX4-Autopilot at ~/PX4-Autopilot, ROS2 Humble workspace sourced,
# px4_msgs/px4_ros_com built in the workspace.
# NOTE: do not use `set -u` here: sourcing /opt/ros/*/setup.bash reads
# AMENT_TRACE_SETUP_FILES which is unset by default and kills nounset shells.
set +u

HEADLESS="${1:-${HEADLESS:-}}"
WORKSPACE="${ROS2_WS:-$HOME/ros2_ws}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
FLOATGEN_SRC="$WORKSPACE/src/floatgen"
PLANNER="$FLOATGEN_SRC/scripts/wind_farm_planner.py"
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

cleanup() {
    echo "=== shutting down ==="
    # kill the whole process groups (make/px4/gz tree + agent)
    [ -n "${PX4_PID:-}" ] && kill -- -"$PX4_PID" 2>/dev/null
    [ -n "${AGENT_PID:-}" ] && kill "$AGENT_PID" 2>/dev/null
    [ -n "${BRIDGE_PID:-}" ] && kill "$BRIDGE_PID" 2>/dev/null
    [ -n "${TF_PID:-}" ] && kill "$TF_PID" 2>/dev/null
    [ -n "${RVIZ_PID:-}" ] && kill "$RVIZ_PID" 2>/dev/null
    wait 2>/dev/null || true
}
trap cleanup EXIT
# non-interactive bash ignores SIGINT; TERM is the reliable shutdown signal
trap 'exit 130' TERM INT

echo "=== [1/5] MicroXRCEAgent ==="
pkill -x MicroXRCEAgent 2>/dev/null || true
# stale gz instances make PX4 attach to an old world (drone GPS/height never
# converges, arming denied); always start from a fresh wind_farm world
pkill -f "gz sim" 2>/dev/null || true
pkill -f "ros_gz_bridge parameter_bridge" 2>/dev/null || true
sleep 1
MicroXRCEAgent udp4 -p 8888 -v 0 &
AGENT_PID=$!

echo "=== [2/5] PX4 SITL (world=wind_farm, model=gz_x500_mid360, pose=$MODEL_POSE) ==="
cd "$PX4_DIR"
if [ -n "$HEADLESS" ]; then export HEADLESS=1; fi
# setsid: give make its own process group so cleanup can kill the whole tree
# UXRCE_DDS_SYNCT=0
PX4_GZ_WORLD=wind_farm PX4_GZ_MODEL_POSE="$MODEL_POSE" setsid  make px4_sitl gz_x500_mid360 &
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

# bridge the gz sensor/odom/clock topics to ROS2 (rviz + tf broadcaster).
# Sensor topics are not model-scoped, so the drone's camera/lidar publish at
# /camera and /mid360 (single-drone world). The drone odometry is on a custom
# topic (the model's OdometryPublisher plugin redirects it away from the
# /model/.../odometry_with_covariance topic that PX4's gz_bridge consumes).
echo "=== [3/5] gz -> ROS2 bridges (clock, odom, lidar, camera) ==="
ros2 run ros_gz_bridge parameter_bridge \
    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
    /x500_mid360/odom_with_cov@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance \
    /mid360/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
    /mid360@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
    /camera@sensor_msgs/msg/Image[gz.msgs.Image \
    /camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo &
BRIDGE_PID=$!
sleep 2

echo "=== [4/5] TF broadcaster + RViz ==="
python3 -u "$FLOATGEN_SRC/scripts/gz_tf_broadcaster.py" &
TF_PID=$!
if [ -z "$HEADLESS" ]; then
    rviz2 -d "$FLOATGEN_SRC/config/rviz/wind_farm.rviz" --ros-args -p use_sim_time:=true &
    RVIZ_PID=$!
fi
sleep 3

echo "=== [5/5] wind_farm_planner ==="
python3 -u "$PLANNER" "$CONFIG"

echo "=== mission finished ==="
