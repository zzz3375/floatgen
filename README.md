# floatgen — PX4 wind-farm inspection simulation

Wind-farm assets and an inspection planner for **PX4 SITL + Gazebo (gz/Harmonic)**
with ROS 2 (Humble): a PX4 `x500` quadrotor takes off from a NREL-5MW wind
farm, flies a planned orbit around the first turbine (offboard position
control), then returns to launch (RTL) and lands.

The original package (URDF/xacro turbines, buoyancy world, `simple_launch`
farm launch) remains in `urdf/`, `launch/` and `meshes/`. The PX4 simulation
assets live under `gz/`.

## Layout

```
gz/
├── dds/loopback_fastdds.xml      # FastDDS profile (WSL2 loopback fallback)
├── models/wind_turbine/          # static NREL-5MW gz model (model://)
│   ├── model.config
│   ├── model.sdf                 # meshes authored in assembled frame -> identity poses
│   └── meshes/*.dae
└── worlds/wind_farm.sdf          # PX4 world (default.sdf template + 4 turbines + wind)
config/wind_farm.yaml             # single source of truth: layout, GPS origin, drone home, orbit
scripts/
├── wind_farm_planner.py          # ROS2 offboard inspection planner
└── wind_farm_bringup.sh          # one-shot: agent + PX4 SITL + planner
```

### Coordinate conventions

- The gz world is **ENU** (x=east, y=north, z=up). `wind_farm.sdf` places 4
  turbines at (0,0), (200,0), (100,200), (300,200) — the same grid formula as
  `urdf/farm.xacro` (scale 200, 2×2).
- PX4 local position is **NED** (x=north, y=east, z=down), home = the drone
  spawn point (`PX4_GZ_MODEL_POSE`). `config/wind_farm.yaml` holds the spawn
  (drone_home), GPS origin and orbit parameters; the planner converts ENU →
  NED internally.
- GPS origin is PX4's default Dübendorf coordinates (matching
  `<spherical_coordinates>` in the world).

## Requirements

- `~/PX4-Autopilot` v1.15 (built), Gazebo Harmonic `gz sim`, ROS 2 Humble.
- Workspace packages: `floatgen`, plus `px4_msgs` / `px4_ros_com`
  (use the `v1.15.4` tag of `px4_msgs` — `main` is not message-compatible
  with PX4 1.15).
- `MicroXRCEAgent` (v2.4.x) on PATH.
- `rmw_cyclonedds_cpp` (`sudo apt install ros-humble-rmw-cyclonedds-cpp`):
  required on this machine — FastDDS 2.6 (ROS 2 Humble) ↔ 2.14 (agent)
  discovery/data interop is broken here, and FastDDS discovery fails on WSL2.
  The bringup script sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

## One-shot bringup

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
bash src/floatgen/scripts/wind_farm_bringup.sh HEADLESS
```

Starts MicroXRCEAgent → PX4 SITL (`PX4_GZ_WORLD=wind_farm`,
`PX4_GZ_MODEL_POSE=-80,-25,0.5,0,0,0`, headless) → planner. The planner arms,
enters offboard, orbits the first turbine (12 waypoints, radius 85 m,
altitude 95 m, yaw towards the turbine), then RTL and lands.

### Manual steps

1. Symlink the assets into PX4 (only needed once; no PX4 source changes):

   ```bash
   ln -sfn ~/ros2_ws/src/floatgen/gz/worlds/wind_farm.sdf  ~/PX4-Autopilot/Tools/simulation/gz/worlds/wind_farm.sdf
   ln -sfn ~/ros2_ws/src/floatgen/gz/models/wind_turbine  ~/PX4-Autopilot/Tools/simulation/gz/models/wind_turbine
   ```

2. World standalone check:

   ```bash
   GZ_SIM_RESOURCE_PATH=~/ros2_ws/src/floatgen/gz/models gz sim -r ~/ros2_ws/src/floatgen/gz/worlds/wind_farm.sdf
   ```

3. PX4 SITL + bridge + planner (three terminals):

   ```bash
   cd ~/PX4-Autopilot
   HEADLESS=1 PX4_GZ_WORLD=wind_farm PX4_GZ_MODEL_POSE="-80,-25,0.5,0,0,0" make px4_sitl gz_x500

   MicroXRCEAgent udp4 -p 8888 -v 0

   source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ros2 topic list | grep fmu            # expect /fmu/out/vehicle_status ...
   python3 -u ~/ros2_ws/src/floatgen/scripts/wind_farm_planner.py
   ```

## Notes

- The turbine meshes are authored in the assembled turbine frame (tower base
  at model origin), so the static model places every visual/collision at an
  identity pose — no joint chain needed (and the URDF in `urdf/turbine.xacro`
  double-counts offsets that cancel out to the same result).
- The inspection orbit (radius 85 m) clears the 63 m rotor and stays below
  the 124 m tip height; the planner logs each waypoint for post-analysis.
- `gz/dds/loopback_fastdds.xml` is a FastDDS fallback profile (loopback-only)
  for cases where FastDDS must be used instead of CycloneDDS; it is not
  required with `rmw_cyclonedds_cpp`.
