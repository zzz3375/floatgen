# floatgen — PX4 wind-farm inspection simulation

Wind-farm assets and an inspection planner for **PX4 SITL + Gazebo (gz/Harmonic)**
with ROS 2 (Humble): a PX4 `x500_mid360` quadrotor (front camera + top
Livox Mid-360 style lidar) takes off from a NREL-5MW wind farm, flies a
planned orbit around the first turbine (offboard position control), then
returns to launch (RTL) and lands. The lidar scan / camera image / TF tree
are streamed to RViz2 via `ros_gz_bridge`.

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
├── models/x500_mid360/           # drone model (symlinked into PX4)
│   ├── model.config
│   └── model.sdf                 # x500 + camera + lidar + odom publisher
└── worlds/wind_farm.sdf          # PX4 world (default.sdf template + 4 turbines + wind)
config/
├── wind_farm.yaml                # single source of truth: layout, GPS origin, drone home, orbit
└── rviz/wind_farm.rviz           # RViz2 config (PointCloud2 /mid360/points, Image /camera, TF)
scripts/
├── wind_farm_bringup.sh          # one-shot: agent + PX4 SITL + bridges + RViz + planner
├── wind_farm_planner.py          # ROS2 offboard inspection planner
└── gz_tf_broadcaster.py          # TF: world -> drone -> sensor frames (from gz odometry)
```

The drone model `x500_mid360` (camera + `gpu_lidar` Mid-360 emulation +
odometry publisher) is **authoritative in this repo** under `gz/models/x500_mid360/`
and symlinked into the PX4 tree, because PX4's `gz_bridge` spawns models via
`model://` URIs:

```
gz/models/x500_mid360/{model.config, model.sdf}   ← source of truth
~/PX4-Autopilot/Tools/simulation/gz/models/x500_mid360 → symlink to above
~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4012_gz_x500_mid360
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

- `~/PX4-Autopilot` v1.15.4 (built), Gazebo Harmonic `gz sim` 8.x, ROS 2 Humble.
- Workspace packages: `floatgen`, plus `px4_msgs` / `px4_ros_com`
  (use the `v1.15.4` tag of `px4_msgs` — `main` is not message-compatible
  with PX4 1.15).
- `MicroXRCEAgent` (v2.4.3) on PATH.
- `rmw_cyclonedds_cpp` (`sudo apt install ros-humble-rmw-cyclonedds-cpp`):
  required on this machine — FastDDS 2.6 (ROS 2 Humble) ↔ 2.14 (agent)
  discovery/data interop is broken here, and FastDDS discovery fails on WSL2.
  The bringup script sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
- `ros_gz` for Harmonic (`ros-humble-ros-gzharmonic`, provides
  `ros2 run ros_gz_bridge parameter_bridge`).

## 从零配置项目

> 以下步骤在一台干净的 Ubuntu 22.04 机器上从零复现本项目的完整环境。

### 1. 系统依赖

```bash
# ROS 2 Humble（桌面版即可）
sudo apt update && sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions ros-dev-tools

# Gazebo Harmonic（gz-sim8）+ 配套工具
sudo apt install gz-harmonic

# ros_gz（Humble 对应 Harmonic 的桥接包）
sudo apt install ros-humble-ros-gzharmonic

# CycloneDDS（本项目必须，见"踩坑点"）
sudo apt install ros-humble-rmw-cyclonedds-cpp

# ROS 2 与 gz 环境变量
source /opt/ros/humble/setup.bash
```

### 2. MicroXRCEAgent（PX4 ↔ ROS2 的 uXRCE-DDS 桥）

```bash
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git ~/Micro-XRCE-DDS-Agent
cd ~/Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install   # 装到 /usr/local/bin/MicroXRCEAgent
```

### 3. PX4-Autopilot（v1.15）+ 无人机模型

```bash
git clone --recursive -b release/1.15 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
cd ~/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools   # 安装编译依赖（或按官方文档）
```

把无人机模型软链进 PX4（`gz_bridge` 通过 `model://` 找模型，模型必须在
PX4 的 `Tools/simulation/gz/models` 下）：

```bash
ln -sfn ~/ros2_ws/src/floatgen/gz/models/x500_mid360 \
        ~/PX4-Autopilot/Tools/simulation/gz/models/x500_mid360
```

`model.sdf` = `x500`（include 合并）+ 前向相机 `camera_link` + 顶部
Mid-360 模拟 `mid360_link`（`gpu_lidar`，高于 base_link 5 cm）+ OdometryPublisher
插件（**话题必须重定向**，见"踩坑点"）。模型的源文件在本项目
`gz/models/x500_mid360/`，修改请在本项目内操作。

注册空气框架（文件名即运行时匹配的 autostart id）：

```bash
# ROMFS/px4fmu_common/init.d-posix/airframes/4012_gz_x500_mid360:
#   PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500_mid360}
#   . ${R}etc/init.d-posix/airframes/4001_gz_x500
# 并在同目录 CMakeLists.txt 的 px4_add_romfs_files 列表中加入 4012_gz_x500_mid360
```

首次构建（会同时生成 `gz_x500_mid360` 的 make 目标）：

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_mid360        # 首次会编译 PX4，几分钟
```

### 4. ROS2 工作区

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <floatgen 仓库> floatgen
git clone -b v1.15.4 https://github.com/PX4/px4_msgs.git   # 必须 v1.15.4，main 与 PX4 1.15 消息不兼容
git clone https://github.com/PX4/px4_ros_com.git           # main 即可（本项目的规划器只用 px4_msgs）

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`floatgen` 的 `hooks/resource_paths.dsv.in` 会把安装目录的 `share`
自动 prepend 到 `GZ_SIM_RESOURCE_PATH`，因此 `wind_farm.sdf` 与
`wind_turbine` 模型对 PX4 启动的 gz 可见。

### 5. 世界文件与风电机组模型

本项目约定：`gz/worlds/wind_farm.sdf` 与 `gz/models/wind_turbine` 软链进
PX4（PX4 启动 gz 时从自己的 `Tools/simulation/gz/{worlds,models}` 找资源）：

```bash
ln -sfn ~/ros2_ws/src/floatgen/gz/worlds/wind_farm.sdf  ~/PX4-Autopilot/Tools/simulation/gz/worlds/wind_farm.sdf
ln -sfn ~/ros2_ws/src/floatgen/gz/models/wind_turbine   ~/PX4-Autopilot/Tools/simulation/gz/models/wind_turbine
```

### 6. 启动

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
bash src/floatgen/scripts/wind_farm_bringup.sh          # 带 GUI（gz + RViz）
bash src/floatgen/scripts/wind_farm_bringup.sh HEADLESS # 无 GUI（gz 与 RViz 均不开）
```

脚本按顺序拉起：MicroXRCEAgent → PX4 SITL（`gz_x500_mid360`，
world=wind_farm，spawn=-80,-25,0.5）→ 6 条 gz↔ROS2 桥 → TF 广播 + RViz →
巡检规划器。任务结束后自动清理全部进程。

## One-shot bringup

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
bash src/floatgen/scripts/wind_farm_bringup.sh HEADLESS
```

Starts MicroXRCEAgent → PX4 SITL (`PX4_GZ_WORLD=wind_farm`,
`PX4_GZ_MODEL_POSE=-80,-25,0.5,0,0,0`, headless) → gz↔ROS2 bridges →
TF broadcaster + RViz → planner. The planner arms (retrying until the EKF
health checks pass), enters offboard, orbits the first turbine (12 waypoints,
radius 85 m, altitude 95 m, yaw towards the turbine), then RTL and lands.

## 核心架构

### 数据流

```
┌─ PX4 SITL 进程（make px4_sitl gz_x500_mid360）─────────────────────┐
│  gz sim（wind_farm.sdf）                                            │
│   └─ x500_mid360 模型（model:// 加载，实体名 x500_mid360_0）        │
│        ├─ camera 传感器    → /camera         （gz.msgs.Image）       │
│        ├─ mid360 gpu_lidar → /mid360         （gz.msgs.LaserScan）  │
│        │                     /mid360/points （gz.msgs.PointCloudPacked）
│        └─ OdometryPublisher → /x500_mid360/odom_with_cov（自定义话题，见踩坑点4）
│  px4（gz_bridge 模块）                                              │
│   ├─ uXRCE-DDS ←→ MicroXRCEAgent ←→ ROS2 /fmu/out/* 话题           │
│   └─ IMU/气压/GPS/电机指令 经 gz 话题与 gz 交互（话题带 _0 实体名）  │
└─────────────────────────────────────────────────────────────────────┘
        │ gz 话题（transport13）
        ▼
ros_gz_bridge parameter_bridge（一条进程，6 条桥）
  /clock                → rosgraph_msgs/Clock
  /x500_mid360/odom_with_cov → nav_msgs/Odometry（OdometryWithCovariance 类型）
  /mid360/points        → sensor_msgs/PointCloud2
  /mid360               → sensor_msgs/LaserScan
  /camera               → sensor_msgs/Image
  /camera_info          → sensor_msgs/CameraInfo
        │
        ├─→ gz_tf_broadcaster.py → /tf（动态 world→模型 + 静态传感器帧）
        ├─→ rviz2（config/rviz/wind_farm.rviz，固定系 world，sim time）
        └─→ wind_farm_planner.py（/fmu/in/{offboard_control_mode,trajectory_setpoint,vehicle_command}）
```

### TF 树

```
world（gz ENU 世界系）
 └─ x500_mid360                    动态：来自 odometry（模型世界位姿）
    └─ x500_mid360/base_link       静态 +z=0.24（x500_base 的模型位姿）
       ├─ x500_mid360/camera_link  静态 (0.12, 0, 0.002)
       │   └─ x500_mid360_0/camera_link/camera    恒等（gz 传感器帧名）
       └─ x500_mid360/mid360_link  静态 (0, 0, 0.05)
           └─ x500_mid360_0/mid360_link/mid360    恒等（gz 传感器帧名）
```

要点：gz 的 OdometryPublisher 报的是**模型原点**的世界位姿，而 base_link 因
`x500_base` 的 `<pose>0 0 .24</pose>` 在模型系 +0.24，所以中间插一个
`model → base_link` 静态帧，由 TF 组合正确分解旋转（无需手动四元数合成）。
传感器数据帧（`<model>::<link>::<sensor>`，桥接后 `::`→`/`）通过恒等静态
帧挂到对应 link 下。

### 规划器状态机

```
STREAM → ARM → OFFBOARD → FLY（12 个航点）→ RTL → LAND → DONE
  悬停  每2s重试解锁  切offboard  轨道巡检       返航降落
```

全部用 PX4 offboard 位置指令（NED 系），home = 出生点；解锁在健康检查未就绪
时每 2 s 重试（最长 30 s），避免 EKF 收敛前一次性 ARM 被拒。

## 踩坑点

1. **gz 传感器 `<topic>` 不带模型名前缀**：`<topic>camera</topic>` 发布在
   `/camera` 而不是 `/x500_mid360/camera`。单机没问题；多机同时跑必冲突，
   需手动加命名空间。

2. **link 的 `<pose>` 是相对 model 坐标系，不是相对父 link**：
   `base_link` 因为 `x500_base` 的 `<pose>0 0 .24</pose>` 在模型系 z=0.24。
   挂载传感器时按模型系写：`mid360_link` 要在 base_link 上方 0.05，就得写
   `0 0 0.29`（0.24+0.05）。TF 广播脚本里的偏移也要按 base_link 相对算。

3. **PX4 以 `<model>_<instance>` 名字 spawn（x500_mid360_0）**，且通过
   `model://` URI 加载时，gz 传感器的 frame_id 会带上这个实体名：
   `x500_mid360_0::mid360_link::mid360` → 桥接后
   `x500_mid360_0/mid360_link/mid360`。TF 静态帧必须匹配这个带 `_0` 的名字
   （用绝对路径 sdf 加载时反而没有 `_0`，行为不一致，务必以实际运行验证为准）。

4. **OdometryPublisher 插件会毒化 PX4 的 EKF（本项目最大的坑）**：
   PX4 的 `gz_bridge` 无条件订阅 `/model/<名字>/odometry_with_covariance`
   并把数据灌进 `vehicle_odometry`。该插件发的协方差全是 0，等于让滤波器
   无限信任这个量测 → East velocity 数值错误、轨迹发散（飞到 z=-286 m）、
   罗盘/姿态预检失败、触发 failsafe。**必须**用 `<odom_covariance_topic>` /
   `<odom_topic>` 把话题重定向到自定义名字（如 `/x500_mid360/odom_with_cov`），
   gz_bridge 的默认话题就没人发布，PX4 不受影响。TF 用重定向后的话题即可。

5. **残留 gz 进程会让 PX4 附着旧世界**：日志出现
   `gazebo already running world: wind_farm` 时，PX4 不会重新启动 gz，而是
   直接加入。旧世界里 EKF 原点可能错位（本机曾出现原点在世界原点、距无人机
   84 m），导致 GPS 漂移检查不过、`height estimate not stable`、拒绝解锁。
   解决：每次启动前杀掉残留 gz（bringup 脚本第 1 步已 `pkill -f "gz sim"`）。

6. **桥接类型写错会静默无数据**：gz 的 OdometryPublisher 发的是
   `gz.msgs.OdometryWithCovariance`，不是 `gz.msgs.Odometry`；桥参数里写错
   类型桥能创建但收不到数据（订阅类型不匹配）。

7. **`nav_msgs/Odometry` 的 position 是 `Point`，`TransformStamped.translation`
   是 `Vector3`**：两者类型不同不能直接赋值，会抛
   `AssertionError: must be a sub message of type 'Vector3'`，需逐字段拷贝。

8. **Harmonic 下 CPU 版 `lidar` 传感器不发布话题**：用 `gpu_lidar`。它同时
   发布 LaserScan（`<topic>`）和 PointCloudPacked（`<topic>/points`），
   Mid-360 仿真用后者（带 x/y/z/intensity/ring 字段）。

9. **RViz2 的 PointCloud2 "Point Style" 是枚举序号**：0=Points、1=Squares、
   2=Flat Squares … 5=Boxes（`rviz_rendering::PointCloud::RenderMode`）。
   手写 .rviz 时别写错（写 8 会被忽略）。

10. **`model://` 加载需要 model.config**：模型目录只有 model.sdf 时 gz 报
    `Could not find model.config`，必须在目录里补一个。

11. **解锁时机**：EKF 需要数秒收敛（GPS/高度融合、磁航向），一次性 ARM 大概率
    被健康检查拒绝且不会自动重试。规划器已改为每 2 s 重发 ARM、30 s 超时。

12. **WSL2 下 ROS2 发现不稳定**：必须 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`；
   新终端里 `ros2 topic list` 空时先 `ros2 daemon start`。FastDDS（Humble 自带）
   在 WSL2 上发现失败、且与 agent 的 FastDDS 2.14 不互通。

13. **`pxh>` 提示符刷屏**：把 PX4 的 stdin 重定向到 /dev/null（自动化测试）时，
   其 shell 会高频重绘 `\033[2K\r pxh>` 刷爆日志；交互终端运行无此问题。

14. **gz GUI 里看不到传感器数据/射线**：`<visualize>true</visualize>` 控制
    传感器可视化；gpu_lidar 的射线与相机画面可视化有额外渲染开销，在低配
    环境（如无 GPU 的 WSL2）会拖慢实时因子，可关掉换取 RTF。

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
