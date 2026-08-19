# 计划：把 floatgen 打包成「PX4 无人机风机巡检 + 路径规划」仿真环境（world SDF）

> 目标：产出一个可被 PX4 SITL 直接加载的 `wind_farm.sdf` world，让一架 PX4 无人机（默认 `x500` 四旋翼）
> 在 NREL-5MW 风机农场中起飞、按规划路径绕机巡检、返航。路径规划由 ROS2 节点完成。
>
> 本文件只是 **TODO 计划**（不执行）。执行时按 Phase 顺序勾选。

---

## 0. 现状盘点（已核实，基于本机 `~/PX4-Autopilot/`）

| 项 | 现状 |
|----|------|
| PX4 | `~/PX4-Autopilot`，**v1.15.4**（`git describe`），SITL 用新版 **gz**（非 gazebo-classic） |
| Gazebo | `gz sim --versions` = **8.11.0**（Harmonic），与 v1.15 匹配 |
| ROS | **Humble**（`$ROS_DISTRO=humble`），工作区 `~/ros2_ws` |
| floatgen | 风机 xacro/URDF（上一轮已改为**落地式** NREL-5MW：tower→nacelle→hub→3 blades，无浮式基础）+ `urdf/floatgen_world.sdf`（含浮力/水面，**缺 PX4 必需项**） |
| MicroXRCEAgent | 已编译并安装：`/usr/local/bin/MicroXRCEAgent`（`~/Micro-XRCE-DDS-Agent/build/`） |
| px4_msgs / px4_ros_com | **尚未**存在于 `~/ros2_ws/src`（路径规划需要，Phase 4 要 clone） |

### PX4↔gz 的启动链路（关键机制，务必理解后再动手）
1. `make px4_sitl gz_x500` → 构建并运行 `px4`，置 `PX4_SIM_MODEL=gz_x500`。
2. `px4` 的启动脚本 `ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator`：
   - 读取 `PX4_GZ_WORLD`（默认 `default`），source `gz_env.sh` 后执行
     `gz sim -r -s ${PX4_GZ_WORLDS}/${PX4_GZ_WORLD}.sdf`（`-s` 服务器，非 `HEADLESS` 时再 `-g` 起 GUI）。
   - 再 `gz_bridge start -p <pose> -m x500 -w <world> -i <instance>` **自动生成无人机**并接管。
3. `gz_env.sh`（生成于 `build/px4_sitl_default/rootfs/gz_env.sh`）**硬编码**：
   - `PX4_GZ_MODELS=~/PX4-Autopilot/Tools/simulation/gz/models`
   - `PX4_GZ_WORLDS=~/PX4-Autopilot/Tools/simulation/gz/worlds`
   - 并把二者追加进 `GZ_SIM_RESOURCE_PATH`。
   → **结论：要让 PX4 自动起 world，world/模型必须位于（或软链到）PX4 的 `Tools/simulation/gz/` 下。**
4. world 列表与 target 生成：`src/modules/simulation/gz_bridge/CMakeLists.txt` 里
   `set(gz_worlds default windy baylands lawn)`，对每个机体×world 生成 `gz_<model>_<world>` target
   （如 `gz_x500_windy`）。`default` 对应裸 `gz_x500`。

### 可用环境变量（都在 px4-rc.simulator 中读取）
- `PX4_GZ_WORLD`：world 名（不含 `.sdf`）。`gz_x500` target 不写死 world，故 `PX4_GZ_WORLD=wind_farm make px4_sitl gz_x500` 可**免改 CMakeLists** 直接换 world。
- `PX4_GZ_MODEL_POSE`：无人机出生位姿 `x,y,z,roll,pitch,yaw`（默认原点）。
- `PX4_GZ_STANDALONE`：置位后 PX4 **不自启** gz，等你手动把 world 跑起来（用于「world 放在别处」的方案 B）。
- `PX4_GZ_MODEL_NAME`：让 gz_bridge 挂到一个**已存在**的模型而非新 spawn。
- `HEADLESS`：置位则不起 GUI。

---

## 关键差距（必须解决，对应下面各 Phase）
1. **G1** `floatgen_world.sdf` 缺 PX4 必需项：`NavSat`/`AirPressure`/`Contact` 等插件、`spherical_coordinates`（GPS 原点）、`magnetic_field`，且物理步长与 PX4 不一致。**不能直接给 PX4 用**，需以 PX4 `default.sdf` 为模板重建。
2. **G2** 网格引用是 `package://floatgen/meshes/...`（URDF）/ `file://$(find floatgen)/...`，**gz 不解析 `package://`**。须重排为标准 gz model，用 `model://` + `GZ_SIM_RESOURCE_PATH`。
3. **G3** 缺 `px4_msgs` / `px4_ros_com`，ROS2 侧暂无 offboard/规划能力。
4. **G4** 无人机出生点、风机坐标、GPS 原点三者需在同一坐标系下对齐，规划器才能拿到正确的巡检点。

---

## Phase 0 — 决策与前置
- [x] 0.1 确认机体：默认 `x500`（四旋翼）。
- [x] 0.2 确认巡检形态：绕单机环绕（12 waypoint，半径 85m，高度 95m）。
- [x] 0.3 选定集成方案：**方案 A（软链进 PX4 目录）**。
- [x] 0.4 定 GPS 原点（沿用 PX4 default 坐标）与风机布局（2×2、scale 200），记录于 `config/wind_farm.yaml`。

## Phase 1 — 风机的 gz 模型化（解决 G2）
- [x] 1.1 建目录 `src/floatgen/gz/models/wind_turbine/`，含 `model.config`、`model.sdf`、`meshes/`。
- [x] 1.2 NREL-5MW 几何转成**静态** gz 模型：`<static>true</static>`，visual+collision 用 `model://wind_turbine/meshes/*.dae`。
      - 注意：各 mesh 本身即“装配后的全局坐标系”（塔底为原点），故 SDF 中所有 visual/collision 用 identity pose；URDF 里 joint+visual 偏移相互抵消等价于该结果。
- [x] 1.3 `.dae` 复制进 `meshes/`。
- [ ] 1.4（可选）叶片旋转动画 —— 巡检场景保持静态，未做。
- [ ] 1.5（可选）叶片 `enable_wind` —— 未做。
- [x] 1.6 验证：headless 渲染截图确认网格加载、装配正确、无报错。

## Phase 2 — 风机农场 world SDF（解决 G1）
- [x] 2.1 以 PX4 `default.sdf` 为模板新建 `wind_farm.sdf`，**保留**全部系统插件与 `gravity`、`magnetic_field`、`atmosphere`、`gui`。
- [x] 2.2 加 `<spherical_coordinates>`（沿用 PX4 default 的 lat/lon/elevation）。
- [x] 2.3 4 台风机按 farm.xacro 布局 include：(0,0)、(200,0)、(100,200)、(300,200)。
- [x] 2.4 加 `<wind>`（3 m/s +x）。
- [x] 2.5 存为 `src/floatgen/gz/worlds/wind_farm.sdf`。
- [x] 2.6 验证：headless 独立运行，风机可见、无报错。

## Phase 3 — PX4 集成（解决 G4，方案 A）
- [x] 3A.1 软链 world 进 PX4 目录。
- [x] 3A.2 软链 wind_turbine model 进 PX4 目录。
- [x] 3A.3 `PX4_GZ_WORLD=wind_farm PX4_GZ_MODEL_POSE="-80,-25,0.5,0,0,0" make px4_sitl gz_x500`：world 自动加载、x500 出生在指定位置、PX4 完整启动。
- [ ] 3A.4（可选）专属 target —— 未做（env 覆盖已够用，保持 PX4 上游干净）。
- [x] 3.x 验证：x500 出现在风机旁（4 台风机+无人机同 world），offboard 起飞/巡航由 Phase 4 端到端验证。

## Phase 4 — ROS2 通信 + 路径规划（解决 G3）
- [x] 4.1 clone 进工作区：`px4_msgs`（**v1.15.4 tag**，main 与 PX4 1.15.4 消息不兼容）、`px4_ros_com`（main）到 `~/ros2_ws/src`，`colcon build`。
- [x] 4.2 起 DDS 桥：`MicroXRCEAgent udp4 -p 8888`（v2.4.3，与 PX4 SITL 对齐）。
- [x] 4.3 确认话题打通：`/fmu/out/vehicle_status` 等 43 个 fmu 话题可见（**须用 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`**，见下“异常”）。
- [x] 4.4 规划节点：`scripts/wind_farm_planner.py`（rclpy + px4_msgs 话题：OffboardControlMode/TrajectorySetpoint/VehicleCommand，直接等价于 px4_ros_com 的 offboard 接口），读取 `config/wind_farm.yaml` 生成环绕 waypoint。
- [ ] 4.5（可选）避障与感知 —— 未做。
- [ ] 4.6（可选）QGroundControl —— 未做。
- [x] 4.7 端到端验证：解锁 → offboard → 12 waypoint 绕风机 → RTL 返航 → 降落 → 解锁（多次完整跑通）。

## Phase 5 — 打包与一键启动
- [x] 5.1 `floatgen/CMakeLists.txt` `install(DIRECTORY gz config ...)` + planner 脚本安装，`colcon build` 验证。
- [x] 5.2 bringup：`scripts/wind_farm_bringup.sh`（agent + px4_sitl + planner 一键，HEADLESS 可选；正常路径端到端跑通并自动清理进程树）。
- [x] 5.3 更新 `README`：目录结构、命令、坐标系/GPS 原点说明。
- [ ] 5.4（可选）截图/录屏 —— 未做（headless 环境，改为日志/位姿验证）。

---

## 执行异常 / 偏差记录（2026-08-18 执行）
1. **px4_msgs 版本**：`main` 分支的 `VehicleCommand`/`VehicleLocalPosition`/`VehicleStatus` 等与 PX4 v1.15.4 固件消息**不兼容**（字段/常量漂移）；改用 `v1.15.4` tag（与固件逐字段一致）。px4_ros_com 无 release 分支，用 main。
2. **DDS 不可用（WSL2）**：本机是 WSL2（eth0 DOWN + FastDDS 发现失败），且 ROS2 Humble FastDDS 2.6 ↔ Agent FastDDS 2.14 **发现后无法传数据**。解决：`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`（sudo apt install ros-humble-rmw-cyclonedds-cpp），并预启动 `ros2 daemon`；`gz/dds/loopback_fastdds.xml` 保留作 FastDDS 回退方案。
3. **坐标系**：gz world 为 ENU（x=东,y=北），PX4 本地为 NED（x=北,y=东）——规划器 ENU→NED 转换必须 `(y,x,-z)`，首版按 `(x,y,-z)` 写错导致环绕圆心偏移（已在模拟中肉眼/位姿验证后修正）。
4. **planner 细节**：PX4 落地后本地 z 参考有 ~2m 漂移，RTL 后以 `arming_state==DISARMED` 判定落地；任务结束后节点主动退出（spin_once + finished 标志），否则 bringup 脚本不会结束。
5. **bringup 脚本**：`set -u` 与 ROS setup.bash（读未定义 `AMENT_TRACE_SETUP_FILES`）冲突；`setsid` 不能带 `VAR=val` 前缀；后台树清理用 `setsid`+`kill -- -PID`；`ros2 topic list` 需 `timeout -k` 防挂死。

## 参考定位（均来自 `~/PX4-Autopilot/`，已核实）
| 用途 | 路径 |
|------|------|
| gz 启动脚本 / env 变量 | `ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator` |
| x500 机体参数 | `ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500` |
| world 列表 + target 生成 | `src/modules/simulation/gz_bridge/CMakeLists.txt` |
| `PX4_GZ_WORLDS/MODELS/RESOURCE_PATH` | `build/px4_sitl_default/rootfs/gz_env.sh`（模板 `src/modules/simulation/gz_bridge/gz_env.sh.in`） |
| world 模板（插件+GPS） | `Tools/simulation/gz/worlds/default.sdf` |
| wind 参考 | `Tools/simulation/gz/worlds/windy.sdf` |
| 无人机模型 | `Tools/simulation/gz/models/x500/`（含 `model.config`/`model.sdf`） |

## 风险 / 注意
- **GPS 一致性**：`spherical_coordinates` 决定 home/原点；风机布局坐标、`PX4_GZ_MODEL_POSE`、规划器 waypoint 必须同一 ENU 原点，否则位置整体偏移。
- **`package://` 不被 gz 解析**：world/model 里一律用 `model://`（配 `GZ_SIM_RESOURCE_PATH`）或绝对 `file://`。
- **改 PX4 源码（方案 A.4 的 CMakeLists）会让 `~/PX4-Autopilot` 变脏**；优先用软链 + `PX4_GZ_WORLD` env，保持上游干净、便于升级。
- 旋转叶片若保留物理，可能与近距离无人机产生碰撞；巡检航线建议先关叶片物理或保持安全距离。
