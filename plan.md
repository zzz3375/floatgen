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
- [ ] 0.1 确认机体：默认 `x500`（四旋翼）。如需相机/深度选 `x500_depth`/`x500_vision`（见 `Tools/simulation/gz/models/`）。
- [ ] 0.2 确认巡检形态：绕单机环绕 / 沿整排风机走廊 / 对叶片近距离拍照航线。
- [ ] 0.3 选定**集成方案**（见 Phase 3，二者选一，推荐 A）：
  - **A：软链进 PX4 目录**（PX4 原生、最稳；对 PX4 仅加软链，不改源码）。
  - **B：standalone**（world 放 floatgen，手动起 gz，PX4 用 `PX4_GZ_STANDALONE`；不碰 PX4 但步骤多）。
- [ ] 0.4 定 GPS 原点（`spherical_coordinates` 的 lat/lon）与风机布局参数（`x,y,scale,nx,ny`），并记录成配置文件（Phase 4 的规划器要读同一份）。

## Phase 1 — 风机的 gz 模型化（解决 G2）
- [ ] 1.1 建目录 `src/floatgen/gz/models/wind_turbine/`，含 `model.config`、`model.sdf`、`meshes/`。
- [ ] 1.2 把现有 NREL-5MW 几何（tower/nacelle/hub/blade_1..3）转成**静态** gz 模型：`<static>true</static>`，visual+collision 用 `model://wind_turbine/meshes/*.dae`。
      - 可直接参考/复用 `urdf/turbine.xacro` 中的 mesh 位姿（各 link 的 `origin rpy/xyz`）。
      - 也可用 `xacro`+脚本把 URDF 转 SDF，但手写静态模型更可控。
- [ ] 1.3 把 `.dae`（及所需贴图）**复制或软链**进 `meshes/`（`base.py`/`.ive`/`.osg` 不需要）。
- [ ] 1.4（可选）叶片旋转动画：给 rotor joint 加 `gz::sim::systems::JointController`（参考 `turbine.xacro` 末尾写法）；注意与无人机的碰撞语义，巡检场景可先关掉。
- [ ] 1.5（可选）给叶片 `enable_wind=true`，配合 world 的 `<wind>`（见 `worlds/windy.sdf`）。
- [ ] 1.6 验证：`GZ_SIM_RESOURCE_PATH=<...>/gz/models gz sim -v 4 -r`（空 world + 手动插入 wind_turbine），确认网格加载、无 missing resource。

## Phase 2 — 风机农场 world SDF（解决 G1）
- [ ] 2.1 以 `~/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf` 为模板新建 `wind_farm.sdf`，**保留**其全部系统插件：
      `Physics/UserCommands/SceneBroadcaster/Contact/Imu/AirPressure/ApplyLinkWrench/NavSat/Sensors`，以及 `gravity`、`magnetic_field`、`atmosphere`、`gui`。
- [ ] 2.2 加 `<spherical_coordinates>`（WGS84/ENU + lat/lon/elevation）——**GPS 原点，必需**。
- [ ] 2.3 用 `<include><uri>model://wind_turbine</uri><pose>...</pose></include>` 按 floatgen 布局（`x,y,scale,nx,ny`）摆放多台风机；地面用 `ground_plane`（`enable_wind` 按需）。
- [ ] 2.4（可选）加 `<wind>`（参考 `worlds/windy.sdf`），模拟风场对无人机的扰动。
- [ ] 2.5 存为 `src/floatgen/gz/worlds/wind_farm.sdf`。
- [ ] 2.6 验证：`gz sim -r wind_farm.sdf`（配好 `GZ_SIM_RESOURCE_PATH`）能独立跑，风机可见、无报错。

## Phase 3 — PX4 集成（解决 G4，二选一）
### 方案 A（推荐）：软链进 PX4 目录
- [ ] 3A.1 `ln -s ~/ros2_ws/src/floatgen/gz/worlds/wind_farm.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/wind_farm.sdf`
- [ ] 3A.2 `ln -s ~/ros2_ws/src/floatgen/gz/models/wind_turbine ~/PX4-Autopilot/Tools/simulation/gz/models/wind_turbine`
- [ ] 3A.3 运行（**免改 CMakeLists**，用 env 覆盖 world）：
      ```
      cd ~/PX4-Autopilot
      PX4_GZ_WORLD=wind_farm PX4_GZ_MODEL_POSE="<x>,<y>,0,0,0,0" make px4_sitl gz_x500
      ```
- [ ] 3A.4（可选，想要专属 target）把 `wind_farm` 加进 `src/modules/simulation/gz_bridge/CMakeLists.txt` 的 `gz_worlds`，重配后即可 `make px4_sitl gz_x500_wind_farm`。
### 方案 B：standalone（不碰 PX4 目录）
- [ ] 3B.1 `export GZ_SIM_RESOURCE_PATH=<floatgen>/gz/models:<floatgen>/gz/worlds:~/PX4-Autopilot/Tools/simulation/gz/models:~/PX4-Autopilot/Tools/simulation/gz/worlds`
- [ ] 3B.2 手动起 world：`gz sim -r wind_farm.sdf`（可另开 `-g` GUI）。
- [ ] 3B.3 起 PX4 并接管已运行 world：
      ```
      cd ~/PX4-Autopilot
      PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=wind_farm PX4_SIM_MODEL=gz_x500 \
        PX4_GZ_MODEL_POSE="<x>,<y>,0,0,0,0" make px4_sitl gz_x500
      ```
- [ ] 3.x 共同验证：x500 出现在风机旁；`commander takeoff` / MAVLink offboard 能解锁起飞。

## Phase 4 — ROS2 通信 + 路径规划（解决 G3）
- [ ] 4.1 clone 进工作区：`px4_msgs`、`px4_ros_com`（Humble 分支）到 `~/ros2_ws/src`，`colcon build`。
- [ ] 4.2 起 DDS 桥：`MicroXRCEAgent udp4 -p 8888 -v`（与 PX4 SITL 默认对齐）。
- [ ] 4.3 确认话题打通：`ros2 topic list | grep fmu`（如 `/fmu/out/vehicle_status`）。
- [ ] 4.4 规划节点：读取 Phase 0.4 的风机坐标配置，生成巡检 waypoint（环绕单机 / 走廊），通过 `px4_ros_com` 的 offboard 接口（`TrajectorySetpoint`/`VehicleCommand`）下发。
- [ ] 4.5（可选）避障与感知：换 `x500_depth`/`x500_vision`，接深度/相机话题；或引入现成避障栈。
- [ ] 4.6（可选）用 QGroundControl 跑 mission 与 ROS2 规划二选一/结合。
- [ ] 4.7 端到端验证：起飞 → 按路径绕风机 → 返航降落；记录 RTF/轨迹。

## Phase 5 — 打包与一键启动
- [ ] 5.1 `floatgen/CMakeLists.txt` `install(DIRECTORY gz ...)`，让 `colcon build` 带上 world+models。
- [ ] 5.2 写 bringup：一条脚本起 `world + px4_sitl + MicroXRCEAgent + planner`（可用 `ros2 launch`）。
- [ ] 5.3 更新 `README`：目录结构、方案 A/B 命令、坐标系/GPS 原点说明。
- [ ] 5.4（可选）截图/录屏，归档到 `docs/`。

---

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
