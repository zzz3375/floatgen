# 计划：从 PX4 仿真（SITL）迁移到真机实飞

> 目标：把当前「gz 仿真 + PX4 SITL + ROS2 巡检规划」这套系统迁移到真实无人机
> （带前向相机 + Livox Mid-360 激光雷达），在真实风电场执行同样的环绕巡检任务。
>
> 本文件先做**架构分析**，再列出**真机迁移工作清单**（按子系统 + 分阶段）。

---

## 一、现状架构分析

### 1.1 总体架构（SITL 现状）

```
┌─────────────────── 仿真层（gz sim 8.x / Harmonic）───────────────────┐
│ wind_farm.sdf（4 台 NREL-5MW 风机 + wind 3m/s + Dübendorf GPS 原点） │
│  └─ x500_mid360 模型（model://，实体名 x500_mid360_0）               │
│       ├─ camera 传感器      → /camera、/camera_info                  │
│       ├─ mid360 gpu_lidar   → /mid360、/mid360/points                │
│       └─ OdometryPublisher  → /x500_mid360/odom_with_cov（重定向话题）│
└────────────────────────────┬──────────────────────────────────────────┘
              gz 话题（transport13）│  IMU/气压/GPS/电机经 gz_bridge 交互
┌────────────────────────────▼──────────────────────────────────────────┐
│ 飞控层：PX4 SITL（px4 进程 + gz_bridge 模块，airframe 4012）           │
│   └─ uXRCE-DDS ←─ UDP 回环 ─→ MicroXRCEAgent ←─ DDS ─→ ROS2 /fmu/*   │
└────────────────────────────────┬──────────────────────────────────────┘
                                 │ ros_gz_bridge parameter_bridge（6 条桥）
                                 ▼
┌─────────────────── 应用层（ROS2 Humble + CycloneDDS）─────────────────┐
│  ├─ gz_tf_broadcaster.py  → /tf（world→模型→base_link→传感器帧）      │
│  ├─ rviz2（config/rviz/wind_farm.rviz，固定系 world）                 │
│  └─ wind_farm_planner.py  → offboard 位置指令（12 航点环绕巡检）       │
└───────────────────────────────────────────────────────────────────────┘
```

### 1.2 组件与职责（迁移视角：留用 / 替换 / 改造）

| 组件 | 位置 | SITL 中的职责 | 真机迁移 |
|------|------|--------------|----------|
| gz world + 风机模型 | `gz/worlds`、`gz/models` | 环境仿真、GPS 原点、风场 | **替换**为真实风电场 + 实测风机坐标 |
| x500_mid360 SDF | PX4 `Tools/simulation/gz/models` | 机体动力学 + 传感器仿真 | **替换**为真实机架；SDF 仅作外参参考 |
| gz camera / gpu_lidar | 模型 SDF | 传感器数据仿真（帧名带 `_0`） | **替换**为真实相机 / 真实 Mid-360 驱动 |
| OdometryPublisher 插件 | 模型 SDF | 世界位姿 → TF（话题已重定向防毒化 PX4） | **删除**；TF 改用 PX4 EKF 输出 |
| PX4 SITL + gz_bridge | PX4 进程 | 飞控逻辑 + 传感器/电机桥 | **替换**为真机固件 + 真实外设 |
| MicroXRCEAgent | 独立进程 | PX4↔ROS2 消息桥（UDP 回环） | **改造**：传输层换串口/WiFi |
| `ros_gz_bridge` | 独立进程 | gz 话题→ROS2 话题 | **替换**为各真传感器驱动 |
| `gz_tf_broadcaster.py` | floatgen | gz odometry→TF（world 系） | **改造**：EKF 里程计→TF（map 系） |
| `wind_farm_planner.py` | floatgen | offboard 位置指令巡检 | **改造**：速度控制 + 安全逻辑 + 真实坐标 |
| `wind_farm_bringup.sh` | floatgen | 一键拉起 SITL 全链路 | **改造**：换成真机启动脚本 |
| `config/wind_farm.yaml` | floatgen | 单一事实源（布局/GPS 原点/航线） | **改造**：填入真实风机坐标与 GPS |

### 1.3 关键接口（迁移时保持稳定的契约）

| 接口 | 现状 | 真机变化 |
|------|------|----------|
| PX4 控制话题（px4_msgs） | `OffboardControlMode` / `TrajectorySetpoint` / `VehicleCommand`（/fmu/in/*） | **不变**（px4_msgs v1.15.4 与真机固件同版本） |
| PX4 状态话题 | `/fmu/out/vehicle_status`、`vehicle_local_position`、`vehicle_odometry` | **不变**，且真机上 EKF 数据同样经 uXRCE-DDS 发布 |
| 传感器 ROS2 话题 | `/camera`(Image)、`/mid360/points`(PointCloud2) | **保持话题名/类型**，来源换成真驱动 |
| TF 框架 | `world → x500_mid360 → base_link → 传感器` | 框架保留，根系从 gz world 换为 EKF 本地原点（map/odom） |
| 坐标系 | 世界 ENU（Dübendorf 原点）、PX4 本地 NED | 世界原点换为**真实风电场 GPS**；NED 约定不变 |
| 配置 | `wind_farm.yaml`（drone_home、turbines、inspection） | 字段保留，数值换真实测量值 |

### 1.4 坐标系与配置基线（真机前必须改的"数值"）

- `config/wind_farm.yaml`：`gps_origin`（lat/lon/elev）与 `turbines[].x/y`（ENU 米）目前是仿真坐标系
  （Dübendorf + 2×2 网格），真机必须换成：实测风机经纬度 + RTK 测量的 ENU 原点。
- `drone_home`：仿真为出生点（-80,-25,0.5）；真机为起飞坪 GPS。
- 规划器 ENU→NED 换算逻辑（`(y, x, -z)` 相对 home）**可复用**，只要输入的数值正确。

---

## 二、迁移工作清单

> 勾选式清单，按子系统分组；[x] = 建议已在仿真侧完成/可复用的部分。

### A. 硬件与机载系统
- [ ] A.1 确定机架：Holybro X500（与仿真一致）或等效四旋翼；核对电调/电机/桨与 airframe 参数。
- [ ] A.2 飞控选型（Pixhawk 6C / Cube / Holybro 等，PX4 1.15 支持、双冗余 IMU 优先）。
- [ ] A.3 伴飞电脑（Jetson Orin Nano / RPi5 等）：性能需满足相机+点云+规划实时运行。
- [ ] A.4 电源树：电池→飞控/电调/伴飞电脑（稳压）/相机/Mid-360（12V），安全电流与散热设计。
- [ ] A.5 传感器安装：前向相机（SDF 参考位姿 0.12,0,0.242）、机顶 Mid-360（0,0,0.35 上方
      需净空，避开桨盘）；安装支架与减震。
- [ ] A.6 结构验证：重心、重量、桨保护、GPS 罗盘远离电机磁场（最好桅杆）。

### B. 飞控固件与机体配置
- [ ] B.1 刷入 PX4 1.15.4 固件（与 px4_msgs v1.15.4 严格同版本）。
- [ ] B.2 真机 airframe：选 x500（4001 的参数移植到真机 airframe id，如 4001 或自定义 5001），
      电机映射/转向/旋向与机架一致；`SIM_GZ_*` 参数不适用真机，需清理。
- [ ] B.3 校准：传感器方向、加速度计、陀螺仪、**磁罗盘**（真机必须，仿真无此坑）、水平。
- [ ] B.4 GPS：外部 GPS+罗盘，检查 HDOP/星数；**建议 RTK**（风机巡检对位置精度要求高）。
- [ ] B.5 电机/电调：ESC 校准、电机方向测试、螺旋桨动平衡。
- [ ] B.6 电池与电源 failsafe、电压/电流传感器接入。

### C. PX4 ↔ ROS2 通信链路
- [ ] C.1 传输层改造：`MicroXRCEAgent udp4`（回环）→ **串口**：飞控 TELEM2 → 伴飞电脑
      `MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600`（或 460800/230400，按线材定）。
- [ ] C.2 或用 WiFi（飞控 `PX4_WIFI_*` + 伴飞电脑）——可靠性不如串口，建议串口主链路。
- [ ] C.3 PX4 侧 uXRCE-DDS 参数（端口/串口波特率、DDS key），与 agent 对齐。
- [ ] C.4 验证：`/fmu/out/vehicle_status` 等话题在真机链路下可见（复用 bringup 的等待逻辑）。
- [ ] C.5 DDS 网络：若伴飞电脑与地面站/开发机跨机通信，配 CycloneDDS 网络配置文件。

### D. 飞行控制与安全（真机最关键）
- [ ] D.1 **offboard 控制模式**：SITL 用位置 setpoint 可直接复用到真机（EKF 位置源=GPS），
      但建议增加**速度 setpoint** 控制与限速/限加速度（位置跳变时不会猛冲）。
- [ ] D.2 **解锁策略改造**：仿真里规划器自动 ARM+重试；真机必须改为——
      等待人工/地面站解锁（或仅在测试模式自动解锁）；规划器在 `arming_state==ARMED`
      后再进入 offboard，不主动抢控制权。
- [ ] D.3 offboard 超时 failsafe：配置 `COM_OB_LOSS_T` 等（指令丢失→切 RTL/回人工），
      并让规划器持续以 ≥2Hz 发布 setpoint。
- [ ] D.4 RC 链路：绑定遥控器，验证 手动/自稳/定高/返航 各模式切换与**急停**。
- [ ] D.5 地理围栏（GEOFENCE）、最大高度/半径限制、RTL 高度。
- [ ] D.6 告警与日志：真机 `ulog` 全量记录；QGroundControl 联调（巡检时作为监控/干预台）。

### E. 传感器接入（相机 / Mid-360）
- [ ] E.1 相机驱动：USB/GStreamer 采集，发布到 `/camera`（sensor_msgs/Image）——
      与仿真话题对齐；`camera_info` 用真实内参标定（棋盘格/OpenCV，仿真无内参概念）。
- [ ] E.2 Mid-360 驱动：`livox_ros_driver2`（Humble 版），发布
      `livox_msgs/CustomMsg` 或转 `sensor_msgs/PointCloud2` 到 `/mid360/points`。
- [ ] E.3 **外参标定**：相机/Mid-360 相对 base_link 的 6DOF 外参（仿真里直接抄 SDF，
      真机必须实测/标定），并更新 TF 静态帧。
- [ ] E.4 时间同步：相机触发/时间戳、lidar 时间戳与飞控时钟对齐（PTP 或基于 GPS 授时），
      为后续感知融合打底。
- [ ] E.5 真机数据验证：RViz 里点云/图像帧率、范围、噪声与仿真对比（Mid-360 非重复扫描
      点云随时间累积，显示/算法需适配）。

### F. 定位与 TF（坐标系迁移）
- [ ] F.1 世界系：把 `wind_farm.yaml` 的 `gps_origin` 换成真实风电场坐标；EKF 本地原点
      在上电时由 GPS 设定——规划器以该原点算 NED 航点（逻辑不变）。
- [ ] F.2 **TF 源替换**：删除 gz OdometryPublisher 依赖，`gz_tf_broadcaster.py` 改为订阅
      `/fmu/out/vehicle_odometry`（px4_msgs/VehicleOdometry，真机经 uXRCE 发布）：
      NED→ENU 换算（home 已知）+ 四元数转换 → `map → base_link`；静态外参帧不变。
- [ ] F.3 遵循 REP-105：`map`（GPS 原点）→ `odom`（EKF 漂移系）→ `base_link`，
      便于后续接入定位重定位（RTK 切换、视觉定位）。
- [ ] F.4 风机坐标：用 RTK/测绘实测每台风机塔基经纬度 → 填入 yaml（巡检圆心/半径逻辑复用）。

### G. 规划器改造
- [ ] G.1 控制指令：位置 setpoint → **速度 setpoint**（或位置+速度混合），带限速/限加速度/
      最大倾斜角，避免仿真到真机的动力学差异导致过冲。
- [ ] G.2 安全监控：每个控制周期检查 EKF 状态（`estimator_status` 故障位）、GPS 有效性、
      位置发散、offboard 超时 → 立即切换 RTL 或放弃控制并告警。
- [ ] G.3 解锁/接管：改为"等待 ARMED 再动作"；支持地面站随时接管。
- [ ] G.4 航点与返航：真实风场航线（考虑风机尾流湍流，见风险）；返航复用 PX4 RTL +
      落地判定（仿真已实现 DISARM 判定，可复用）。
- [ ] G.5 参数化：`wind_farm.yaml` 增加真机参数节（限速、offboard 超时、安全半径）。

### H. 可视化与调试
- [ ] H.1 RViz：话题不变，数据源换成真机后直接可用；`config/rviz/wind_farm.rviz` 基本复用。
- [ ] H.2 新增：QGroundControl 地面站（手动/应急）、`ros2 bag` 录制巡检数据、飞行日志分析。
- [ ] H.3 仿真回归：每次规划器/坐标系改动先在 SITL 全流程验证（现有 bringup 就是回归工具）。

### I. 测试与验证流程（严格递增）
- [ ] I.1 **硬件在环联调**：真飞控 + 仿真验证固件/通信/控制闭环。注意 PX4 1.15 对 gz 的
      HITL 支持有限（官方 HITL 主要在 gazebo-classic/SIH），可先用 SIH 或直接走
      「真飞控 + 伴飞电脑串口链路 + 地面站」的静态联调，改 A/B/C 后必做。
- [ ] I.2 地面系留测试：无桨/限位，验证解锁流程、指令链路、传感器供电与数据。
- [ ] I.3 低空手动试飞：GPS/罗盘/姿态/震动验证。
- [ ] I.4 低速 offboard 悬停：位置/速度控制、failsafe 触发行为验证。
- [ ] I.5 小半径航线试飞（远离风机）→ 全尺寸巡检航线（逐级扩大，先单机后多机）。
- [ ] I.6 数据后处理：比对真机点云/图像与仿真，验证巡检数据质量。

### J. 合规与安全
- [ ] J.1 无人机实名登记、飞手执照、空域审批、视距/BVLOS 许可、保险。
- [ ] J.2 风电场：业主许可、风机停机/限产窗口、现场应急通信与疏散预案。
- [ ] J.3 限高与禁飞区检查（中国 120m 视距内；风机巡检常需超视距，逐项申请）。
- [ ] J.4 应急预案：GPS 丢失、强风、链路中断、风机异常气流下的处置流程文档化。

---

## 三、建议实施顺序（阶段路线图）

| 阶段 | 内容 | 依赖 | 出口条件 |
|------|------|------|----------|
| **P1 硬件+通信** | A + B + C，硬件在环联调 | 硬件到位 | 真机链路下解锁/offboard/悬停流程打通 |
| **P2 传感器接入** | E + F（TF 源替换），RViz 出真机数据 | P1 | 相机/点云/坐标在 RViz 中正确显示 |
| **P3 控制与安全** | D + G（速度控制、failsafe、真实坐标） | P2 | SITL 回归通过 + 地面测试通过 |
| **P4 试飞迭代** | I（系留→手动→offboard→航线）+ H | P3 | 完整巡检航线试飞成功 |
| **P5 现场部署** | J 合规 + 现场联调 + 数据后处理 | P4 | 风电场现场巡检任务闭环 |

---

## 四、风险与注意

- **风机尾流湍流**：风机下游气流紊乱（仿真只给了 3m/s 均匀风），巡检航线要避开
  下风向或保持安全距离；真实阵风下 offboard 控制器可能饱和。
- **GPS/RTK 精度**：仿真 GPS 理想；真机风机巡检对横向精度要求高，强烈建议 RTK；
  风机塔身本身可能造成多径/遮挡，GPS 失锁处理（进入 RTL/悬停）必须演练。
- **电磁环境**：风机及变流器电磁干扰可能影响罗盘/GPS；装机前做磁干扰测试。
- **传感器时间同步**：相机/lidar 与飞控时钟不同步会导致后续融合/建图错位，
  尽早搭建同步方案（P2 就做，别拖到感知阶段）。
- **续航与航线**：12 航点环绕一圈耗时长，电池容量与返航余量需核算（仿真无此约束）。
- **仿真与真机动力学差异**：SDF 中的质量/惯量是估的；速度控制 + 限幅是迁移的第一道保险。
- **px4_msgs 版本耦合**：真机固件必须与 px4_msgs v1.15.4 严格同版本，否则消息字段漂移
  （仿真已踩过 main 分支的坑）。
