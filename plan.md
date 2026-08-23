# 动态风机世界加载改造计划

## 现状分析

当前 `wind_farm_bringup.sh` 的加载流程：
1. PX4 SITL 启动 `wind_farm` 世界（`gz/worlds/wind_farm.sdf`）
2. SDF 世界中 **硬编码** 了 3 台静态装饰风机（`wind_turbine_2_1`, `1_2`, `2_2`），使用单 link 静态模型（`gz/models/wind_turbine/model.sdf`），无关节、无动画
3. 仅目标风机 `wind_turbine_1_1` 通过 xacro → URDF → `ros_gz_sim create` 在 PX4 启动 **之后** 动态 spawn，有叶片旋转动画
4. 装饰风机不支持任何参数配置（velocity、nacelle_yaw、blade_pitch、hub_position）

## 目标

- **所有风机** 在 PX4 SITL 启动时即作为世界的一部分加载（不再是启动后 spawn）
- **所有风机** 都是动态模型（有 joints + JointController），支持叶片旋转动画
- **所有风机** 支持像 `farm_launch.py` 一样指定参数：`velocity`、`nacelle_yaw`、`blade_pitch`、`hub_position`
- 配置来源：`config/wind_farm.yaml`，支持全局默认值 + 每台风机单独覆盖

## 修改计划

### 1. 扩展 `config/wind_farm.yaml` — 支持每台风机独立参数

```yaml
# 全局默认值（所有风机继承）
turbine_defaults:
  velocity: -2.0
  nacelle_yaw: 0.0
  blade_pitch: 0.0
  hub_position: 0.0

turbines:
  - name: wind_turbine_1_1
    x: 0.0
    y: 0.0
    # 未指定的参数从 turbine_defaults 继承
  - name: wind_turbine_2_1
    x: 200.0
    y: 0.0
    velocity: -1.5        # 单独覆盖转速
  - name: wind_turbine_1_2
    x: 100.0
    y: 200.0
  - name: wind_turbine_2_2
    x: 300.0
    y: 200.0
    nacelle_yaw: 0.3      # 单独覆盖机舱朝向
```

- 删除旧的顶层 `turbine_velocity` 字段，改用 `turbine_defaults.velocity`
- 保持向后兼容：若未提供 `turbine_defaults`，使用硬编码默认值

### 2. 新建 `scripts/generate_world.py` — 动态生成世界 SDF

功能：
- 读取 `config/wind_farm.yaml`，解析每台风机参数（合并 defaults + 个体覆盖）
- 读取模板 `gz/worlds/wind_farm.sdf.template`（或内嵌基础世界内容）
- 为每台风机生成带 joints + JointController 的 SDF `<model>` 块，内联到世界中
- 输出完整 SDF 到指定路径（如 `/tmp/wind_farm_generated.sdf`）

风机 SDF 模型结构（从 `turbine.xacro` 逻辑移植）：
- links: `tower`, `nacelle`, `hub`, `blade_1`, `blade_2`, `blade_3`
- joints:
  - `{name}_yaw` (revolute): tower → nacelle, limit = nacelle_yaw
  - `{name}_rotor` (continuous 或 revolute): nacelle → hub, velocity != 0 时用 continuous + JointController
  - `{name}_pitch_1/2/3` (revolute): hub → blade_i, limit = blade_pitch
- `<plugin>` JointController: 当 velocity != 0 时添加，设定 `<initial_velocity>`
- mesh 路径使用 `model://wind_turbine_dynamic/meshes/...` 或绝对路径

### 3. 修改 `gz/worlds/wind_farm.sdf` — 移除静态风机 `<include>`

- 删除 3 个 `<include><uri>model://wind_turbine</uri>...</include>` 块
- 保留 ground_plane、light、wind、spherical_coordinates 等基础世界元素
- 此文件变为纯模板/基础世界，不再包含任何风机

### 4. 修改 `scripts/wind_farm_bringup.sh` — 集成动态世界生成

改动点：
- **新增步骤 [2.5/7]**：在 PX4 SITL 启动前，调用 `generate_world.py` 生成完整 SDF
  ```bash
  python3 "$FLOATGEN_SRC/scripts/generate_world.py" \
      --config "$CONFIG" \
      --output /tmp/wind_farm_dynamic.sdf
  ```
- **修改步骤 [2/7]**：PX4 SITL 使用生成的 SDF 而非静态文件
  ```bash
  PX4_GZ_WORLD=/tmp/wind_farm_dynamic.sdf PX4_GZ_MODEL_POSE="$MODEL_POSE" make px4_sitl gz_x500_mid360 &
  ```
  或者将生成文件复制到 PX4 的 worlds 目录 / 通过 `GZ_SIM_RESOURCE_PATH` 指定路径
- **删除步骤 [3/7]**：不再需要单独 spawn 目标风机（所有风机已在世界中）
- **调整步骤编号**：从 7 步变为 6 步
- **清理 cleanup()**：移除 `TURBINE_PID` 相关逻辑

### 5. 新建动态风机 mesh 资源 或 复用现有 mesh

方案选择：
- **方案 A（推荐）**：在 `gz/models/` 下新建 `wind_turbine_dynamic/` 模型目录，包含 `model.sdf`（带 joints 的完整模型模板）和 `meshes/` 软链接到现有 mesh 文件。生成的世界 SDF 通过 `<include><uri>model://wind_turbine_dynamic</uri></include>` 引用，并通过 `<plugin>` 覆盖参数
- **方案 B**：在 `generate_world.py` 中直接内联所有 mesh 路径和 joint 定义到世界 SDF，不依赖独立模型目录

推荐方案 B（内联），因为：
- 每台风机参数不同，独立模型目录需要为每台风机的参数组合创建变体
- 内联更简单直接，所有逻辑集中在 `generate_world.py`

### 6. 处理 PX4 世界发现路径

PX4 SITL 通过 `PX4_GZ_WORLD` 环境变量查找世界文件。需要确保生成的 SDF 可被发现：
- 设置 `GZ_SIM_RESOURCE_PATH` 包含生成文件所在目录
- 或将生成文件写入 `$PX4_DIR/build/gz_x500_mid360/worlds/` 下
- 或直接使用绝对路径（需验证 PX4 make target 是否支持）

## 文件变更清单

| 文件 | 操作 | 说明 |
|------|------|------|
| `config/wind_farm.yaml` | 修改 | 添加 `turbine_defaults`，每台风机可独立指定参数 |
| `scripts/generate_world.py` | 新建 | 读取 YAML 配置，生成包含所有动态风机的完整 SDF |
| `gz/worlds/wind_farm.sdf` | 修改 | 移除 3 个静态风机 `<include>` 块 |
| `scripts/wind_farm_bringup.sh` | 修改 | 集成 `generate_world.py`，删除单独 spawn 步骤，调整步骤编号 |
| `gz/models/wind_turbine/model.sdf` | 保留或删除 | 若不再需要静态模型可删除 |

## 关键设计决策

1. **SDF 内联 vs 模型引用**：选择内联，每台风机作为独立 `<model>` 块直接写入世界 SDF
2. **mesh 路径**：使用 `model://wind_turbine/meshes/...`（复用现有模型目录的 mesh），或改用绝对路径 `$FLOATGEN_SRC/meshes/...`
3. **JointController 参数**：velocity=0 时使用 revolute joint + limit 锁定在 hub_position；velocity!=0 时使用 continuous joint + JointController
4. **世界文件传递**：生成到 `/tmp/` 并通过 `GZ_SIM_RESOURCE_PATH` 让 PX4 找到
