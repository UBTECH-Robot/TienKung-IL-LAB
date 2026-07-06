# UBT Sim

UBTECH 机器人仿真平台，基于 NVIDIA Isaac Sim 5.0 + Isaac Lab 2.2。

支持双机器人（天工 Pro / Walker S2）、遥操作仿真、数据采集（HDF5/LeRobot 格式）、轨迹回放、零件随机化。

## ⚠️ Git LFS 注意事项

本项目使用 Git LFS 管理 3D 模型等大文件（`*.usd`、`*.urdf`、`*.exr`、`*.png`）。未安装 Git LFS 或下载不完整时，这些文件仅为指针文件，仿真无法启动。

```bash
git lfs install                          # 安装 LFS（仅需一次）
git clone <仓库地址>                      # 克隆（LFS 文件自动下载）
git lfs pull                             # 补全未下载的 LFS 文件
GIT_LFS_SKIP_SMUDGE=1 git clone <仓库地址>  # 仅克隆代码，跳过 LFS
```

## 架构

单容器架构，ROS2 Humble（Py 3.10）与 Isaac Sim（Py 3.11）共存：

```
┌─────────────────────────────────────────────────────────┐
│ Docker Container (host network)                         │
│                                                         │
│  Isaac Sim (Py 3.11)                                    │
│  └─ scripts/sim_runner.py (自动检测天工 Pro / Walker S2) │
│       │                                                 │
│       │  ZMQ PUB/SUB (127.0.0.1)                        │
│       │  ├─ cmd port (5555 / 5655)                      │
│       │  ├─ status port (5556 / 5656)                   │
│       │  ├─ image port (5557 / 5657)                    │
│       │  └─ jpeg port (N/A  / 5658)                     │
│       ▼                                                 │
│  ROS2 Bridge (Py 3.10, 子进程)                          │
│  ├─ bridges/tienkung/tiangong_ros2_zmq_bridge.py (天工) │
│  └─ bridges/walker/walker_s2_ros2_zmq_bridge.py         │
│       │                                                 │
│       │  ROS2 DDS (domain 146 仿真 / 0 真机)            │
│       ▼                                                 │
│  控制脚本 (Py 3.10)                                     │
│  ├─ teleoperation/control/tienkung/  (天工 Pro)         │
│  └─ teleoperation/control/walker/    (Walker S2)        │
└─────────────────────────────────────────────────────────┘
```

- `source/`（Isaac Sim Py 3.11）和 `teleoperation/`（ROS2 Py 3.10）运行在不同 Python 环境，**零交叉导入**，仅通过 ZMQ 通信
- Walker S2 桥接使用独立的 ZMQ 端口组（默认 5655-5658），天工 Pro 使用 5555-5557
- 天工 Pro 桥接集成 C++ 图像桥接（`zmq_image_bridge`），支持 ZMQ 原始图像 → ROS2 Image 消息的高效转换

## 支持的机器人

| 机器人 | DOF | 夹爪 | 手部 | USD 路径 |
|--------|-----|------|------|----------|
| 天工 Pro | 50 | — | 12×2 (五指灵巧手) | `assets/robots/tiangong_pro/tiangong_pro_v2.usd` |
| Walker S2 | 34 | 2×2 (PGC 两指) | — | `assets/robots/walker_s2/s2_v1.usd` |

## 支持的任务

| 任务 ID | 机器人 | 场景 | 说明 |
|---------|--------|------|------|
| `UBTSim-TiangongPro-Parlor-v0` | 天工 Pro | 客厅 | 基础遥操作，RGB + Depth 相机 |
| `UBTSim-WalkerS2-Parlor-v0` | Walker S2 | 客厅 | 基础遥操作，RGB 相机 |
| `UBTSim-WalkerS2-PartSorting-v0` | Walker S2 | 仓库 | 零件分拣，含零件随机化 |

任务通过 YAML 配置文件定义（`config/*.yaml`），Python 任务类从 YAML 自动加载场景、机器人、相机和仿真参数。

## 快速开始

```bash
# 1. 构建并启动容器
cd docker
bash run.sh build && bash run.sh start && bash run.sh init && bash run.sh check

# 2. 进入容器
bash run.sh bash

# 3. 启动仿真（根据 UBT_SIM_TASK 自动检测机器人并启动对应桥接）
# 天工 Pro（默认）：
bash scripts/start_sim.sh
# Walker S2：
UBT_SIM_TASK=UBTSim-WalkerS2-PartSorting-v0 bash scripts/start_sim.sh

# 跳过桥接（仅加载场景，无 ROS 控制）：
UBT_SIM_NO_BRIDGE=1 bash scripts/start_sim.sh

# 仅加载场景预览（不启动遥操作控制）：
UBT_SIM_LOAD_ONLY=1 bash scripts/start_sim.sh

# 4. 天工 Pro 数据采集（同一容器内，用系统 Python 3.10）
source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=0
/usr/bin/python3 /ubt_sim/teleoperation/control/tienkung/reset.py     # 机器人回零
/usr/bin/python3 /ubt_sim/teleoperation/control/tienkung/pick_place_save_data.py  # 单次采集
bash /ubt_sim/teleoperation/control/tienkung/save_data.sh              # 批量采集

# 5. Walker S2 数据采集
source /opt/ros/humble/setup.bash && source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
export ROS_DOMAIN_ID=0
/usr/bin/python3 /ubt_sim/teleoperation/control/walker/walker_s2_reset.py  # 机器人回零
/usr/bin/python3 /ubt_sim/teleoperation/control/walker/pick_part_save_data.py  # 单次采集
bash /ubt_sim/teleoperation/control/walker/save_data.sh                     # 批量采集
```

## 模式切换

通过 `ROS_DOMAIN_ID` 区分仿真与真机：

| 模式 | ROS_DOMAIN_ID | ZMQ 桥接目标 | 说明 |
|------|--------------|-------------|------|
| 仿真 | 146 | Isaac Sim（同容器） | 默认仿真模式 |
| 真机 | 0 | 真实机器人 SDK | 同容器，`ROS_DOMAIN_ID=0` |

真机部署：同一容器切换 `ROS_DOMAIN_ID=0` 即可。

注意：使用 `echo $ROS_DOMAIN_ID` 和 `ros2 topic list` 检查当前模式和桥接状态。

## 项目结构

```
ubt_sim/
├── config/                          # YAML 任务/场景配置
│   ├── tiangong_pro/parlor.yaml     # 天工 Pro 客厅场景
│   ├── walker_s2/parlor.yaml        # Walker S2 客厅场景
│   └── walker_s2/part_sorting.yaml  # Walker S2 零件分拣场景
├── source/ubt_sim/                  # Python pip 包（Py 3.11，Isaac Sim 侧）
│   ├── devices/                     # 机器人配置 + 遥操作设备
│   │   ├── device_base.py           # DeviceBase 抽象类
│   │   ├── action_process.py        # 动作预处理（天工 Pro）
│   │   ├── tiangong_pro/            # 天工 Pro：config + controller
│   │   └── walker_s2/               # Walker S2：config + controller + action_process
│   ├── env/                         # 数字孪生环境 + MDP
│   │   ├── digital_twin_env.py      # RGB overlay / 绿幕合成
│   │   └── digital_twin_env_cfg.py  # 环境配置基类
│   ├── task/                        # Gym 任务注册
│   │   ├── tiangong_parlor/         # 天工 Pro 客厅
│   │   ├── walker_s2_parlor/        # Walker S2 客厅
│   │   └── walker_s2_part_sorting/  # Walker S2 零件分拣
│   └── utils/                       # 工具函数（配置加载、循环工具、常量）
├── teleoperation/                   # 遥操作脚本（Py 3.10，ROS2 侧）
│   ├── bridges/                     # ROS2-ZMQ 桥接
│   │   ├── tienkung/tiangong_ros2_zmq_bridge.py  # 天工 Pro 桥接（Python）
│   │   ├── walker/walker_s2_ros2_zmq_bridge.py   # Walker S2 桥接（Python）
│   │   ├── zmq_image_bridge.cpp     # C++ 图像桥接（ZMQ → ROS2 Image）
│   │   └── bridge_config.yaml       # 天工 Pro 桥接 topic 映射
│   ├── control/                     # 机器人控制 + 数据采集
│   │   ├── constants.py             # 天工 Pro 关节 ID 常量
│   │   ├── tienkung/                # 天工 Pro：回零、抓放、数据采集
│   │   └── walker/                  # Walker S2：回零、IK、抓取、数据采集
│   └── tools/                       # 诊断工具（图像测试、数据分析、URDF 提取）
├── scripts/                         # 仿真启动脚本
│   ├── start_sim.sh                 # 统一启动脚本（自动检测机器人）
│   └── sim_runner.py                # 统一仿真主循环（天工 Pro + Walker S2）
├── assets/                          # 3D 模型（USD, URDF, 贴图）
│   ├── robots/{tiangong_pro,walker_s2}/
│   └── scenes/{parlor,part_sorting}/
├── docker/                          # Docker 容器配置
│   └── isaac_sim/                   # 仿真 + ROS2 统一容器
│       ├── Dockerfile               # 基于 NVIDIA Isaac Sim 基础镜像
│       ├── run.sh                   # 容器管理（build/start/init/check）
│       └── env.sh                   # 环境变量
├── shell/                           # Isaac Sim 运行时软链接
├── docs/                            # 文档
└── dataset/                         # 采集数据（运行时生成）
```

## 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `ROS_DOMAIN_ID` | 0 | 146=仿真，0=真机 |
| `UBT_SIM_NO_BRIDGE` | — | 设为 1 跳过 ZMQ 桥接启动 |
| `UBT_SIM_LOAD_ONLY` | — | 设为 1 仅加载场景预览，不启动遥操作 |
| `UBT_SIM_TASK` | `UBTSim-TiangongPro-Parlor-v0` | 天工 Pro 默认任务 |
| `UBT_SIM_NUM_ENVS` | 1 | 并行环境数量 |
| `UBT_SIM_WALKER_S2_DEVICE` | `cuda:0` | Walker S2 渲染设备 |
| `UBT_SIM_WALKER_S2_PHYSICS_DEVICE` | `cpu` | Walker S2 物理设备 |
| `UBT_SIM_WALKER_S2_HEAD_MATERIAL_MODE` | `all` | Walker S2 头部材质模式 |
| `UBT_SIM_ASSETS_ROOT` | 自动检测 | 覆盖 assets 目录路径 |

## Walker S2 特殊说明

### GPU 渲染 + CPU PhysX

Walker S2 默认采用 **GPU 渲染 + CPU PhysX** 架构：`start_sim.sh` / `sim_runner.py` 将渲染/AppLauncher 设备设为 `cuda:0`，但 Isaac Lab physics/env 设备设为 `cpu`。这是为了规避 Isaac Sim 5.0 + Isaac Lab 2.2 在 Walker S2 articulation 初始化时 `get_dof_velocities()` 触发的 PhysX GPU tensor device mismatch（`getVelocities: expected device 0, received device -1`）。不要默认改回 GPU physics；如需实验可显式设置 `UBT_SIM_WALKER_S2_PHYSICS_DEVICE=cuda:0`。

### 头部材质修复

`sim_runner.py` 在环境初始化后自动修复 Walker S2 头部材质绑定（通过 `source/ubt_sim/utils/head_material.py`），支持多种材质模式通过 `UBT_SIM_WALKER_S2_HEAD_MATERIAL_MODE` 控制：
- `all`（默认）：每个 GeomSubset 使用对应材质
- `stable`：全部使用稳定灰调材质
- 也可指定单一材质（如 `paint_matte`、`steel_blued`、`glass`）

### 零件随机化（Part Sorting 任务）

Walker S2 零件分拣任务支持通过 ZMQ 命令触发零件位姿随机化：

```json
{"randomize_part_sorting_pieces": {"parts": ["part_a_ori"], "range": {"x": [-0.05, 0.05], "yaw": [-0.785, 0.785]}}}
```

随机化参数在 `config/walker_s2/part_sorting.yaml` 的 `objects.part_randomization` 段配置。

## 关键约束

- **Python 不可混用**：Isaac Sim 用 3.11 (`/isaac-sim/python.sh`)，ROS2 用 3.10 (`/usr/bin/python3`)
- **numpy < 2**（cv_bridge 兼容性）
- **ROS_DOMAIN_ID**：仿真 146，真机 0
- **Docker 网络**：必须使用 `host` 模式（ZMQ + ROS2 DDS 需要）
- **Walker S2 桥接**：需要预先构建 SDK ROS2 消息包（`bash run.sh init` 自动完成）

## 扩展

### 添加新场景
1. 放置 3D 文件到 `assets/scenes/<name>/`
2. 创建 `config/<task>.yaml`（含 scene、robot、cameras、simulation 字段）
3. 创建 `source/ubt_sim/task/<task>/`（继承 `ManagerBasedRLDigitalTwinEnvCfg`）
4. 在 `source/ubt_sim/__init__.py` 注册任务

### 添加新机器人
1. 放置 3D 文件到 `assets/robots/<name>/`
2. 创建 `source/ubt_sim/devices/<name>/`（config.py + controller.py + action_process.py）
3. 在 `source/ubt_sim/devices/__init__.py` 导出
4. 创建桥接脚本 `teleoperation/bridges/<name>_ros2_zmq_bridge.py`
5. 创建控制脚本 `teleoperation/control/<name>/`
6. 创建任务配置和启动脚本

### 添加新控制任务
- 继承 `DeviceBase`（[source/ubt_sim/devices/device_base.py](source/ubt_sim/devices/device_base.py)），实现 `reset()` / `advance()` / `add_callback()`
- 在对应任务 EnvCfg 中实现 `preprocess_device_action()` 方法完成动作映射

## Docker 管理命令

```bash
cd docker
bash run.sh build          # 构建镜像
bash run.sh start          # 创建/启动容器
bash run.sh stop           # 停止容器
bash run.sh restart        # 重启容器
bash run.sh bash           # 进入容器 shell
bash run.sh init           # 安装所有依赖（ubt_sim 包、ROS2 消息、C++ 图像桥接）
bash run.sh check          # 环境检查
bash run.sh rm             # 删除容器
bash run.sh bridge-start   # 手动启动 ROS2-ZMQ 桥接
bash run.sh bridge-stop    # 手动停止 ROS2-ZMQ 桥接
```

## License

Apache-2.0
