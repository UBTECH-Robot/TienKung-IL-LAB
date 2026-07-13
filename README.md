# TienKung-IL-LAB

优必选天工机器人模仿学习工具链

## 项目简介

本项目基于 [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim) 与 [LeRobot](https://github.com/huggingface/lerobot) 框架开发，为天工（TienKung）及 Walker S2 机器人提供完整的模仿学习工具链，涵盖以下核心能力：

| 能力              | 说明                                          | 状态      |
| ----------------- | --------------------------------------------- | --------- |
| 🌐 ROS 仿真环境   | 高逼真度 Isaac Sim 仿真，支持遥操作与数据采集 | ✅ 已完成 |
| 🎮 遥控操作       | 键盘/空间鼠标等设备遥操作，仿真与真机统一接口 | 🚧 开发中 |
| 📦 数据采集与转换 | HDF5 / LeRobot 格式数据采集，格式转换与清洗   | ✅ 已完成 |
| 🧠 模型训练       | 基于 LeRobot 的模仿学习策略训练               | ✅ 已完成 |
| 🤖 真机部署       | 模型推理与真机控制部署                        | ✅ 已完成 |
| 🤖 其台机型       | Walker S2 / Walker C1 / TienKung 3.0          | 🚧 待发布 |

## 代码获取

克隆仓库后，需先拉取 LFS 大文件（USD 模型、贴图等）并初始化子模块（`lerobot`）。

```bash
# （可选）配置代理：访问 GitHub 较慢时设置，端口按本地代理调整
git config --global http.proxy  http://127.0.0.1:7897
git config --global https.proxy http://127.0.0.1:7897
export GIT_LFS_PROXY="http://127.0.0.1:7897"   # LFS 走代理

# 1. 克隆仓库（先跳过 LFS 和子模块）
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/UBTECH-Robot/TienKung-IL-LAB.git

# 2. 拉取 LFS 大文件
git lfs pull

# 3. 初始化并拉取子模块（lerobot）
git submodule update --init
```

## 仿真模块

### 快速开始

```bash
# 1. 构建并启动容器
cd ubt_sim/docker/isaac_sim
bash run.sh build && bash run.sh start && bash run.sh init && bash run.sh check
# 若需要区分真机/仿真模式启动容器，使用参数启动（默认0）：ROS_DOMAIN_ID=0 bash run.sh start

# 2. 启动仿真（自动启动 ROS2-ZMQ 桥接）
bash run.sh bash
bash /ubt_sim/scripts/start_sim.sh
# 按 R 机器人可复位

# 3. 数据采集（同一容器内，用系统 Python 3.10）
/usr/bin/python3 /ubt_sim/teleoperation/control/reset.py  # 机器人回零
/usr/bin/python3 /ubt_sim/teleoperation/control/pick_place_save_data.py  # 单次
bash /ubt_sim/teleoperation/control/save_data.sh                         # 批量

# （其他）测试相机
python3 /ubt_sim/teleoperation/image_client.py
```

注意：使用 `echo $ROS_DOMAIN_ID` 和 `ros2 topic list` 检查当前模式仿真/真机，以及桥接是否启动。
仿真模块具体介绍与详细使用说明请参考 [ubt_sim/README.md](ubt_sim/README.md)。

### 模式说明

可通过 `ROS_DOMAIN_ID` 区分仿真与真机，以隔离 ROS 指令：

| 模式 | ROS_DOMAIN_ID | 说明                   |
| ---- | ------------- | ---------------------- |
| 仿真 | 146           | ZMQ 桥接连接 Isaac Sim |
| 真机 | 0（默认）     | ZMQ 桥接连接真实机器人 |

## 真机数采

使用 Thinker Studio 遥操数采平台进行数据采集，官方提供 [Thinker Studio](https://thinkercosmos.ubtrobot.com/#/studio) 遥操数采平台，可进行数据采集。具体参见官网使用文档，直接导出 LeRobot v3.0 数据集。

```bash
# 合并采集到的数据集
INPUT_DATASETS="Pick_up_the_red_bottle_1 Pick_up_the_red_bottle_2 Pick_up_the_red_bottle_3 Pick_up_the_red_bottle_4" \
  OUTPUT_DATASET=Pick_up_the_red_bottle \
  bash /ubt_IL/scripts/convert/common/merge_datasets.sh
```

## 训练模块

基于 LeRobot ACT 策略，数据集来源于仿真采集的 HDF5 或真机遥操作数据，训练在 `lerobot-tienkung` 容器内完成。`bash run.sh check` 用于环境健康检查。

### 快速开始

```bash
# 1. 构建并启动训练容器（自动启动 Bridge2）
cd ubt_IL/docker
bash run.sh build && bash run.sh start && bash run.sh check

# 2. 数据转换：HDF5 -> LeRobot 格式（默认仿真配置）
bash /ubt_IL/scripts/convert/tienkung_pro/convert.sh
# 自定义数据集路径：SRC_ROOT="你的数据集目录" REPO_ID="任务ID" CONFIG="自定义数据转换配置文件"
# 举例：转换 13-DOF 数据集（右臂7+右手6）
SRC_ROOT=ubt_IL/dataset/sim_pick_place_hdf5 REPO_ID=sim_pick_place_right13 \
  CONFIG=/ubt_IL/scripts/convert/tienkung_pro/configs/tienkung_pro_13d_1RGB.json \
  bash /ubt_IL/scripts/convert/tienkung_pro/convert.sh

# 3. 训练（默认使用仿真 ACT 配置）
bash /ubt_IL/scripts/train/tienkung_pro/train.sh
# 使用真机数据训练
CONFIG_PATH=/ubt_IL/scripts/train/tienkung_pro/train_config_real_act.json \
  DATASET_ROOT=/ubt_IL/dataset/Pick_up_real_data \
  DATASET_REPO_ID=Pick_up_real_data \
  OUTPUT_DIR=/ubt_IL/model/Pick_up_real_act \
  bash /ubt_IL/scripts/train/tienkung_pro/train.sh
```

`train.sh` 通过 `--config_path` 加载 `train_config_sim_act.json` 作为完整配置，环境变量覆盖的字段优先级高于配置文件。

### 关键参数

| 环境变量            | 默认值                                                       | 说明                             |
| ------------------- | ------------------------------------------------------------ | -------------------------------- |
| `CONFIG_PATH`       | `/ubt_IL/scripts/train/tienkung_pro/train_config_sim_act.json` | 训练配置文件路径                 |
| `DATASET_ROOT`      | `/ubt_IL/dataset/real_merged`                                | 数据集根目录                     |
| `DATASET_REPO_ID`   | `real_pick_place`                                            | 数据集 repo id                   |
| `OUTPUT_DIR`        | `/ubt_IL/model/real_pick_place_act`                          | 模型与检查点输出目录             |
| `STEPS`             | `50000`                                                      | 训练步数                         |
| `BATCH_SIZE`        | `8`                                                          | 批大小                           |
| `SEED`              | `10000`                                                      | 随机种子                         |
| `DEVICE`            | `cuda`                                                       | 训练设备                         |
| `HF_HUB_OFFLINE`    | `1`                                                          | 离线模式，不访问 HuggingFace Hub |
| `RESUME`            | `false`                                                      | 断点续训开关                     |

详细参数、ACT 配置、数据可视化命令见 [ubt_IL/README.md](ubt_IL/README.md#3-模型训练)。

## 仿真部署

天工仿真部署使用上述 ubt_sim 模块代替机器人真机进行测试。该仿真环境与真机 ROS 话题部署和通信方法一致，可用于真机部署前的验证工作，避免真机动作错误造成损坏等严重后果。仿真模块容器独立运行，与模型训练推理容器在同一主机通过本地回环 `127.0.0.1` 网段进行 ROS 通信。

### 快速开始

```bash
# 1. 启动仿真（已启动可跳过）
cd ubt_sim/docker/isaac_sim
bash run.sh bash
bash /ubt_sim/scripts/start_sim.sh
# 按 R 机器人可复位

# 2. 初始化动作（抬起手臂到桌面上）
/usr/bin/python3 /ubt_sim/teleoperation/control/reset.py

# 3. 启动推理容器，运行推理脚本
cd ubt_IL/docker
bash run.sh bash

# 部署 26-DOF 模型（默认，全 26 自由度）
POLICY_PATH=/ubt_IL/model/sim_pick_place_act/checkpoints/last/pretrained_model \
  bash /ubt_IL/scripts/deploy/tienkung_pro/rollout.sh

# 部署 13-DOF 模型（右臂7+右手6，JOINT_CONFIG 须与训练 DOF 一致）
POLICY_PATH=/ubt_IL/model/sim_pick_place_right13_act/checkpoints/last/pretrained_model \
  JOINT_CONFIG=tienkung_13 \
  bash /ubt_IL/scripts/deploy/tienkung_pro/rollout.sh

# （可选操作）仿真中回放数据集动作
/usr/bin/python3 /ubt_IL/scripts/deploy/tienkung_pro/replay.py \
  --dataset /ubt_IL/dataset/sim_pick_place --episode 0 --rate 30
```

### 关键参数

| 变量            | 默认值                                                         | 说明                                                                                      |
| --------------- | -------------------------------------------------------------- | ----------------------------------------------------------------------------------------- |
| `POLICY_PATH`   | `.../real_pick_place_act/.../pretrained_model`                 | 模型 checkpoint                                                                           |
| `JOINT_CONFIG`  | `tienkung_26`                                                  | 关节 DOF 配置（`tienkung_26`=全26；`tienkung_13`=右臂7+右手6），须与数据/模型训练 DOF 一致 |
| `STRATEGY`      | `base`                                                         | 推理策略（`base` 自主执行不录制；`sentry`/`highlight`/`dagger` 用于录制或交互）            |
| `TASK`          | `sim_pick_place`                                               | 任务描述（注入 policy 的任务条件）                                                        |
| `ZMQ_HOST`      | `127.0.0.1`                                                    | ImageServer 相机地址（仿真使用本地回环）                                                  |
| `DURATION`      | `60`                                                           | 运行时长（秒）                                                                            |
| `FPS`           | `30`                                                           | 控制环频率（与训练 fps 对齐）                                                             |

> `JOINT_CONFIG` 决定 policy 的关节维度与顺序，须与模型训练时的数据集顺序一致；非激活关节（如 13-DOF 的左侧）自动用 home 位姿/张开手填充。
> 13-DOF 数据集转换、训练、续训与部署的完整流程见 [ubt_IL/README.md](ubt_IL/README.md#4-模型部署)。

## 真机部署

天工真机部署需确认 `ROS_DOMAIN_ID=0`，部署机与机器人需同网段（如 `192.168.41.x`）。架构为容器内 LeRobot 通过 ZMQ 与 Bridge2 通信，相机由机器人端 ImageServer 提供 JPEG 流。

### 快速开始

```bash
# 0. 网络配置：编辑 ubt_IL/docker/fastdds_no_shm.xml 第二个 <address>，
#    改为本机 IP（如 192.168.41.99），保证与机器人在同一网段，随后重启容器
cd ubt_IL/docker
bash run.sh restart

# 1. 机器人端启动相机服务（仅真机部署需要）
scp ubt_IL/scripts/deploy/tienkung_pro/image_server.py nvidia@192.168.41.2:~
ssh nvidia@192.168.41.2 'python3 image_server.py'
python3 /ubt_IL/scripts/deploy/tienkung_pro/image_client.py  # 测试相机通路

# 2. 容器内复位 + 推理
bash run.sh bash
/usr/bin/python3 /ubt_IL/scripts/deploy/tienkung_pro/reset.py     # 机器人回零
POLICY_PATH=/ubt_IL/model/test_model ZMQ_HOST=192.168.41.2 DURATION=60 \
  bash /ubt_IL/scripts/deploy/tienkung_pro/rollout.sh

# 3. （可选）数据集回放校验链路
/usr/bin/python3 /ubt_IL/scripts/deploy/tienkung_pro/replay.py \
  --dataset /ubt_IL/dataset/real_grasp_bottle --episode 0 --rate 30
```

### 关键参数

| 变量            | 默认值                                                         | 说明                                                                                      |
| --------------- | -------------------------------------------------------------- | ----------------------------------------------------------------------------------------- |
| `POLICY_PATH`   | `.../real_pick_place_act/.../pretrained_model`                 | 模型 checkpoint                                                                           |
| `JOINT_CONFIG`  | `tienkung_26`                                                  | 关节 DOF 配置（`tienkung_26`=默认全26；`tienkung_13`=右臂7+右手6），支持自定义关节配置   |
| `STRATEGY`      | `base`                                                         | 推理策略（`base` 自主执行不录制；`sentry`/`highlight`/`dagger` 用于录制或交互）            |
| `TASK`          | `pick and place`                                               | 任务描述（注入 policy 的任务条件）                                                        |
| `ZMQ_HOST`      | `192.168.41.2`                                                 | ImageServer 地址（真机改机器人 IP）                                                       |
| `DURATION`      | `60`                                                           | 运行时长（秒）                                                                            |
| `FPS`           | `30`                                                           | 控制环频率（与训练 fps 对齐）                                                             |

注意：`ubt_IL/docker/fastdds_no_shm.xml` 中的 IP 必须改为本机 IP，否则 ROS 无法与真机通信。详细架构图、26 维向量布局见 [ubt_IL/CLAUDE.md](ubt_IL/CLAUDE.md)；完整部署参数与 `lerobot-rollout` CLI 调用见 [ubt_IL/README.md](ubt_IL/README.md#4-模型部署)。

## 真机本体 ARM 板部署

天工真机本体部署教程，使用机器人 Orin 板做推理部署，由于 docker 容器使用 x86 构建系统架构不同，使用 conda 在 Orin 板构建虚拟部署环境。详细操作流程见 [ARM 板部署详细文档](ubt_IL/scripts/deploy/tienkung_pro/arm_64/README.md)。

### 快速开始

```bash
# 0. 环境初始化
mkdir /home/nvidia/vla/   # 将项目代码复制到此处
cd /home/nvidia/vla/TienKung-IL-LAB/ubt_IL/scripts/deploy/tienkung_pro/arm_64
bash setup_env.sh  # 构建 conda 环境 env_vla

# 1. 机器人端启动相机服务（仅真机部署需要）
conda activate env_vla
bash /home/nvidia/vla/TienKung-IL-LAB/ubt_IL/scripts/deploy/tienkung_pro/arm_64/image_server_host.sh

# 若机器人端未安装 pyorbbec 相机驱动请安装相关依赖包（仅安装一次）
python3 -m pip install evdev
python3 -m pip install pyorbbecsdk2

# （可选）机器人相机预览测试
bash /home/nvidia/vla/TienKung-IL-LAB/ubt_IL/scripts/deploy/tienkung_pro/arm_64/image_client_host.sh --show

# 2. 机器人预备动作（抬起右手）
bash /home/nvidia/vla/TienKung-IL-LAB/ubt_IL/scripts/deploy/tienkung_pro/arm_64/robot_ready.sh

# 3. 运行推理脚本
conda activate env_vla
# 部署 26-DOF 模型（默认）
POLICY_PATH=/home/nvidia/vla/TienKung-IL-LAB/ubt_IL/model/Pick_up_tiangong_all_act/checkpoints/last/pretrained_model \
  DURATION=60 \
  bash /home/nvidia/vla/TienKung-IL-LAB/ubt_IL/scripts/deploy/tienkung_pro/arm_64/rollout_host.sh

# 4. （可选）数据集回放，在真机上播放采集的动作
/usr/bin/python3 /home/nvidia/vla/TienKung-IL-LAB/ubt_IL/scripts/deploy/tienkung_pro/replay.py \
  --dataset /home/nvidia/vla/TienKung-IL-LAB/ubt_IL/dataset/Pick_up_the_apple_all \
  --episode 0 --rate 30
```

### 关键参数

| 变量            | 默认值                                                                                                 | 说明                                                                     |
| --------------- | ------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------ |
| `POLICY_PATH`   | `$PROJECT_ROOT/model/Pick_up_tiangong_all_act/checkpoints/last/pretrained_model`                       | ACT checkpoint                                                           |
| `JOINT_CONFIG`  | `tienkung_26`                                                                                          | 关节 DOF 配置（`tienkung_26`=全26；`tienkung_13`=右臂7+右手6），须与训练时 DOF 一致 |
| `STRATEGY`      | `base`                                                                                                 | 推理策略（`base` 自主执行；`sentry`/`highlight`/`dagger` 用于录制或交互） |
| `TASK`          | `sim_pick_place`                                                                                       | 任务描述（注入 policy 的任务条件）                                       |
| `ZMQ_HOST`      | `127.0.0.1`                                                                                            | image_server 地址（真机相机在机器人端则改其 IP）                          |
| `DURATION`      | `60`                                                                                                   | 运行时长（秒）                                                           |
| `FPS`           | `30`                                                                                                   | 控制环频率（与训练 fps 对齐）                                            |
| `DISPLAY_CAM`   | `true`                                                                                                 | 相机显示（SSH 无 X 设 `false`）                                          |

**关键约束**

- **Python 3.12**：LeRobot 0.5.2 + tienkung 插件原生运行，免源码补丁；勿降级 3.10。
- **本地 wheel**：torch/torchvision 用本目录的 cp312 Jetson 专属 wheel（链接本机 glibc 2.35）；勿用 PyPI 的 aarch64 wheel（CPU-only 无 CUDA），亦勿用需 GLIBC_2.38 的预编译 wheel（本机 2.35 会崩）。
- **numpy 必须 <2**：本地 torch 按 numpy 1.x 编译，`setup_env.sh` 已强制 `numpy==1.26.4`，勿手动升级。
- **双栈别混**：`image_server_host.sh`、`robot_ready.sh` 走系统 python3.10（无需 activate）；`rollout_host.sh`、`train_host.sh`、`image_client_host.sh` 走 env_vla (3.12)，需先 `conda activate env_vla`。
- **真机网络**：需把 `ubt_IL/docker/fastdds_no_shm.xml` 中的 IP 改为本机 IP，否则 ROS2 无法与真机通信。

完整参数表、双栈架构图与注意事项见 [ARM 板部署详细文档](ubt_IL/scripts/deploy/tienkung_pro/arm_64/README.md)。

## 致谢

本项目站在以下开源项目的肩膀上，谨致谢忱：

- [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim) - 高保真机器人仿真环境
- [HuggingFace LeRobot](https://github.com/huggingface/lerobot) - 模仿学习框架（ACT 策略、数据格式与训练/推理工具链）
- [Thinker Studio](https://thinkercosmos.ubtrobot.com/#/studio) - 优必选遥操数采平台

感谢以上社区与所有贡献者的卓越工作。如本项目对您有帮助，欢迎 Star ⭐ 支持。
