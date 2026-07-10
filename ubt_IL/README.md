# TienKung Robot — LeRobot 部署指南

天工双臂+灵巧手机器人 LeRobot 部署插件。支持 `lerobot-rollout` CLI 直接部署真机。

> 技术架构、端口定义、ROS2 话题、26 维向量格式等参见 [CLAUDE.md](./CLAUDE.md)。

## 目录

1. [快速开始](#快速开始)
2. [数据转换](#数据转换)
3. [模型训练](#模型训练)
4. [模型部署](#模型部署)

## 1. 快速开始

构建镜像并启动容器：

```bash
# 构建镜像 + 启动容器（自动启动 Bridge2）
cd docker
bash run.sh build && bash run.sh start  && bash run.sh check
bash run.sh bash           # 进入容器
```
容器启动后 `entrypoint.sh` 自动安装 lerobot 和天工插件（editable 模式），约 30 秒。查看日志确认完成：
```bash
sudo docker logs -f lerobot-tienkung  # 看到 "Installing lerobot-robot-tienkung plugin..." 即完成
# 其他容器操作
bash run.sh stop           # 停止容器
bash run.sh restart        # 重启容器
bash run.sh rm             # 删除容器
```
---

### 环境变量

环境变量集中定义在 `docker/env.sh`，可通过宿主机环境变量覆盖：

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `DOMAIN_ID` | `0` | ROS_DOMAIN_ID，真机默认 0 |

### 容器dds配置

真机部署请配置`ubt_IL/docker/fastdds_no_shm.xml`中的IP地址，该配置影响容器与真机的ROS通信，若无法通过ROS控制真机请修改后重启容器：

```bash
<address>127.0.0.1</address>      # 同机部署 IP走本地回环 
<address>192.168.41.99</address>  # 远程真机走网线直连，需设置IP为本机IP
```
---

## 2. 数据转换

将 HDF5 采集数据转换为 LeRobot 训练格式。

### 用法

```bash
# 默认参数（仿真数据，Tien_Kung_26_1RGB 配置）
bash scripts/convert/convert.sh

# 指定数据/输出目录
SRC_ROOT=path/to/hdf5_episodes TGT_PATH=path/to/output REPO_ID=my_dataset \
  bash scripts/convert/convert.sh

# 转换 13-DOF 数据集（右臂7+右手6）
REPO_ID=sim_pick_place_right13 TGT_PATH=/ubt_IL/dataset \
CONFIG=/ubt_IL/scripts/convert/configs/Tien_Kung_13_1RGB_sim.json \
  bash scripts/convert/convert.sh

# 或直接调用 Python 脚本获得更细粒度控制
python scripts/convert/convert_to_lerobot.py \
  --config scripts/convert/configs/Tien_Kung_26_1RGB_sim.json \
  --src_root path/to/hdf5_episodes --tgt_path path/to/output \
  --repo_id my_dataset --fps 30 --robot_type tienkung --task_name pick_and_place
```

主要参数（完整列表见 `convert_to_lerobot.py --help`）：

| 参数 | 环境变量 | 默认值 | 说明 |
|------|----------|--------|------|
| `--config` | `CONFIG` | `Tien_Kung_26_1RGB_sim.json` | 特征映射配置文件 |
| `--src_root` | `SRC_ROOT` | `/ubt_IL/dataset/sim_pick_place_hdf5` | HDF5 源数据目录 |
| `--tgt_path` | `TGT_PATH` | `/ubt_IL/dataset` | 输出父目录 |
| `--repo_id` | `REPO_ID` | `sim_pick_place` | 数据集名称 |
| `--fps` | `FPS` | `30` | 采样帧率 |

### 配置文件

配置文件位于 `scripts/convert/configs/`，定义 HDF5 → LeRobot 特征映射：

| 配置文件 | 场景 | 维度 | 说明 |
|----------|------|------|------|
| `Tien_Kung_26_1RGB_sim.json` | 仿真 | 26 | action 从 `action/` 读取 |
| `Tien_Kung_13_1RGB_sim.json` | 仿真 | 13 | 右臂7+右手6，action 从 `action/` 读取（仅右侧） |
| `Tien_Kung_26_1RGB_real.json` | 真机 | 26 | action 从 `master/` 读取，含灵巧手 invert/padding |
| `Tien_Kung_Gello_1RGB.json` | Gello | 16 | 关节空间，单相机 |

### 数据可视化

使用 `lerobot-dataset-viz` 在容器内可视化已转换的 LeRobot 数据集：

```bash
bash run.sh bash
# 在容器内：
HF_HUB_OFFLINE=1 lerobot-dataset-viz \
  --repo-id <数据集名称> \
  --episode-index 0 \
  --root /ubt_IL/dataset/<数据集名称>
```

示例：

```bash
# 可视化测试数据集
HF_HUB_OFFLINE=1 lerobot-dataset-viz \
  --repo-id test_pick_place \
  --episode-index 0 \
  --root /ubt_IL/dataset/test_pick_place
```
> **注意**：`--root` 须指向包含 `meta/` 目录的数据集路径（即 `repo_id` 目录本身），而非父目录。`HF_HUB_OFFLINE=1` 用于禁止访问 HuggingFace Hub。


### 真机回放数据集 action（replay.py）

将数据集的 action 原样发回真机/仿真。容器内运行，先跑 `reset.py` 复位。

```bash
# --rate 需匹配数据集 fps（真机/仿真均为 30）
/usr/bin/python3 /ubt_IL/scripts/deploy/replay.py \
  --dataset /ubt_IL/dataset/real_grasp_bottle --episode 0 --rate 30
```

常用参数：`--episode N`、`--rate Hz`、`--start/--end` 限帧、`--dry-run` 只打印不发。首次试跑可加 `--rate 10 --end 100` 低速验证前 100 帧。

依赖：`/usr/bin/python3 -m pip install --user pyarrow pandas`。


---

## 3. 模型训练

在 `lerobot-tienkung` 容器内运行。`train.sh` 通过 `--config_path` 加载训练配置（默认 `train_config_sim_act.json`），环境变量覆盖的字段优先级高于配置文件。

```bash
# 默认配置（仿真 ACT）
bash /ubt_IL/scripts/deploy/train.sh

# 使用真机数据训练
CONFIG_PATH=/ubt_IL/scripts/deploy/train_config_real_act.json \
DATASET_ROOT=/ubt_IL/dataset/Pick_up_real_data \
DATASET_REPO_ID=Pick_up_real_data \
OUTPUT_DIR=/ubt_IL/model/Pick_up_real_act \
  bash /ubt_IL/scripts/deploy/train.sh

# 训练 13-DOF 模型（右臂7+右手6，须先转换 13-DOF 数据集）
CONFIG_PATH=/ubt_IL/scripts/deploy/train_config_sim_act_right13.json \
DATASET_REPO_ID=sim_pick_place_right13 \
DATASET_ROOT=/ubt_IL/dataset/sim_pick_place_right13 \
OUTPUT_DIR=/ubt_IL/model/sim_pick_place_right13_act \
STEPS=50000 \
  bash /ubt_IL/scripts/deploy/train.sh

# 自定义参数
STEPS=100000 BATCH_SIZE=8 SEED=10000 \
  bash /ubt_IL/scripts/deploy/train.sh
```

### 续训（从 checkpoint 恢复）

`RESUME=true` 时 `train.sh` 从 `CONFIG_PATH` 指向的 checkpoint 内 `train_config.json` 定位 `training_state`，加载已训步数后继续训练到 `STEPS`。**`STEPS` 为总目标步数（非增量），须大于当前 checkpoint 步数才会继续训**。

```bash
# 续训 13-DOF 模型（须显式设 dataset/output_dir 为 13-DOF 值，否则用默认 26-DOF 配置导致 shape 不匹配）
CONFIG_PATH=/ubt_IL/model/sim_pick_place_right13_act/checkpoints/last/pretrained_model/train_config.json \
RESUME=true \
DATASET_REPO_ID=sim_pick_place_right13 \
DATASET_ROOT=/ubt_IL/dataset/sim_pick_place_right13 \
OUTPUT_DIR=/ubt_IL/model/sim_pick_place_right13_act \
STEPS=100000 \
  bash /ubt_IL/scripts/deploy/train.sh

# 续训 26-DOF 模型（默认值已匹配，仅需 CONFIG_PATH/RESUME/STEPS）
CONFIG_PATH=/ubt_IL/model/sim_pick_place_act/checkpoints/last/pretrained_model/train_config.json \
RESUME=true STEPS=300000 \
  bash /ubt_IL/scripts/deploy/train.sh
```

> 续训 13-DOF 时 `DATASET_REPO_ID`/`DATASET_ROOT`/`OUTPUT_DIR` 三个必须显式设为 13-DOF 的值：`train.sh` 始终用环境变量覆盖这几项，默认值是 26-DOF 的，不覆盖会加载错误数据集。若首次训练是被中断（未到 STEPS），用相同的 STEPS 续训即可跑完；要超过原 STEPS 才需调大。

可覆盖的环境变量（CLI 优先级高于配置文件）：

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `CONFIG_PATH` | `/ubt_IL/scripts/deploy/train_config_sim_act.json` | 训练配置文件路径 |
| `DATASET_ROOT` | `/ubt_IL/dataset/sim_pick_place` | 数据集根目录 |
| `DATASET_REPO_ID` | `sim_pick_place` | 数据集 repo id |
| `OUTPUT_DIR` | `/ubt_IL/model/sim_pick_place_act` | 模型与检查点输出目录 |
| `STEPS` | `500000` | 训练总步数 |
| `BATCH_SIZE` | `8` | 批大小 |
| `SEED` | `10000` | 随机种子 |
| `DEVICE` | `cuda` | 训练设备 |
| `HF_HUB_OFFLINE` | `1` | 离线模式，不访问 HuggingFace Hub |

Checkpoint 每 `save_freq`（默认 100000 步）保存一次，另有 `checkpoints/last/pretrained_model`，部署默认使用它。路径：`{OUTPUT_DIR}/checkpoints/last/pretrained_model`。

---

## 4. 模型部署

### 前置条件

- 真机/仿真，`ROS_DOMAIN_ID=0`
- 真机部署请保证本机IP 与机器人 IP 在统一网段：例如192.168.41.99
- ImageServer 已部署到机器人端（仿真部署可跳过）：

```bash
# 将 image_server.py 传到机器人端并启动
scp ubt_IL/scripts/deploy/camera/image_server.py nvidia@192.168.41.2:~
# 机器人端执行：
python3 image_server.py

# 验证图像流（容器内运行）
python3 ubt_IL/scripts/deploy/camera/image_client.py
```

### 仿真部署

仿真使用 `ubt_sim` 模块代替真机，ROS 话题与通信方式与真机一致，用于真机部署前验证。仿真容器与推理容器在同一主机经 `127.0.0.1` 通信。

```bash
# 1. 启动仿真（已启动可跳过）
cd ubt_sim/docker/isaac_sim && bash run.sh bash
bash /ubt_sim/scripts/start_sim.sh        # 按R复位机器人

# 2. 初始化动作（抬起手臂到桌面）
/usr/bin/python3 /ubt_sim/teleoperation/control/reset.py

# 3. 启动推理容器并运行 rollout
cd ubt_IL/docker && bash run.sh bash

# 部署 26-DOF 模型（默认）
POLICY_PATH=/ubt_IL/model/sim_pick_place_act/checkpoints/last/pretrained_model \
DURATION=60 bash /ubt_IL/scripts/deploy/rollout.sh

# 部署 13-DOF 模型（右臂7+右手6，JOINT_CONFIG 须与训练 DOF 一致）
POLICY_PATH=/ubt_IL/model/sim_pick_place_right13_act/checkpoints/last/pretrained_model \
JOINT_CONFIG=tienkung_13 DURATION=60 \
  bash /ubt_IL/scripts/deploy/rollout.sh

# （可选）仿真中回放数据集动作校验链路
/usr/bin/python3 /ubt_IL/scripts/deploy/replay.py \
  --dataset /ubt_IL/dataset/sim_pick_place --episode 0 --rate 30
```

### 真机部署步骤

```bash
# 1. 机器人复位
bash run.sh bash
/usr/bin/python3 /ubt_IL/scripts/deploy/reset.py

# 2. 启动推理（真机 ZMQ_HOST 改为机器人 IP）
POLICY_PATH=/ubt_IL/model/test_model ZMQ_HOST=192.168.41.2 DURATION=60 \
  bash /ubt_IL/scripts/deploy/rollout.sh
```

### 关节 DOF 配置（JOINT_CONFIG）

`JOINT_CONFIG` 决定 policy 的关节维度与顺序，须与模型训练时的数据集顺序一致。配置定义在 `tienkung/lerobot_robot_tienkung/lerobot_robot_tienkung/constants.py` 的 `JOINT_INDEX_ENUMS`：

| `JOINT_CONFIG` | 维度 | 关节 | 说明 |
|----------------|------|------|------|
| `tienkung_26` | 26 | 左臂7+右臂7+左手6+右手6 | 默认，全自由度 |
| `tienkung_13` | 13 | 右臂7+右手6 | 仅右侧，左侧自动用 home 位姿/张开手填充 |

- **policy ↔ 数据集按位映射**：枚举成员顺序须与数据集 action/state 顺序一致（枚举可任意重排以匹配）。
- **policy ↔ 硬件按名散射**：4 个硬件分组为固定物理 motor/手指序（bridge 按位寻址），与枚举顺序无关；二者解耦，故枚举可随意重排/取子集。
- **新增自定义 DOF**：在 `constants.py` 定义一个 `IntEnum`（成员名取自 canonical 26，顺序匹配数据集）并注册到 `JOINT_INDEX_ENUMS`，设 `JOINT_CONFIG=<名>` 即可，无需改逻辑代码。非激活关节默认臂取 `ARM_HOME`、手取 1.0，可用 `INACTIVE_FILL_OVERRIDES` 覆盖。

### lerobot-rollout CLI 直接调用

也可不经过 `rollout.sh`，直接调用 CLI（真机改 `server_address` 为机器人 IP）：

```bash
/lerobot/.venv/bin/lerobot-rollout \
    --policy.path=/ubt_IL/model/test_model \
    --robot.type=tienkung --robot.bridge_enabled=true \
    --robot.joint_config=tienkung_26 \
    --robot.cameras="{head: {type: image_server, server_address: '192.168.41.2', port: 5558, offset_x: 0, width: 640, height: 360, fps: 30, display: true}}" \
    --task="sim_pick_place" --fps=30 --duration=60
```

### 相机 & 部署参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `server_address` | `127.0.0.1` | ImageServer ZMQ 地址（真机改为机器人 IP） |
| `port` | `5558` | ImageServer ZMQ 端口 |
| `offset_x` | `0` | 拼接帧水平偏移（多相机时用） |
| `width` / `height` | `640` / `360` | 截取尺寸 |
| `display` | `true` | 是否弹窗实时显示 |
| `POLICY_PATH` | `.../real_pick_place_act/.../pretrained_model` | 模型路径 |
| `JOINT_CONFIG` | `tienkung_26` | 关节 DOF 配置，须与训练 DOF 一致 |
| `STRATEGY` | `base` | 推理策略（`base`/`sentry`/`highlight`/`dagger`） |
| `TASK` | `sim_pick_place` | 任务描述（注入 policy 条件） |
| `DURATION` | `60` | 运行时长（秒） |
| `FPS` | `30` | 控制环频率（与训练 fps 对齐） |
| `ZMQ_HOST` | `127.0.0.1` | ZMQ 连接主机 |

---
