#!/bin/bash
# 天工机器人 ACT 模型训练脚本
# 通过 --config_path 加载 train_config_sim_act.json 作为完整训练配置
# （含 input_features、网络结构、归一化、优化器等）；
# 环境变量可覆盖常用参数，CLI 参数优先级高于配置文件（draccus 合并时 CLI 优先）。
# 在 lerobot-tienkung 容器内运行
#
# 用法示例：
#   从头训练（务必换一个新的 OUTPUT_DIR，否则 resume=false 时已存在目录会报 FileExistsError）：
#     OUTPUT_DIR=/ubt_IL/model/sim_pick_place_act_v2 bash train.sh
#   从最近一次 checkpoint 续训（resume=true 时 CONFIG_PATH 必须指向 checkpoint 内的 train_config.json，
#   lerobot 会据其相对位置定位 model.safetensors 与 training_state）：
#     CONFIG_PATH=/ubt_IL/model/sim_pick_place_act/checkpoints/last/pretrained_model/train_config.json \
#       RESUME=true bash train.sh
set -e

# === 配置 ===
CONFIG_PATH="${CONFIG_PATH:-/ubt_IL/scripts/deploy/train_config_sim_act.json}"
DATASET_ROOT="${DATASET_ROOT:-/ubt_IL/dataset/sim_pick_place}"
DATASET_REPO_ID="${DATASET_REPO_ID:-sim_pick_place}"
OUTPUT_DIR="${OUTPUT_DIR:-/ubt_IL/model/sim_pick_place_act}"
STEPS="${STEPS:-50000}"
SAVE_FREQ="${SAVE_FREQ:-10000}"  # checkpoint 保存间隔（步）；如 50K 步→每 10K 存一次共 5 个
BATCH_SIZE="${BATCH_SIZE:-8}"
SEED="${SEED:-10000}"
DEVICE="${DEVICE:-cuda}"
HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"
# 续训开关：true=断点续训（需配合 CONFIG_PATH 指向 checkpoint 内的 train_config.json）
RESUME="${RESUME:-false}"

cd /ubt_IL/lerobot
export HF_HUB_OFFLINE

# 完整 ACT 训练配置见 $CONFIG_PATH；以下 CLI 参数覆盖配置文件中的同名字段。
# 注意：input_features 固定为 RGB 头部图像 + 状态（不含 head_depth），
# 以便与 rollout 部署（仅提供 head RGB 相机）保持一致。
/lerobot/.venv/bin/lerobot-train \
    --config_path="$CONFIG_PATH" \
    --dataset.repo_id="$DATASET_REPO_ID" \
    --dataset.root="$DATASET_ROOT" \
    --output_dir="$OUTPUT_DIR" \
    --steps="$STEPS" \
    --save_freq="$SAVE_FREQ" \
    --batch_size="$BATCH_SIZE" \
    --seed="$SEED" \
    --policy.device="$DEVICE" \
    --resume="$RESUME" \
    --wandb.enable=false
