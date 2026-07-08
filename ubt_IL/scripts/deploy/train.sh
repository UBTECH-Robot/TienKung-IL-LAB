#!/bin/bash
# 天工机器人 ACT 模型训练脚本
# 通过 --config_path 加载 train_config_sim_act.json 作为完整训练配置
# （含 input_features、网络结构、归一化、优化器等）；
# 环境变量可覆盖常用参数，CLI 参数优先级高于配置文件（draccus 合并时 CLI 优先）。
# 在 lerobot-tienkung 容器内运行
set -e

# === 配置 ===
CONFIG_PATH="${CONFIG_PATH:-/ubt_IL/scripts/deploy/train_config_sim_act.json}"
DATASET_ROOT="${DATASET_ROOT:-/ubt_IL/dataset/sim_pick_place}"
DATASET_REPO_ID="${DATASET_REPO_ID:-sim_pick_place}"
OUTPUT_DIR="${OUTPUT_DIR:-/ubt_IL/model/sim_pick_place_act}"
STEPS="${STEPS:-50000}"
BATCH_SIZE="${BATCH_SIZE:-8}"
SEED="${SEED:-1000}"
DEVICE="${DEVICE:-cuda}"
HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"

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
    --batch_size="$BATCH_SIZE" \
    --seed="$SEED" \
    --policy.device="$DEVICE" \
    --wandb.enable=false
