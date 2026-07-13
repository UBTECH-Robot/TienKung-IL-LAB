#!/bin/bash
# 天工机器人 host 版 train（去 Docker，conda env_vla + Python 3.12）
# ACT 模型训练。决策与依据见同目录 README.md。
#
# 用法:
#   bash train_host.sh
#   STEPS=100000 BATCH_SIZE=8 bash train_host.sh
#
# 注意：数据集目录名为 Pick_up_the_apple_all，而 train_config_tiangong_all.json 中
#       repo_id=Pick_up_tiangong_all。LeRobotDataset 以 repo_id 在 root 下找同名文件夹，
#       若不匹配需统一（重命名数据集目录或改 repo_id）。
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"   # -> .../ubt_IL
CONDA_BASE="${CONDA_BASE:-$(conda info --base 2>/dev/null || echo "$HOME/miniconda3")}"
ENV_NAME="${ENV_NAME:-env_vla}"

# === 配置（可环境变量覆盖） ===
CONFIG_PATH="${CONFIG_PATH:-$PROJECT_ROOT/scripts/deploy/train_config_tiangong_all.json}"
DATASET_ROOT="${DATASET_ROOT:-$PROJECT_ROOT/dataset}"                 # LeRobotDataset root（repo_id 同名文件夹的父目录）
DATASET_REPO_ID="${DATASET_REPO_ID:-Pick_up_the_apple_all}"           # 与实际数据集目录名一致
OUTPUT_DIR="${OUTPUT_DIR:-$PROJECT_ROOT/model/Pick_up_tiangong_all_act}"
STEPS="${STEPS:-500000}"
BATCH_SIZE="${BATCH_SIZE:-8}"
SEED="${SEED:-10000}"
DEVICE="${DEVICE:-cuda}"
HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"

# === 激活 env_vla (3.12) ===
# shellcheck disable=SC1091
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate "$ENV_NAME"

echo "[train] python: $(which python)"
echo "[train] CONFIG_PATH : $CONFIG_PATH"
echo "[train] DATASET_ROOT: $DATASET_ROOT (repo_id=$DATASET_REPO_ID)"
echo "[train] OUTPUT_DIR  : $OUTPUT_DIR"

cd "$PROJECT_ROOT/lerobot"
export HF_HUB_OFFLINE

# --dataset.root / --output_dir 覆盖配置文件里的 /ubt_IL/... 容器路径
lerobot-train \
    --config_path="$CONFIG_PATH" \
    --dataset.repo_id="$DATASET_REPO_ID" \
    --dataset.root="$DATASET_ROOT" \
    --output_dir="$OUTPUT_DIR" \
    --steps="$STEPS" \
    --batch_size="$BATCH_SIZE" \
    --seed="$SEED" \
    --policy.device="$DEVICE" \
    --wandb.enable=false
