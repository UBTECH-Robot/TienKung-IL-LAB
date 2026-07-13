#!/bin/bash
# Walker S2 模型训练（lerobot-train + config_path 方式）
# 容器内运行：cd /ubt_IL/lerobot 后执行。
#
# 默认用仿真 ACT 配置；切 Pi0.5 / 真机配置用 CONFIG 环境变量，例如：
#   CONFIG=configs/train_config_walker_s2_sim_pi05.json bash train.sh
#   CONFIG=$SCRIPT_DIR/configs/train_config_walker_s2_real_act_19d.json bash train.sh
#
# 可选覆盖（不设则沿用 config 自带值，避免覆盖 Pi0.5 的 steps/lr 等）：
#   OUTPUT_DIR=... STEPS=... BATCH_SIZE=... DEVICE=cuda bash train.sh
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

CONFIG="${CONFIG:-$SCRIPT_DIR/configs/train_config_walker_s2_sim.json}"
HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"
export HF_HUB_OFFLINE

# 仅在用户显式设置时才作为 CLI 覆盖传入，否则沿用 config 文件里的值。
OVERRIDES=()
[ -n "${OUTPUT_DIR+x}" ] && OVERRIDES+=(--output_dir="$OUTPUT_DIR")
[ -n "${STEPS+x}" ] && OVERRIDES+=(--steps="$STEPS")
[ -n "${BATCH_SIZE+x}" ] && OVERRIDES+=(--batch_size="$BATCH_SIZE")
[ -n "${DEVICE+x}" ] && OVERRIDES+=(--policy.device="$DEVICE")
[ -n "${SEED+x}" ] && OVERRIDES+=(--seed="$SEED")

cd /ubt_IL/lerobot

/lerobot/.venv/bin/lerobot-train \
    --config_path="$CONFIG" \
    "${OVERRIDES[@]}" \
    "$@"
