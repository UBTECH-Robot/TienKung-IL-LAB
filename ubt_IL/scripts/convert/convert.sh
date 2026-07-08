#!/bin/bash
# HDF5 -> LeRobot 数据集转换
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# 默认参数（可通过环境变量覆盖）
SRC_ROOT="${SRC_ROOT:-$PROJECT_ROOT/dataset/sim_pick_place_hdf5}"
TGT_PATH="${TGT_PATH:-/ubt_IL/dataset}"
CONFIG="${CONFIG:-$SCRIPT_DIR/configs/Tien_Kung_26_1RGB_sim.json}"
REPO_ID="${REPO_ID:-sim_pick_place}"
FPS="${FPS:-30}"
ROBOT_TYPE="${ROBOT_TYPE:-tiangong}"
TASK_NAME="${TASK_NAME:-sim_pick_place}"
VCODEC="${VCODEC:-h264}"

python "$SCRIPT_DIR/convert_to_lerobot.py" \
  --config "$CONFIG" \
  --repo_id "$REPO_ID" \
  --src_root "$SRC_ROOT" \
  --tgt_path "$TGT_PATH" \
  --fps "$FPS" \
  --robot_type "$ROBOT_TYPE" \
  --task_name "$TASK_NAME" \
  --vcodec "$VCODEC" \
  "$@"
