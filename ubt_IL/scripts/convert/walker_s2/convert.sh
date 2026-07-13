#!/bin/bash
# Walker S2 真机 HDF5 -> LeRobot v3.0 数据集转换
# 容器内运行（需 lerobot 在 PYTHONPATH，默认 /ubt_IL/lerobot/src）。
#
# 源目录可传单个 episode，也可传整个 walker-s2-real-data 根目录：
# 传根目录时脚本会批量扫描每个子目录下的 hdf5/metadata_aligned.hdf5 并合并成一个数据集。
#
# 可选追加转换器参数，例如：
#   bash convert.sh --overwrite        # 覆盖已有输出
#   bash convert.sh --save_one true    # 只转第一条 episode
#   bash convert.sh --fps auto         # 自动估 fps（保持真实采集时序）
#   SRC_ROOT=/ubt_IL/dataset/walker-s2-real-data/<single_episode> bash convert.sh
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 默认参数（可通过环境变量覆盖）
SRC_ROOT="${SRC_ROOT:-/ubt_IL/dataset/walker-s2-real-data}"
TGT_PATH="${TGT_PATH:-/ubt_IL/dataset}"
CONFIG="${CONFIG:-$SCRIPT_DIR/configs/walker_s2_real_19d_1RGBD.json}"
REPO_ID="${REPO_ID:-Walker_S2_real_19_1RGBD}"
FPS="${FPS:-12.5}"
ROBOT_TYPE="${ROBOT_TYPE:-walker_s2}"
TASK_NAME="${TASK_NAME:-walker_s2_real}"
PYTHONPATH="${PYTHONPATH:-/ubt_IL/lerobot/src}"
export PYTHONPATH

python "$SCRIPT_DIR/convert_real_to_lerobot_v3.py" \
  --config "$CONFIG" \
  --src_root "$SRC_ROOT" \
  --tgt_path "$TGT_PATH" \
  --repo_id "$REPO_ID" \
  --task_name "$TASK_NAME" \
  --robot_type "$ROBOT_TYPE" \
  --fps "$FPS" \
  "$@"
