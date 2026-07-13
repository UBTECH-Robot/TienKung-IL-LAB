#!/bin/bash
# 合并多个 LeRobot 数据集 -> 单个数据集
# 使用 lerobot 容器的 lerobot-edit-dataset 工具（merge 操作）
# 在 lerobot-tienkung 容器内运行（路径为容器内路径 /ubt_IL/...）
#
# 用法：
#   1) 默认合并全部 10 个分任务数据集 -> Pick_up_merged
#      bash merge_datasets.sh
#
#   2) 自定义输入（空格分隔的目录名，位于 $DATASET_ROOT 下）与输出名
#      INPUT_DATASETS="Pick_up_the_apple_1 Pick_up_the_apple_2" \
#        OUTPUT_DATASET=Pick_up_apple bash merge_datasets.sh
#
#   3) 覆盖已存在的输出目录
#      OVERWRITE=1 bash merge_datasets.sh
#
# 说明：
#   - 合并按「任务字符串」去重保留：apple / bottle / red_bottle 会作为 3 个不同
#     任务保留（与已存在的 Pick_up_tiangong_all 把任务统一成单一标签不同）。
#   - 任务按字符串去重，因此 Pick_up_the_red_bottle_4 中带前导制表符的
#     '\t Pick up the red bottle' 会被当作额外任务混入；脚本会检测并告警，
#     建议先修复其 tasks.parquet 再合并。
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 容器环境校验
if [ ! -x /lerobot/.venv/bin/lerobot-edit-dataset ]; then
    echo "错误：未找到 /lerobot/.venv/bin/lerobot-edit-dataset，" >&2
    echo "      请在 lerobot-tienkung 容器内运行此脚本。" >&2
    exit 1
fi

# === 配置（环境变量可覆盖）===
DATASET_ROOT="${DATASET_ROOT:-/ubt_IL/dataset}"
OUTPUT_DATASET="${OUTPUT_DATASET:-Pick_up_merged}"
# 默认合并 10 个分任务数据集
DEFAULT_INPUTS="Pick_up_the_apple_1 Pick_up_the_apple_2 \
Pick_up_the_bottle_1 Pick_up_the_bottle_2 Pick_up_the_bottle_3 Pick_up_the_bottle_4 \
Pick_up_the_red_bottle_1 Pick_up_the_red_bottle_2 Pick_up_the_red_bottle_3 Pick_up_the_red_bottle_4"
INPUT_DATASETS="${INPUT_DATASETS:-$DEFAULT_INPUTS}"
PUSH_TO_HUB="${PUSH_TO_HUB:-false}"
OVERWRITE="${OVERWRITE:-0}"

OUTPUT_DIR="$DATASET_ROOT/$OUTPUT_DATASET"

HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"
export HF_HUB_OFFLINE

cd /ubt_IL/lerobot

# 解析输入列表为数组
read -r -a INPUTS <<< "$INPUT_DATASETS"
if [ "${#INPUTS[@]}" -eq 0 ]; then
    echo "错误：INPUT_DATASETS 为空。" >&2
    exit 1
fi

# 校验输入数据集，并构造 repo_ids / roots 的 Python 字面量列表
REPO_IDS_STR="["
ROOTS_STR="["
first=1
for name in "${INPUTS[@]}"; do
    src="$DATASET_ROOT/$name"
    if [ ! -f "$src/meta/info.json" ]; then
        echo "错误：输入数据集不存在或非 LeRobot 数据集：$src" >&2
        exit 1
    fi
    # 检测 red_bottle_4 任务标签 typo（前导制表符导致的重复任务）
    if [ "$name" = "Pick_up_the_red_bottle_4" ]; then
        echo "警告：$name 含带前导制表符的重复任务 '\\t Pick up the red bottle'，" >&2
        echo "      合并后会作为额外任务混入，建议先修复其 tasks.parquet。" >&2
    fi
    if [ $first -ne 1 ]; then
        REPO_IDS_STR+=", "
        ROOTS_STR+=", "
    fi
    REPO_IDS_STR+="'$name'"
    ROOTS_STR+="'$src'"
    first=0
done
REPO_IDS_STR+="]"
ROOTS_STR+="]"

# 输出目录处理（lerobot-edit-dataset 要求输出目录不能已存在）
if [ -e "$OUTPUT_DIR" ]; then
    if [ "$OVERWRITE" = "1" ]; then
        echo "OVERWRITE=1，删除已有输出目录：$OUTPUT_DIR"
        rm -rf "$OUTPUT_DIR"
    else
        echo "错误：输出目录已存在：$OUTPUT_DIR" >&2
        echo "      设置 OVERWRITE=1 可覆盖。" >&2
        exit 1
    fi
fi

echo "============================================================"
echo "合并数据集 (lerobot-edit-dataset merge)"
echo "  输入 (${#INPUTS[@]}): ${INPUTS[*]}"
echo "  输出: $OUTPUT_DIR  (repo_id=$OUTPUT_DATASET)"
echo "  push_to_hub=$PUSH_TO_HUB  HF_HUB_OFFLINE=$HF_HUB_OFFLINE"
echo "============================================================"

/lerobot/.venv/bin/lerobot-edit-dataset \
    --new_repo_id="$OUTPUT_DATASET" \
    --new_root="$OUTPUT_DIR" \
    --operation.type=merge \
    --operation.repo_ids="$REPO_IDS_STR" \
    --operation.roots="$ROOTS_STR" \
    --push_to_hub="$PUSH_TO_HUB"

echo "------------------------------------------------------------"
echo "合并完成：$OUTPUT_DIR"
# 打印汇总（读取 info.json；python3 不可用时静默跳过）
python3 - "$OUTPUT_DIR/meta/info.json" <<'PY' 2>/dev/null || true
import json, sys
with open(sys.argv[1]) as f:
    info = json.load(f)
print(f"  total_episodes = {info.get('total_episodes')}")
print(f"  total_frames   = {info.get('total_frames')}")
print(f"  total_tasks    = {info.get('total_tasks')}")
print(f"  fps            = {info.get('fps')}")
PY
