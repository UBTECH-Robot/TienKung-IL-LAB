#!/bin/bash
# 相机通路验证客户端（env_vla 3.12，无界面，连 image_server ZMQ 5558）
# 用于在启动 rollout 前先确认 image_server -> 客户端 的图像通路正常。
# 决策与依据见同目录 README.md。
#
# 用法:
#   bash image_client_host.sh                  # 持续接收, Ctrl-C 停并出结论
#   bash image_client_host.sh --count 60       # 收 60 帧后自动停并出结论
#   bash image_client_host.sh --address 192.168.41.2   # 连真机上的 image_server
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONDA_BASE="${CONDA_BASE:-$(conda info --base 2>/dev/null || echo "$HOME/miniconda3")}"
ENV_NAME="${ENV_NAME:-env_vla}"

# shellcheck disable=SC1091
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate "$ENV_NAME"
echo "[camera_client] python: $(which python)"

exec python "$SCRIPT_DIR/image_client.py" "$@"
