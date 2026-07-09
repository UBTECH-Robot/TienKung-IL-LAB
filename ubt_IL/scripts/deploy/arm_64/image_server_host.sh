#!/bin/bash
# 天工相机图像服务 host 版（系统 python3 + pyorbbecsdk，ZMQ 5558）
# 独立进程：相机硬件 -> ZMQ 5558 (JPEG) -> LeRobot ImageServerCamera（env_vla 侧连接）。
# pyorbbecsdk 只有 cp310（在 ~/.local），故用系统 python3.10，不入 env_vla。
# 决策与依据见同目录 README.md。
#
# 用法:
#   bash image_server_host.sh
#
# 部署顺序：先启动本脚本（相机服务），再启动 rollout_host.sh（推理）。
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"   # -> .../ubt_IL

echo "[image_server] python: $(/usr/bin/python3 --version 2>&1) (system, pyorbbecsdk cp310)"

# image_server.py 依赖 cv2/zmq/numpy/pyorbbecsdk，均在系统 python 的 ~/.local 下可用
exec /usr/bin/python3 "$PROJECT_ROOT/scripts/deploy/camera/image_server.py" "$@"
