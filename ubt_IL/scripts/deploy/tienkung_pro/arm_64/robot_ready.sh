#!/bin/bash
# 天工机器人 host 版 reset（系统 python3 + ROS2，不需要 env_vla）
# 模型推理前把机器人复位到预设位置。决策与依据见同目录 README.md。
#
# 用法:
#   bash robot_ready.sh
#   bash robot_ready.sh --config-file /tmp/tienkung_bridge_config.json
#
# 注意：reset.py 默认读 /tmp/tienkung_bridge_config.json（由 TienKungRobot._start_bridge()
#       在 rollout 启动 Bridge2 时写出）。若该文件不存在，使用 reset.py 内置默认值。
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"   # -> .../ubt_IL

# ROS2 Humble 环境（宿主机已装）
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
else
    echo "[reset] ERROR: /opt/ros/humble/setup.bash 不存在" >&2
    exit 1
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "[reset] python: $(which python3) | ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# reset.py 用系统 python3（有 rclpy + bodyctrl_msgs）
exec /usr/bin/python3 "$PROJECT_ROOT/scripts/deploy/tienkung_pro/reset.py" "$@"
