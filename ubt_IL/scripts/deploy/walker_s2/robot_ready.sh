#!/bin/bash
# Walker S2 机器人预备姿态（分步安全到位）
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROBOT_CTRL="$SCRIPT_DIR/../../../walker/walker_sdk_ros2/robot_control/robot_control.py"

exec /usr/bin/python3 "$ROBOT_CTRL" --init
