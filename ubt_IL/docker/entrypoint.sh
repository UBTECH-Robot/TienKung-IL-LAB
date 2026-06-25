#!/bin/bash
set -e

# Source ROS2 Humble environment
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Fast-DDS: disable shared memory transport (required for Docker, even with --network=host)
# Without this, ros2 topic list works but ros2 topic echo / subscribe fails
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fastdds_no_shm.xml

build_walker_ros2_msgs() {
    local walker_ws="/ubt_IL/walker/walker_sdk_ros2"
    local walker_msgs="shm_msgs mc_state_msgs mc_task_msgs emb_task_msgs sys_task_msgs rosa_msgs ecat_task_msgs"

    if [ ! -d "$walker_ws" ]; then
        return 0
    fi

    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    fi
    local ros_pythonpath="$PYTHONPATH"
    if [ -f "$walker_ws/install/setup.bash" ]; then
        source "$walker_ws/install/setup.bash"
    fi

    if /usr/bin/python3 - <<'PY' >/dev/null 2>&1
from mc_state_msgs.msg import RobotState
from mc_task_msgs.msg import JointCmd, JointCommand, RobotCommand
from shm_msgs.msg import Image2m
PY
    then
        echo "[entrypoint] Walker ROS2 messages already available."
        return 0
    fi

    echo "[entrypoint] Building Walker ROS2 message packages..."
    (
        cd "$walker_ws"
        # A failed/stale CMake cache can keep /lerobot/.venv/bin/python3 as the
        # rosidl generator even after PATH is fixed, so rebuild messages cleanly.
        rm -rf build install log
        unset VIRTUAL_ENV PYTHONHOME
        export PYTHONPATH="$ros_pythonpath"
        export AMENT_PYTHON_EXECUTABLE=/usr/bin/python3
        export Python3_EXECUTABLE=/usr/bin/python3
        export PYTHON_EXECUTABLE=/usr/bin/python3
        export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
        /usr/bin/colcon build --packages-select $walker_msgs \
            --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
    ) || {
        echo "[entrypoint] WARNING: Walker ROS2 message build failed"
        return 0
    }

    if [ -f "$walker_ws/install/setup.bash" ]; then
        source "$walker_ws/install/setup.bash"
    fi
}

# ROS_DOMAIN_ID: 默认 0 (真机)，可通过 DOMAIN_ID 环境变量覆盖
if [ -n "$DOMAIN_ID" ]; then
    export ROS_DOMAIN_ID="$DOMAIN_ID"
else
    export ROS_DOMAIN_ID=0
fi

# Ensure HuggingFace cache directory exists (inside /ubt_IL mount, always writable)
if [ -n "$HF_HOME" ]; then
    mkdir -p "$HF_HOME" 2>/dev/null || true
fi

# 运行时安装（如果挂载了项目目录）
# 挂载路径为 /ubt_IL，避免覆盖基础镜像的 /lerobot/.venv/
if [ -d "/ubt_IL" ]; then
    # Activate base image venv for subsequent uv commands
    export VIRTUAL_ENV=/lerobot/.venv
    export PATH="/lerobot/.venv/bin:$PATH"

    # Install lerobot from source (editable) if not already
    if ! python -c "import lerobot; assert '/ubt_IL/lerobot/' in lerobot.__file__" 2>/dev/null; then
        echo "[entrypoint] Installing lerobot from /ubt_IL/lerobot (editable)..."
        uv pip install "numpy<2" || true
        cd /ubt_IL/lerobot && uv pip install -e . || echo "[entrypoint] WARNING: lerobot install failed"
    fi

    # Build/source Walker ROS2 messages before installing the Python plugin.
    build_walker_ros2_msgs

    # Install TienKung plugin (editable)
    if [ -d "/ubt_IL/tienkung/lerobot_robot_tienkung" ]; then
        echo "[entrypoint] Installing lerobot-robot-tienkung plugin..."
        uv pip install -e /ubt_IL/tienkung/lerobot_robot_tienkung || echo "[entrypoint] WARNING: tienkung plugin install failed"
    fi

    # Install Walker plugin (editable)
    if [ -d "/ubt_IL/walker/lerobot_robot_walker" ]; then
        echo "[entrypoint] Installing lerobot-robot-walker plugin..."
        uv pip install -e /ubt_IL/walker/lerobot_robot_walker || echo "[entrypoint] WARNING: walker plugin install failed"
    fi

    # Replace headless OpenCV with GUI version (MUST be after lerobot install)
    # lerobot's dependencies pull in opencv-python-headless + numpy>=2, so we fix it last.
    # Check via pip list (not import) because numpy mismatch may crash cv2 import.
    if uv pip list 2>/dev/null | grep -q "opencv-python-headless"; then
        echo "[entrypoint] Replacing opencv-python-headless with opencv-python (GUI support)..."
        uv pip uninstall opencv-python-headless opencv-python -y 2>/dev/null || true
        uv pip install "opencv-python<4.10" "numpy<2" || echo "[entrypoint] WARNING: opencv upgrade failed"
    fi
fi

exec "$@"
