#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

# 捕获 DISPLAY（sudo 默认不传递），用于容器 X11 转发
_HOST_DISPLAY="${DISPLAY:-}"

WALKER_WS="/ubt_IL/walker/walker_sdk_ros2"

case "${1:-}" in
    build)
        echo "[INFO] Building image: $IMAGE"
        echo "[INFO] Architecture: $ARCH"
        echo "[INFO] Dockerfile: $DOCKERFILE"
        echo "[INFO] Base image: $BASE_IMAGE"
        echo "[INFO] GPU args: $DOCKER_GPU_ARGS"
        echo "[INFO] This may take a few minutes on first build..."
        sudo docker build \
            --build-arg BASE_IMAGE="$BASE_IMAGE" \
            -t "$IMAGE" \
            -f "$DOCKERFILE" \
            "$PROJECT_ROOT"
        echo "[INFO] Image built: $IMAGE"
        ;;
    start)
        # 幂等启动：已运行→提示，存在但停止→start，不存在→run
        if sudo docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "[WARN] Container '$CONTAINER_NAME' is already running."
            exit 0
        fi

        if sudo docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "[INFO] Starting existing container '$CONTAINER_NAME'..."
            sudo docker start "$CONTAINER_NAME"
        else
            echo "[INFO] Creating container '$CONTAINER_NAME'..."
            mkdir -p "${PROJECT_ROOT}/.cache/huggingface"

            GPU_ARGS=()
            if [ -n "$DOCKER_GPU_ARGS" ]; then
                read -r -a GPU_ARGS <<< "$DOCKER_GPU_ARGS"
            fi
            sudo docker run -d --name "$CONTAINER_NAME" \
                "${GPU_ARGS[@]}" \
                --network=host \
                --shm-size=16g \
                --privileged \
                -v /dev:/dev \
                -e DOMAIN_ID="$DOMAIN_ID" \
                -e HF_HOME="$HF_HOME" \
                -e TORCH_HOME="$TORCH_HOME" \
                -e UV_INDEX_URL="$UV_INDEX_URL" \
                -v "$PROJECT_ROOT":/ubt_IL \
                -e DISPLAY="${_HOST_DISPLAY}" \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -v "${XAUTHORITY:-$HOME/.Xauthority}":/home/user_lerobot/.Xauthority:ro \
                -w /ubt_IL \
                "$IMAGE" \
                tail -f /dev/null

            echo "[INFO] Container created."
        fi

        # 等待容器完全启动
        sleep 2

        if ! sudo docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "[ERROR] Container failed to start!"
            echo "[INFO] Check logs: sudo docker logs $CONTAINER_NAME"
            exit 1
        fi

        # 等待 entrypoint 安装完成，同时实时显示安装日志。
        # STARTUP_TIMEOUT=0 表示不超时；如果设置了超时，超时后也不会跳过，
        # 而是直接失败，避免在 lerobot/plugins 仍未安装完时继续使用容器。
        echo "[INFO] Waiting for entrypoint to finish installing lerobot, plugins and messages..."
        STARTUP_TIMEOUT="${STARTUP_TIMEOUT:-0}"
        ELAPSED=0
        IDLE_ELAPSED=0

        # 后台跟踪容器日志（实时输出安装进度）
        sudo docker logs -f "$CONTAINER_NAME" 2>&1 &
        LOG_PID=$!

        is_env_ready() {
            sudo docker exec "$CONTAINER_NAME" bash -lc '/lerobot/.venv/bin/python - <<'"'"'PY'"'"'
import lerobot
from lerobot_robot_tienkung import TienKungRobotConfig
from lerobot_robot_walker import WalkerRobotConfig, WalkerCameraConfig
PY' >/dev/null 2>&1
        }

        is_install_running() {
            sudo docker exec "$CONTAINER_NAME" pgrep -af "uv pip install|colcon build|pip install" >/dev/null 2>&1
        }

        while true; do
            if is_env_ready; then
                break
            fi

            if ! sudo docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
                sudo kill $LOG_PID 2>/dev/null || true
                wait $LOG_PID 2>/dev/null || true
                echo "[ERROR] Container stopped before environment setup completed."
                echo "[INFO] Check logs: sudo docker logs $CONTAINER_NAME"
                exit 1
            fi

            if is_install_running; then
                IDLE_ELAPSED=0
            else
                IDLE_ELAPSED=$((IDLE_ELAPSED + 3))
                if [ $IDLE_ELAPSED -ge 30 ]; then
                    sudo kill $LOG_PID 2>/dev/null || true
                    wait $LOG_PID 2>/dev/null || true
                    echo "[ERROR] Entrypoint setup appears finished, but lerobot/plugins are not importable."
                    echo "[INFO] Check logs: sudo docker logs $CONTAINER_NAME"
                    exit 1
                fi
            fi

            sleep 3
            ELAPSED=$((ELAPSED + 3))
            if [ "$STARTUP_TIMEOUT" != "0" ] && [ $ELAPSED -ge "$STARTUP_TIMEOUT" ]; then
                sudo kill $LOG_PID 2>/dev/null || true
                wait $LOG_PID 2>/dev/null || true
                echo "[ERROR] Environment setup did not complete within ${STARTUP_TIMEOUT}s."
                echo "[INFO] Install may still be running; not proceeding with incomplete setup."
                echo "[INFO] Check logs: sudo docker logs -f $CONTAINER_NAME"
                exit 1
            fi
        done

        # 停止日志跟踪（sudo docker logs 以 root 运行，需 sudo kill）
        sudo kill $LOG_PID 2>/dev/null || true
        wait $LOG_PID 2>/dev/null || true

        echo "[INFO] Environment setup completed (${ELAPSED}s)"

        echo ""
        echo "Next steps:"
        echo "  Enter container:  bash run.sh bash"
        echo "  Check env:        bash run.sh check"
        echo "  Stop container:   bash run.sh stop"
        ;;
    stop)
        if sudo docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "[INFO] Stopping container '$CONTAINER_NAME'..."
            sudo docker stop "$CONTAINER_NAME"
            echo "[INFO] Container stopped."
        else
            echo "[WARN] Container '$CONTAINER_NAME' is not running."
        fi
        ;;
    restart)
        if ! sudo docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "[ERROR] Container '$CONTAINER_NAME' does not exist!"
            echo "[INFO] Create it first: bash run.sh start"
            exit 1
        fi
        bash "$SCRIPT_DIR/run.sh" stop
        bash "$SCRIPT_DIR/run.sh" start
        ;;
    bash)
        if ! sudo docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "[ERROR] Container '$CONTAINER_NAME' is not running!"
            echo "[INFO] Start it first: bash run.sh start"
            exit 1
        fi
        sudo docker exec -it "$CONTAINER_NAME" bash -c "\
            source /opt/ros/humble/setup.bash 2>/dev/null || true; \
            source /opt/bodyctrl_msgs_ws/install/setup.bash 2>/dev/null || true; \
            source $WALKER_WS/install/setup.bash 2>/dev/null || true; \
            export ROS_DOMAIN_ID=$DOMAIN_ID; \
            export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fastdds_no_shm.xml; \
            export DISPLAY=\${DISPLAY:-} 2>/dev/null; \
            bash"
        ;;
    rm)
        # 停止并删除容器
        if sudo docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "[INFO] Stopping running container..."
            sudo docker stop "$CONTAINER_NAME" >/dev/null
        fi
        if sudo docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            sudo docker rm -f "$CONTAINER_NAME" 2>/dev/null || true
            echo "[INFO] Container '$CONTAINER_NAME' removed."
        else
            echo "[WARN] Container '$CONTAINER_NAME' does not exist."
        fi
        ;;
    check)
        if ! sudo docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "[ERROR] Container '$CONTAINER_NAME' is not running!"
            exit 1
        fi
        echo "=========================================="
        echo "  LeRobot TienKung Environment Check"
        echo "=========================================="
        echo ""
        echo "Host arch:      $ARCH"
        CONTAINER_ARCH=$(sudo docker exec "$CONTAINER_NAME" uname -m 2>/dev/null || echo "unknown")
        echo "Container arch: $CONTAINER_ARCH"
        echo "Image:          $IMAGE"
        echo ""

        ERRORS=0
        WARNINGS=0

        # 项目挂载
        if sudo docker exec "$CONTAINER_NAME" test -d /ubt_IL; then
            echo "[OK] Project mounted: /ubt_IL"
        else
            echo "[FAIL] Project NOT mounted!"
            ERRORS=$((ERRORS + 1))
        fi

        # lerobot 导入
        if sudo docker exec "$CONTAINER_NAME" /lerobot/.venv/bin/python -c "import lerobot" 2>/dev/null; then
            echo "[OK] lerobot package: installed"
        else
            echo "[FAIL] lerobot package: NOT installed"
            ERRORS=$((ERRORS + 1))
        fi

        # tienkung 插件导入
        if sudo docker exec "$CONTAINER_NAME" /lerobot/.venv/bin/python -c "from lerobot_robot_tienkung import TienKungRobotConfig" 2>/dev/null; then
            echo "[OK] tienkung plugin: installed"
        else
            echo "[FAIL] tienkung plugin: NOT installed"
            ERRORS=$((ERRORS + 1))
        fi

        # Walker 插件导入（如果已迁移 walker/ 目录）
        if sudo docker exec "$CONTAINER_NAME" test -d /ubt_IL/walker/lerobot_robot_walker; then
            if sudo docker exec "$CONTAINER_NAME" /lerobot/.venv/bin/python -c "from lerobot_robot_walker import WalkerRobotConfig, WalkerCameraConfig" 2>/dev/null; then
                echo "[OK] walker plugin: installed"
            else
                echo "[FAIL] walker plugin: NOT installed"
                ERRORS=$((ERRORS + 1))
            fi
        else
            echo "[WARN] walker plugin: /ubt_IL/walker/lerobot_robot_walker not found"
            WARNINGS=$((WARNINGS + 1))
        fi

        # Walker Bridge2 脚本
        if sudo docker exec "$CONTAINER_NAME" test -f /ubt_IL/walker/ros2_walker_bridge.py; then
            echo "[OK] walker bridge: available"
        else
            echo "[WARN] walker bridge: /ubt_IL/walker/ros2_walker_bridge.py not found"
            WARNINGS=$((WARNINGS + 1))
        fi

        # Walker ROS2 messages
        if sudo docker exec "$CONTAINER_NAME" bash -lc "source /opt/ros/humble/setup.bash 2>/dev/null || true; source $WALKER_WS/install/setup.bash 2>/dev/null || true; /usr/bin/python3 - <<'PY'
from mc_state_msgs.msg import RobotState
from mc_task_msgs.msg import JointCmd, JointCommand, RobotCommand
from shm_msgs.msg import Image2m
PY" 2>/dev/null; then
            echo "[OK] walker ROS2 msgs: installed"
        else
            echo "[WARN] walker ROS2 msgs: NOT installed"
            WARNINGS=$((WARNINGS + 1))
        fi

        # ROS2 Humble
        if sudo docker exec "$CONTAINER_NAME" test -f /opt/ros/humble/setup.bash 2>/dev/null; then
            echo "[OK] ROS2 Humble: installed"
        else
            echo "[FAIL] ROS2 Humble: NOT installed"
            ERRORS=$((ERRORS + 1))
        fi

        # bodyctrl_msgs: x86 由 deb 装入 /opt/ros/humble，arm64 由源码编译到 /opt/bodyctrl_msgs_ws
        if sudo docker exec "$CONTAINER_NAME" bash -lc \
          "source /opt/ros/humble/setup.bash 2>/dev/null; source /opt/bodyctrl_msgs_ws/install/setup.bash 2>/dev/null; /usr/bin/python3 -c 'from bodyctrl_msgs.msg import MotorCtrl'" 2>/dev/null; then
            echo "[OK] bodyctrl_msgs: available"
        else
            echo "[WARN] bodyctrl_msgs: NOT available"
            WARNINGS=$((WARNINGS + 1))
        fi

        # GPU
        if sudo docker exec "$CONTAINER_NAME" nvidia-smi >/dev/null 2>&1; then
            GPU=$(sudo docker exec "$CONTAINER_NAME" nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
            echo "[OK] GPU: $GPU"
        else
            echo "[FAIL] GPU: not detected"
            ERRORS=$((ERRORS + 1))
        fi

        # 网络
        NET=$(sudo docker inspect --format='{{.HostConfig.NetworkMode}}' "$CONTAINER_NAME" 2>/dev/null || echo "unknown")
        if [ "$NET" == "host" ]; then
            echo "[OK] Network: host mode"
        else
            echo "[FAIL] Network: $NET (expected host mode)"
            ERRORS=$((ERRORS + 1))
        fi

        echo ""
        echo "=========================================="
        if [ $ERRORS -eq 0 ] && [ $WARNINGS -eq 0 ]; then
            echo "  All checks passed!"
        elif [ $ERRORS -eq 0 ]; then
            echo "  Checks passed with $WARNINGS warning(s)"
        else
            echo "  $ERRORS error(s), $WARNINGS warning(s)"
            exit 1
        fi
        echo "=========================================="
        ;;
    *)
        echo "Usage: $0 {build|start|stop|restart|bash|rm|check}"
        echo ""
        echo "Commands:"
        echo "  build         Build the Docker image"
        echo "  start         Create and/or start the container (idempotent)"
        echo "  stop          Stop the container"
        echo "  restart       Restart the container"
        echo "  bash          Enter the container shell (with ROS2 env)"
        echo "  rm            Remove the container"
        echo "  check         Verify container environment"
        exit 1
        ;;
esac
