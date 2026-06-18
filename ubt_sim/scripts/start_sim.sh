#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

BRIDGE_PID=""

cleanup() {
    if [ -n "$BRIDGE_PID" ]; then
        echo "[INFO] Stopping ROS2-ZMQ bridge (PID=$BRIDGE_PID)..."
        kill "$BRIDGE_PID" 2>/dev/null
        # Wait up to 3 seconds for graceful shutdown
        for i in 1 2 3; do
            if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
                break
            fi
            sleep 1
        done
        # Force kill if still running
        if kill -0 "$BRIDGE_PID" 2>/dev/null; then
            kill -9 "$BRIDGE_PID" 2>/dev/null
        fi
        wait "$BRIDGE_PID" 2>/dev/null
    fi
    exit 0
}
trap cleanup EXIT INT TERM

# Auto-start ROS2-ZMQ bridge (unless disabled)
if [ -z "$UBT_SIM_NO_BRIDGE" ]; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
        export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
        /usr/bin/python3 "$PROJECT_DIR/teleoperation/bridges/ros2_zmq_bridge.py" &
        BRIDGE_PID=$!
        echo "[INFO] ROS2-ZMQ bridge started (PID=$BRIDGE_PID)"
        sleep 2
    else
        echo "[WARN] ROS2 not found in container. Skipping bridge. Set UBT_SIM_NO_BRIDGE=1 to suppress."
    fi
fi

# Launch simulation
UBT_SIM_TASK="${UBT_SIM_TASK:-UBTSim-TiangongPro-Parlor-v0}"
UBT_SIM_NUM_ENVS="${UBT_SIM_NUM_ENVS:-1}"
EXTRA_ARGS=()
if [ -n "$UBT_SIM_LOAD_ONLY" ]; then
    EXTRA_ARGS+=(--load_only)
    if [ -z "$UBT_SIM_LOAD_ONLY_KEEP_DEVICE" ]; then
        EXTRA_ARGS+=(--device "${UBT_SIM_LOAD_ONLY_DEVICE:-cpu}")
    fi
fi

/isaac-sim/python.sh "$SCRIPT_DIR/sim_runner.py" \
    --task "$UBT_SIM_TASK" \
    --enable_cameras \
    --num_envs "$UBT_SIM_NUM_ENVS" \
    "${EXTRA_ARGS[@]}" \
    "$@"
