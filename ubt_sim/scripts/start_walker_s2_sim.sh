#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

BRIDGE_PID=""

cleanup() {
    if [ -n "$BRIDGE_PID" ]; then
        echo "[INFO] Stopping Walker S2 ROS2-ZMQ bridge (PID=$BRIDGE_PID)..."
        kill "$BRIDGE_PID" 2>/dev/null
        for i in 1 2 3; do
            if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
                break
            fi
            sleep 1
        done
        if kill -0 "$BRIDGE_PID" 2>/dev/null; then
            kill -9 "$BRIDGE_PID" 2>/dev/null
        fi
        wait "$BRIDGE_PID" 2>/dev/null
    fi
    exit 0
}
trap cleanup EXIT INT TERM

UBT_SIM_TASK="${UBT_SIM_TASK:-UBTSim-WalkerS2-PartSorting-v0}"
UBT_SIM_NUM_ENVS="${UBT_SIM_NUM_ENVS:-1}"
UBT_SIM_WALKER_S2_BRIDGE_CONFIG="${UBT_SIM_WALKER_S2_BRIDGE_CONFIG:-$PROJECT_DIR/teleoperation/bridges/walker_s2_bridge_config.yaml}"
# Walker S2 uses GPU rendering but CPU PhysX by default. GPU PhysX triggers
# Isaac Sim 5.0 / Isaac Lab 2.2 getVelocities device-mismatch spam during
# articulation initialization. Override only for experiments:
#   UBT_SIM_WALKER_S2_PHYSICS_DEVICE=cuda:0 bash scripts/start_walker_s2_sim.sh

if [ -z "$UBT_SIM_NO_BRIDGE" ]; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
        if [ -f /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash ]; then
            source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
        else
            echo "[ERROR] Walker SDK ROS2 messages not built: /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash"
            echo "[INFO] Run inside container: cd /ubt_sim/docker/isaac_sim && bash run.sh init"
            exit 1
        fi
        export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
        export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
        /usr/bin/python3 "$PROJECT_DIR/teleoperation/bridges/walker_s2_ros2_zmq_bridge.py" \
            --config "$UBT_SIM_WALKER_S2_BRIDGE_CONFIG" &
        BRIDGE_PID=$!
        echo "[INFO] Walker S2 ROS2-ZMQ bridge started (PID=$BRIDGE_PID)"
        sleep 2
    else
        echo "[WARN] ROS2 not found in container. Skipping bridge. Set UBT_SIM_NO_BRIDGE=1 to suppress."
    fi
fi

EXTRA_ARGS=()
if [ -n "$UBT_SIM_LOAD_ONLY" ]; then
    EXTRA_ARGS+=(--load_only)
    if [ -z "$UBT_SIM_LOAD_ONLY_KEEP_DEVICE" ]; then
        EXTRA_ARGS+=(--device "${UBT_SIM_LOAD_ONLY_DEVICE:-${UBT_SIM_WALKER_S2_DEVICE:-cuda:0}}")
    fi
fi

/isaac-sim/python.sh "$SCRIPT_DIR/walker_s2_sim_runner.py" \
    --task "$UBT_SIM_TASK" \
    --enable_cameras \
    --num_envs "$UBT_SIM_NUM_ENVS" \
    "${EXTRA_ARGS[@]}" \
    "$@"
