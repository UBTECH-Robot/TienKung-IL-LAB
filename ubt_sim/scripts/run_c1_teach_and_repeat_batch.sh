#!/bin/bash
# Walker C1 teach-and-repeat batch runner: produces N SUCCESSFUL episodes at
# the fixed taught position, retrying with a fresh sim stack on failure.
#
# Why retry-with-fresh-stack (not retry-in-process): empirically, a single
# "cold" attempt (fresh Isaac Sim process, episode 1) succeeds ~65% of the
# time — NOT deterministically (measured across 6 independent fresh-stack
# trials: 4 success, 2 failure). Once ANY episode fails on a running
# process, every subsequent episode on that SAME process fails
# near-identically thereafter (observed across every multi-episode trial
# today) — in-process retries do not recover. So: on failure, restart the
# whole stack and try again, rather than retrying within the same process.
# With p~0.65 per fresh attempt, expected tries per success ~1.5,
# P(success within 3 tries) ~96%.
#
# Waist/head/leg joint drift was checked and ruled out as the degradation
# cause; the underlying physics-engine mechanism for the post-failure
# same-process degradation is still unresolved (see C1_HANDOFF.md).
#
# Usage:
#   docker exec walker-c1-ubt-sim bash /ubt_sim/scripts/run_c1_teach_and_repeat_batch.sh <N> [max_attempts_per_episode]
#     N: number of SUCCESSFUL episodes desired (default 3)
#     max_attempts_per_episode: cap on fresh-stack retries per desired success (default 5)

set -uo pipefail
N="${1:-3}"
MAX_ATTEMPTS="${2:-5}"
LOG_DIR="/tmp/c1_batch_$(date +%s)"
mkdir -p "$LOG_DIR"

source /opt/ros/humble/setup.bash
source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
export ROS_DOMAIN_ID=146

TOTAL_ATTEMPTS=0
SUCCESS_COUNT=0
EPISODE_ATTEMPT_COUNTS=()

restart_stack() {
    pkill -9 -f 'sim_runner|walker_c1_ros2|zmq_image' 2>/dev/null
    sleep 3
    cd /ubt_sim
    UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0 ROS_DOMAIN_ID=146 \
        nohup bash scripts/start_sim.sh --headless --device cpu --step_hz 30 \
        > "$LOG_DIR/stack_${TOTAL_ATTEMPTS}.log" 2>&1 &
    for attempt in $(seq 1 45); do
        if timeout 8 ros2 topic echo /sim/object_state --once 2>/dev/null | grep -q sim_step; then
            return 0
        fi
        sleep 10
    done
    return 1
}

for ep in $(seq 1 "$N"); do
    EP_SUCCESS=0
    for try in $(seq 1 "$MAX_ATTEMPTS"); do
        TOTAL_ATTEMPTS=$((TOTAL_ATTEMPTS + 1))
        echo "=================================================="
        echo "[BATCH] Target episode $ep/$N, attempt $try/$MAX_ATTEMPTS (global attempt $TOTAL_ATTEMPTS): restarting stack..."
        echo "=================================================="

        if ! restart_stack; then
            echo "[BATCH] Stack failed to boot, retrying..."
            continue
        fi

        echo "[BATCH] Stack ready, running replay..."
        OUT=$(timeout 400 /usr/bin/python3 \
            /ubt_sim/teleoperation/control/walker_c1/pick_place_replay.py --episodes 1 \
            2>&1 | tee "$LOG_DIR/episode_${ep}_try${try}.log")

        if echo "$OUT" | grep -q "SUCCESS"; then
            echo "[BATCH] Target episode $ep: SUCCESS on attempt $try"
            EP_SUCCESS=1
            EPISODE_ATTEMPT_COUNTS+=("$try")
            SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
            break
        else
            echo "[BATCH] Target episode $ep: attempt $try FAILED, will restart and retry"
        fi
    done
    if [ "$EP_SUCCESS" -ne 1 ]; then
        echo "[BATCH] Target episode $ep: FAILED after $MAX_ATTEMPTS attempts"
        EPISODE_ATTEMPT_COUNTS+=("EXHAUSTED")
    fi
done

pkill -9 -f 'sim_runner|walker_c1_ros2|zmq_image' 2>/dev/null

echo "=================================================="
echo "[BATCH] Summary: $SUCCESS_COUNT/$N target episodes achieved, $TOTAL_ATTEMPTS total sim attempts"
for i in "${!EPISODE_ATTEMPT_COUNTS[@]}"; do
    echo "  target episode $((i+1)): attempts-to-success = ${EPISODE_ATTEMPT_COUNTS[$i]}"
done
echo "[BATCH] Logs: $LOG_DIR"
