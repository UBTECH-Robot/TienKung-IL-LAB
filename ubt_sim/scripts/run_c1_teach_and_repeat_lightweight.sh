#!/bin/bash
# Walker C1 teach-and-repeat batch runner v2 — S2-INSPIRED PATTERN.
#
# Walker S2's own data collection script (pick_part_save_data.py) runs
# EXACTLY ONE task per Python process invocation and exits; the Isaac Sim
# stack keeps running continuously across many such invocations (S2 has been
# doing this in production). We had been restarting the FULL ~5min Isaac
# boot per episode, assuming the physics engine itself degraded. This script
# tests the much cheaper hypothesis first: restart only the lightweight ROS
# control script (a few seconds) per episode, leaving Isaac Sim + bridge
# running continuously, matching S2's proven pattern.
#
# Usage:
#   docker exec walker-c1-ubt-sim bash /ubt_sim/scripts/run_c1_teach_and_repeat_lightweight.sh <N>
#     (assumes the sim stack is ALREADY running; runs N fresh-PROCESS
#      episodes back to back, no stack restarts)

set +u
source /opt/ros/humble/setup.bash
source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
set -u
export ROS_DOMAIN_ID=146

N="${1:-5}"
LOG_DIR="/tmp/c1_lw_$(date +%s)"
mkdir -p "$LOG_DIR"

SUCCESS_COUNT=0
for i in $(seq 1 "$N"); do
    echo "=================================================="
    echo "[LW] Episode $i/$N (fresh CONTROL PROCESS, sim stack untouched)"
    echo "=================================================="
    OUT=$(timeout 300 /usr/bin/python3 \
        /ubt_sim/teleoperation/control/walker_c1/pick_place_replay.py --episodes 1 \
        2>&1 | tee "$LOG_DIR/episode_$i.log")
    if echo "$OUT" | grep -q "SUCCESS"; then
        echo "[LW] Episode $i: SUCCESS"
        SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
    else
        echo "[LW] Episode $i: FAILURE"
    fi
done

echo "=================================================="
echo "[LW] Summary: $SUCCESS_COUNT/$N (fresh-process-per-episode, no stack restart)"
echo "[LW] Logs: $LOG_DIR"
