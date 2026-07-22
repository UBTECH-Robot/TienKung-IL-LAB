#!/bin/bash
# Start the Walker C1 pick-and-place simulator and ROS bridge in the foreground.
# GUI mode is the default; pass --headless explicitly when no window is needed.

set -euo pipefail

export UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0
export ROS_DOMAIN_ID=146
C1_STEP_HZ="${UBT_SIM_C1_STEP_HZ:-100}"

cd /ubt_sim
exec bash scripts/start_sim.sh --device cpu --step_hz "$C1_STEP_HZ" "$@"
