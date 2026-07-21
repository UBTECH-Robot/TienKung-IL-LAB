#!/bin/bash
# Run and record one online-IK pick-and-place episode against an existing C1 sim.
# Successful trajectories are saved under /ubt_sim/dataset/walker_c1_ros.
# This script does not start, stop, or restart the simulator. Pass --no-record
# after the script name when only task execution is needed.

set -eo pipefail

source /opt/ros/humble/setup.bash
source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
export ROS_DOMAIN_ID=146

exec /usr/bin/python3 \
    /ubt_sim/teleoperation/control/walker_c1/pick_place_controller.py \
    --episodes 1 \
    --max-grasp-attempts 1 \
    --record \
    "$@"
