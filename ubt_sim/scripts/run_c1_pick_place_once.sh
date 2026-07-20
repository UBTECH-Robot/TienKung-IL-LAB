#!/bin/bash
# Run one online-IK pick-and-place episode against an already running C1 sim.
# This script does not start, stop, or restart the simulator.

set -eo pipefail

source /opt/ros/humble/setup.bash
source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
export ROS_DOMAIN_ID=146

exec /usr/bin/python3 \
    /ubt_sim/teleoperation/control/walker_c1/pick_place_controller.py \
    --episodes 1 \
    --max-grasp-attempts 1 \
    "$@"
