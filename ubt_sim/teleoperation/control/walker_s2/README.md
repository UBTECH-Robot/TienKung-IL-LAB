# Walker S2 control

Walker S2 ROS2 control helpers copied and adapted from `/home/qingxiangliu/work/ubt_IL/walker/walker_sdk_ros2/robot_control`.

Run these scripts with system Python and the Walker SDK ROS2 messages sourced:

```bash
source /opt/ros/humble/setup.bash
source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
```

Typical simulation flow:

```bash
# Terminal 1: start Walker S2 sim and bridge
cd /home/qingxiangliu/work/UBTECH-IL-LAB/ubt_sim
UBT_SIM_TASK=UBTSim-WalkerS2-PartSorting-v0 bash scripts/start_sim.sh

# Terminal 2: query state / run small tests
cd /home/qingxiangliu/work/UBTECH-IL-LAB/ubt_sim/teleoperation/control/walker_s2
/usr/bin/python3 walker_s2_controller.py --print-state
/usr/bin/python3 walker_s2_joint_test.py --print
/usr/bin/python3 walker_s2_camera.py --msg-type sensor_msgs/Image
```

Files:

- `walker_s2_controller.py` — main `WalkerS2Controller` with body trajectory, hand command, reset, and image cache APIs.
- `walker_s2_joint_test.py` — joint and hand debugging CLI.
- `walker_s2_camera.py` — camera topic helper for `sensor_msgs/Image` and optional `shm_msgs` images.
- `walker_s2_reset.py` — conservative home/open-hand smoke script.
- `walker_s2_constants.py` — constants mirrored from the SDK and the sim bridge joint order.
