# realsense_wrist_camera

Intel RealSense D405 wrist camera ROS2 publisher — standalone pip package.

## Quick Start

```bash
# 1. Install
bash /ubt_IL/realsense_wrist_camera/scripts/install.sh

# 2. Start (auto-discover cameras + publish to wrist topics)
bash /ubt_IL/realsense_wrist_camera/scripts/start.sh
```

## Setup (one-time per machine)

### Docker container

The container must run with `--privileged` to access USB and video devices.
The project's `run.sh` already includes this.

### Host udev rules

RealSense D405 USB devices default to `root:root` (mode 0644). Add a udev rule
on the **host** so the container user can access them:

```bash
sudo sh -c 'echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"8086\", ATTR{idProduct}==\"0b5b\", MODE=\"0666\"" > /etc/udev/rules.d/99-realsense-d405.rules'
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Rebuild container

After modifying `run.sh`, recreate the container:

```bash
cd /ubt_IL/docker
bash run.sh rm
bash run.sh start
bash run.sh bash
```

## Usage

### One-command start (recommended)

```bash
bash /ubt_IL/realsense_wrist_camera/scripts/start.sh
```

The script sources ROS2 + Walker messages automatically, then:
- Uses `configs/wrist_cameras.json` if it exists
- Falls back to `--discover` (auto-detects cameras, assigns to wrist topics)

### Discover cameras + generate config

```bash
find-realsense-cameras
# → writes configs/wrist_cameras.json with detected serial numbers
```

Output:
```
[0] RealSense D405  →  /sensor/camera/wrist_left/color/raw
    Serial:  260622270436
    USB:     3.2
    FW:      5.13.0.55

[1] RealSense D405  →  /sensor/camera/wrist_right/color/raw
    Serial:  260522275978
    USB:     3.2
    FW:      5.13.0.55

Config written to: .../configs/wrist_cameras.json
```

### Manual start

```bash
# From config file
realsense-wrist-camera --config /path/to/cameras.json &

# Auto-discover
realsense-wrist-camera --discover &

# Single camera test
realsense-wrist-camera --serial <SN> --topic /test/camera &
```

### Auto-start with container

```bash
INSTALL_REALSENSE_WRIST_CAMERA=1 bash run.sh bash
```

Set `REALSENSE_WRIST_CAMERA_CONFIG=/path/to/config.json` to use a specific config,
otherwise `--discover` is used.

## JSON config format

```json
{
  "cameras": [
    {
      "serial": "260622270436",
      "topic": "/sensor/camera/wrist_left/color/raw",
      "msg_type": "shm_msgs/Image1m",
      "frame_id": "wrist_left",
      "width": 640,
      "height": 480,
      "fps": 15
    }
  ]
}
```

| Field | Default | Description |
|-------|---------|-------------|
| `serial` | *(required)* | Camera serial number |
| `topic` | — | ROS2 topic to publish |
| `msg_type` | `shm_msgs/Image1m` | `sensor_msgs/Image` or `shm_msgs/Image1m` etc. |
| `frame_id` | auto | Optical frame ID |
| `width` | `640` | Frame width |
| `height` | `480` | Frame height |
| `fps` | `15` | Frame rate |

## Walker S2 integration

The camera service publishes `shm_msgs/Image1m` on the standard wrist topics
(`/sensor/camera/wrist_left/color/raw`, `/sensor/camera/wrist_right/color/raw`).
Bridge2's `CameraRelay` automatically picks up the frames — no config changes needed.

```bash
# Terminal 1: Start camera service
bash /ubt_IL/realsense_wrist_camera/scripts/start.sh &

# Terminal 2: Run rollout as usual
POLICY_PATH=/ubt_IL/model/<policy>/checkpoints/last/pretrained_model \
bash /ubt_IL/scripts/deploy/walker_s2/rollout.sh
```

### Verify frames

```bash
# ROS2 topic check
ros2 topic echo /sensor/camera/wrist_right/color/raw --once --qos-reliability best_effort

# Preview tool
/usr/bin/python3 /ubt_IL/scripts/deploy/walker_s2/preview_camera.py \
    --topic /sensor/camera/wrist_right/color/raw --once --save-frame /tmp/wrist.jpg
```

## Python API

```python
from realsense_wrist_camera import RealSenseD405Driver, RealSenseWristCameraNode

# Driver only (no ROS2)
driver = RealSenseD405Driver(serial="...", width=640, height=480, fps=15)
driver.start()
img = driver.get_frame()  # numpy BGR array
driver.stop()

# Full ROS2 node
import rclpy
rclpy.init()
node = RealSenseWristCameraNode(cameras=[{
    "serial": "...",
    "topic": "/camera/wrist/color",
}])
node.start()
node.spin_forever()
```

## Requirements

- Python >= 3.10 with ROS2 (`rclpy`, `shm_msgs` for Walker S2)
- `pyrealsense2 >= 2.55` (auto-installed by `install.sh`)
- `numpy < 2`, `opencv-python < 4.10`
- Intel RealSense D405 via USB 3.0
- Docker: `--privileged` + `-v /dev:/dev`

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `No RealSense devices detected` | USB permission or missing udev rule | See [Host udev rules](#host-udev-rules) above |
| `NvRmMemInitNvmap failed` (Jetson) | Jetson memory manager; non-fatal warning | Ignore — does not affect functionality |
| `pip` fails with `jetson.webredirect.org` | Jetson base image pip index unreachable | `install.sh` auto-switches to Tsinghua mirror |
| `ModuleNotFoundError: pyrealsense2` | Not installed | Run `bash scripts/install.sh` |
| `ModuleNotFoundError: rclpy` | ROS2 not sourced | `source /opt/ros/humble/setup.bash` |
| `shm_msgs` import error | Walker messages not built | colcon build in `walker_sdk_ros2/` |
| `Permission denied` opening USB device | Docker missing `--privileged` | Recreate container with updated `run.sh` |
| Channel echo shows wrong type | Run `--msg-type Image1m` explicitly, or wait for DDS cache to clear | |
