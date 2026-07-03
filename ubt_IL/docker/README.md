# Docker usage

This directory supports both the original x86_64 LeRobot GPU image and a Jetson/aarch64 image path.

## Build and start

```bash
cd ubt_IL/docker
bash run.sh build
bash run.sh start
bash run.sh check
```

`env.sh` selects defaults from `uname -m`:

| Host arch | Dockerfile | Default image | Default GPU args |
| --- | --- | --- | --- |
| `x86_64` / `amd64` | `Dockerfile` | `lerobot-tienkung:humble` | `--gpus all` |
| `aarch64` / `arm64` | `Dockerfile.arm64` | `lerobot-tienkung:humble-arm64` | `--runtime nvidia` |

## Overrides

Use environment variables when the automatic defaults do not match the host setup:

```bash
# Use the default ARM64 mirror image directly.
bash run.sh build

# Or pre-pull and tag the image locally.
sudo docker pull swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/dustynv/l4t-pytorch:r36.4.0-linuxarm64
sudo docker tag \
  swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/dustynv/l4t-pytorch:r36.4.0-linuxarm64 \
  docker.io/dustynv/l4t-pytorch:r36.4.0
BASE_IMAGE=docker.io/dustynv/l4t-pytorch:r36.4.0 bash run.sh build

# Use a different base image.
BASE_IMAGE=nvcr.io/nvidia/l4t-pytorch:<tag> bash run.sh build

# Use a different Dockerfile or output image name.
DOCKERFILE=./Dockerfile.arm64 IMAGE=my-lerobot:arm64 bash run.sh build

# Jetson systems with newer NVIDIA Container Toolkit may prefer --gpus all.
DOCKER_GPU_ARGS="--gpus all" bash run.sh start

# Disable explicit GPU args for debugging.
DOCKER_GPU_ARGS="" bash run.sh start
```

## ARM64 limitation

`ros2_msgs/ros-humble-bodyctrl-msgs_0.0.1-1_amd64.deb` is not installed by `Dockerfile.arm64`. It is an amd64 package and contains x86_64 ROS type-support libraries.

Walker S2 support does not use that deb: Walker ROS2 message packages are built from `/ubt_IL/walker/walker_sdk_ros2` by `entrypoint.sh` when the container starts.

If TianKung must run on Jetson/aarch64, add an arm64 build of `bodyctrl_msgs` or add the source package and build it in the ARM64 Dockerfile.
