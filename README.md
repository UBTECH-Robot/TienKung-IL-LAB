# TienKung-IL-LAB

优必选天工机器人模仿学习工具链（开发中）

## 项目简介

本项目基于 [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim) 与 [LeRobot](https://github.com/huggingface/lerobot) 框架开发，为天工（TienKung）机器人提供完整的模仿学习工具链，涵盖以下核心能力：

| 能力 | 说明 | 状态 |
|------|------|------|
| 🌐 ROS 仿真环境 | 高逼真度 Isaac Sim 仿真，支持遥操作与数据采集 | ✅ 已完成 |
| 🎮 遥控操作 | 键盘/空间鼠标等设备遥操作，仿真与真机统一接口 | 🚧 开发中 |
| 📦 数据采集与转换 | HDF5 / LeRobot 格式数据采集，格式转换与清洗 | 🚧 开发中 |
| 🧠 模型训练 | 基于 LeRobot 的模仿学习策略训练 | 🚧 开发中 |
| 🤖 真机部署 | 模型推理与真机控制部署 | 🚧 开发中 |

仿真环境已开发完成，具体介绍与使用说明请参考 [ubt_sim/README.md](ubt_sim/README.md)。

## 仿真模块
### 快速开始

```bash
# 1. 构建并启动容器
cd docker/isaac_sim
bash run.sh build && bash run.sh start && bash run.sh init && bash run.sh check
# 若真机模式启动容器：ROS_DOMAIN_ID=0 bash run.sh start

# 2. 启动仿真（自动启动 ROS2-ZMQ 桥接）
bash run.sh bash
bash scripts/start_sim.sh 
# 真机模式启动仿真+桥接：ROS_DOMAIN_ID=0 bash scripts/start_sim.sh
# 按R机器人可复位

# 3. 数据采集（同一容器内，用系统 Python 3.10）
/usr/bin/python3 /ubt_sim/teleoperation/control/reset.py  # 机器人回零
/usr/bin/python3 /ubt_sim/teleoperation/control/pick_place_save_data.py  # 单次
bash /ubt_sim/teleoperation/control/save_data.sh                         # 批量
```
注意：使用**echo $ROS_DOMAIN_ID**和**ros2 topic list**检查当前模式仿真/真机，以及桥接是否启动。

### 模式说明

通过 `ROS_DOMAIN_ID` 区分仿真与真机：

| 模式 | ROS_DOMAIN_ID | 说明 |
|------|--------------|------|
| 仿真 | 146 | ZMQ 桥接连接 Isaac Sim |
| 真机 | 0 | ZMQ 桥接连接真实机器人 |
