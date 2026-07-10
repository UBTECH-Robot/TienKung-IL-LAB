# C1/Astron ubt_sim 改造顺序清单

## 目标

先只改 `/home/changzhang/VLA/C1-IL-LAB/ubt_sim`，跑通 C1/Astron 仿真链路：

1. C1 模型能在 Isaac Sim 中加载
2. C1 关节能被控制
3. C1 仿真中能自动执行任务
4. 能保存 HDF5 数据
5. 后续方便接真机

暂时不做：

- 真机 SDK
- `cc.api`
- `ubt_IL`
- 训练部署
- 真机 rollout

## 1. 整理 C1 资产

源文件：

```text
/home/changzhang/VLA/C1-IL-LAB/c1/walker_astron_v2.urdf
/home/changzhang/VLA/C1-IL-LAB/c1/meshes/
```

目标位置：

```text
/home/changzhang/VLA/C1-IL-LAB/ubt_sim/assets/robots/c1/
```

要做：

- 复制 C1 URDF
- 复制 C1 meshes
- 修正 URDF mesh 路径

重点修正：

```text
./meshes/walker_astron_v1/xxx.STL
```

改成：

```text
./meshes/xxx.STL
```

以及：

```text
package://ubt_right_hand_v3_description/meshes/hand3_v1_new/xxx.STL
```

改成：

```text
./meshes/hand3_v1_new/xxx.STL
```

输出：

```text
ubt_sim/assets/robots/c1/walker_astron_v2_fixed.urdf
```

## 2. 生成 C1 USD

用修好的 URDF 生成 Isaac 可加载的 USD。

输出建议：

```text
ubt_sim/assets/robots/c1/c1_astron.usd
```

验收：

- Isaac 能加载
- 模型完整
- 左右手 mesh 不缺
- 关节能识别

## 3. 新增 C1 device 目录

参考：

```text
ubt_sim/source/ubt_sim/devices/tiangong_pro/
```

新建：

```text
ubt_sim/source/ubt_sim/devices/c1/
```

包含：

```text
config.py
controller.py
action_process.py
__init__.py
```

不要覆盖 `tiangong_pro`。

## 4. 编写 C1 config.py

文件：

```text
ubt_sim/source/ubt_sim/devices/c1/config.py
```

参考：

```text
ubt_sim/source/ubt_sim/devices/tiangong_pro/config.py
```

需要定义：

```python
C1_USD_PATH
C1_LEFT_ARM_JOINTS
C1_RIGHT_ARM_JOINTS
C1_LEFT_HAND_JOINTS
C1_RIGHT_HAND_JOINTS
C1_HEAD_JOINTS
C1_WAIST_JOINTS
C1_LEFT_LEG_JOINTS
C1_RIGHT_LEG_JOINTS
C1_HOME_POSE
C1_JOINT_LIMITS
C1_MIMIC_JOINTS
C1_CFG
```

C1 左臂：

```text
L_shoulder_pitch_joint
L_shoulder_roll_joint
L_shoulder_yaw_joint
L_elbow_pitch_joint
L_elbow_yaw_joint
L_wrist_pitch_joint
L_wrist_roll_joint
```

C1 右臂：

```text
R_shoulder_pitch_joint
R_shoulder_roll_joint
R_shoulder_yaw_joint
R_elbow_pitch_joint
R_elbow_yaw_joint
R_wrist_pitch_joint
R_wrist_roll_joint
```

第一版 action 维度建议：

```text
left_arm[7] + left_hand[6] + right_arm[7] + right_hand[6] = 26
```

腰、头、腿先固定 home pose，不进 action。

## 5. 编写 C1 action_process.py

文件：

```text
ubt_sim/source/ubt_sim/devices/c1/action_process.py
```

作用：

- `ROS/ZMQ command -> C1 Isaac action tensor`
- `Isaac joint state -> status dict`
- 处理手指控制值
- 处理 mimic joints

验收：

- 输入 C1 joint dict，能生成 Isaac action tensor
- 仿真里能读出左右臂和左右手状态

## 6. 编写 C1 controller.py

文件：

```text
ubt_sim/source/ubt_sim/devices/c1/controller.py
```

参考：

```text
ubt_sim/source/ubt_sim/devices/tiangong_pro/controller.py
```

作用：

- 接收 ZMQ 控制命令
- 发布仿真状态
- 发布相机图像
- 处理 reset
- 处理任务物体位置随机化

第一版复用端口：

```text
5555 control command
5556 robot status
5557 raw image
5558 jpeg image
```

验收：

- C1Controller 能启动
- 收到 ZMQ command 后 C1 关节能动
- 5558 有相机 JPEG 流

## 7. 注册 C1 device

修改：

```text
ubt_sim/source/ubt_sim/devices/__init__.py
```

加入：

```python
from .c1.controller import C1Controller
```

保留：

```python
TiangongProController
```

## 8. 新增 C1 task

参考：

```text
ubt_sim/source/ubt_sim/task/tiangong_parlor/
```

新建：

```text
ubt_sim/source/ubt_sim/task/c1_parlor/
```

包含：

```text
__init__.py
c1_parlor_env_cfg.py
```

在 `__init__.py` 注册：

```python
gym.register(
    id="UBTSim-C1-Parlor-v0",
    entry_point="ubt_sim.env:ManagerBasedRLDigitalTwinEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "ubt_sim.task.c1_parlor.c1_parlor_env_cfg:C1ParlorEnvCfg",
    },
)
```

## 9. 编写 C1 环境配置

文件：

```text
ubt_sim/source/ubt_sim/task/c1_parlor/c1_parlor_env_cfg.py
```

参考：

```text
ubt_sim/source/ubt_sim/task/tiangong_parlor/tiangong_parlor_env_cfg.py
```

要改：

- `robot = C1_CFG`
- robot 初始位置
- robot 初始朝向
- camera 挂载 link
- `ActionsCfg` 使用 C1 joint lists
- `ViewerCfg` 视角

相机挂载需要根据 C1 URDF 选择实际 link，例如：

```text
head_pitch_link
head_yaw_link
```

## 10. 新增 C1 YAML 配置

参考：

```text
ubt_sim/config/tiangong_parlor.yaml
```

新建：

```text
ubt_sim/config/c1_parlor.yaml
```

建议内容：

```yaml
scene:
  usd_path: "scenes/parlor/scene_v2.usd"

robot:
  name: "c1"
  usd_path: "robots/c1/c1_astron.usd"

simulation:
  dt: 0.01
  decimation: 1
  render_interval: 3
```

第一版可以复用原 parlor 场景。

## 11. 修改 sim_runner.py 支持 C1

文件：

```text
ubt_sim/scripts/sim_runner.py
```

当前硬编码：

```python
env_cfg.use_teleop_device("tiangong_pro")
teleop_interface = TiangongProController(env)
```

改成支持参数：

```bash
--robot c1
```

逻辑：

```python
if args_cli.robot == "c1":
    env_cfg.use_teleop_device("c1")
    teleop_interface = C1Controller(env)
else:
    env_cfg.use_teleop_device("tiangong_pro")
    teleop_interface = TiangongProController(env)
```

启动目标：

```bash
bash scripts/start_sim.sh --task UBTSim-C1-Parlor-v0 --robot c1
```

## 12. ROS topic 第一版先复用现有接口

为了改动少，先继续用当前内部仿真 topic：

```text
/arm/cmd_pos
/arm/status
/inspire_hand/ctrl/left_hand
/inspire_hand/ctrl/right_hand
/inspire_hand/state/left_hand
/inspire_hand/state/right_hand
/ob_camera_head/color/image_raw
/ob_camera_head/depth/image_raw
/sim/cmd_reset
/scene/apple/offset
/sim/task_completed
```

暂时不切换到 C1 真机 topic：

```text
/mc/sdk/robot_state
/mc/sdk/robot_command
/mc/left_hand/command
/mc/right_hand/command
```

后续接真机时再做 C1 真机 bridge。

## 13. 新增 C1 control constants

参考：

```text
ubt_sim/teleoperation/control/constants.py
```

建议新建：

```text
ubt_sim/teleoperation/control/c1_constants.py
```

定义：

```python
C1_LEFT_ARM_JOINTS
C1_RIGHT_ARM_JOINTS
C1_LEFT_HAND_JOINTS
C1_RIGHT_HAND_JOINTS
ARM_HOME_PICK_PLACE
HAND_OPEN
HAND_CLOSE
CONTROL_LOOP_HZ
```

## 14. 新增 C1 采集控制器

参考：

```text
ubt_sim/teleoperation/control/robot_controller.py
ubt_sim/teleoperation/control/pick_place_controller.py
ubt_sim/teleoperation/control/pick_place_save_data.py
```

新建：

```text
ubt_sim/teleoperation/control/c1_robot_controller.py
ubt_sim/teleoperation/control/c1_pick_place_controller.py
ubt_sim/teleoperation/control/c1_pick_place_save_data.py
```

第一版只做右臂抓取即可。

需要改：

- C1 home pose
- C1 右臂 IK
- C1 手指 open/close
- C1 抓取路径
- C1 HDF5 action/state 顺序

HDF5 字段建议保持原结构：

```text
puppet/arm_left_position_align/data
puppet/arm_right_position_align/data
puppet/end_effector_left_position_align/data
puppet/end_effector_right_position_align/data

action/arm_left_position_align/data
action/arm_right_position_align/data
action/end_effector_left_position_align/data
action/end_effector_right_position_align/data

camera_observations/color_images/camera_head
camera_observations/depth_images/camera_head
observations/timestamp
```

这样后面 `ubt_IL` 转换脚本改动最小。

## 15. 生成 C1 右臂 IK URDF

当前天工文件：

```text
ubt_sim/teleoperation/control/right_arm.urdf
```

C1 新建：

```text
ubt_sim/teleoperation/control/c1_right_arm.urdf
```

从 C1 URDF 抽出：

- torso / waist base
- `R_shoulder_pitch_joint`
- `R_shoulder_roll_joint`
- `R_shoulder_yaw_joint`
- `R_elbow_pitch_joint`
- `R_elbow_yaw_joint`
- `R_wrist_pitch_joint`
- `R_wrist_roll_joint`
- 末端 link

验收：

- `ikpy.Chain.from_urdf_file()` 能加载
- 能找到 7 个右臂 joint
- 小幅 offset IK 能求解

## 16. 测试顺序

### 16.1 静态加载测试

```bash
cd /home/changzhang/VLA/C1-IL-LAB/ubt_sim/docker/isaac_sim
bash run.sh bash
bash scripts/start_sim.sh --task UBTSim-C1-Parlor-v0 --robot c1
```

检查：

- 模型完整
- mesh 不缺
- joint 不缺
- 相机有画面

### 16.2 关节控制测试

写最小测试脚本发送右臂小动作。

检查：

- ZMQ 收到 command
- C1 右臂动作正确
- 状态能回传

### 16.3 相机测试

检查 JPEG 流：

```bash
/usr/bin/python3 /ubt_sim/teleoperation/tools/test_zmq_image.py
```

### 16.4 自动采集测试

```bash
/usr/bin/python3 /ubt_sim/teleoperation/control/c1_pick_place_save_data.py
```

检查生成：

```text
ubt_sim/dataset/<timestamp>/trajectory.hdf5
```

### 16.5 HDF5 自检

检查：

- 各字段帧数一致
- action shape = 26
- observation state shape = 26
- 图像可 decode
- timestamp 正常

## 17. 第一阶段不要做

先不要改：

```text
ubt_IL/
ubt_IL/tienkung/
ubt_IL/scripts/deploy/
ubt_IL/docker/
C1 SDK
cc.api
真机 ROS topic
真机控制权限
```

## 18. 推荐执行顺序总结

1. 复制 C1 URDF 和 meshes 到 `ubt_sim/assets/robots/c1/`
2. 修 URDF mesh 路径
3. 生成 `c1_astron.usd`
4. 新建 `devices/c1/config.py`
5. 新建 `devices/c1/action_process.py`
6. 新建 `devices/c1/controller.py`
7. 注册 `C1Controller`
8. 新建 `task/c1_parlor/`
9. 新建 `config/c1_parlor.yaml`
10. 改 `sim_runner.py` 支持 `--robot c1`
11. 启动 Isaac 验证 C1 静态加载
12. 新建 `c1_constants.py`
13. 生成 `c1_right_arm.urdf`
14. 新建 C1 robot/pick-place/save-data 控制器
15. 测试关节控制
16. 测试相机链路
17. 测试自动采集
18. 检查 HDF5 数据

