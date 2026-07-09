# Walker C1 / Astron Handoff Notes

本文档给后续接手的 AI/工程师使用。当前有效工作目录已经切换到新 clone 的 `walker_c1` 分支仓库。

## Current Workspace

```text
/home/changzhang/VLA/walker_c1
```

Git 分支：

```text
walker_c1
```

当前只改 `ubt_sim` 相关内容。`ubt_IL` 暂时不动。

旧探索目录仍在：

```text
/home/changzhang/VLA/C1-IL-LAB
```

旧目录只作为参考，不再作为主工作区。

## Naming

- `C1 = Astron`。
- 新仓库里统一使用 `walker_c1` 作为目录/代码前缀，和现有 `walker_s2` 对齐。
- 原始 URDF 文件名暂时保留 `walker_astron_*`，因为这是来源资产名。

## User Preference

- 用户希望一步一步慢慢做。
- 每一步执行前需要说明要做什么、为什么做。
- 不要直接大范围改主链路。
- 当前优先级：先跑通 Walker C1/Astron 在 `ubt_sim` 仿真中的基础加载链路，再接 task/controller/ROS。
- 当前只做仿真，不做真机。

## Current Git Status Summary

截至本接力文件更新时，新仓库新增未跟踪内容主要是：

```text
ubt_sim/assets/robots/walker_c1/
ubt_sim/scripts/export_walker_c1_usd.py
ubt_sim/scripts/import_walker_c1_urdf_test.py
ubt_sim/scripts/test_walker_c1_cfg_spawn.py
ubt_sim/source/ubt_sim/devices/walker_c1/
ubt_sim/source/ubt_sim/task/walker_c1_parlor/
ubt_sim/config/walker_c1/
```

注意：运行测试后可能生成：

```text
ubt_sim/source/ubt_sim/devices/walker_c1/__pycache__/
ubt_sim/source/ubt_sim/task/walker_c1_parlor/__pycache__/
```

这是 Python 运行缓存，不是源码改动。

## Docker / Isaac Container State

为了不影响旧的 `ubt-sim` 容器，已新建专用容器：

```text
walker-c1-ubt-sim
```

镜像：

```text
ubt-sim-isaac:latest
```

挂载：

```text
/home/changzhang/VLA/walker_c1/ubt_sim -> /ubt_sim
```

旧容器 `ubt-sim` 仍然挂载旧项目：

```text
/home/changzhang/VLA/TienKung-IL-LAB/ubt_sim -> /ubt_sim
```

所以后续 Walker C1 测试应使用：

```bash
docker exec walker-c1-ubt-sim ...
```

不要误用旧的 `ubt-sim` 容器。

## Assets Done

Walker C1/Astron 资产已经放到：

```text
ubt_sim/assets/robots/walker_c1/
```

关键文件：

```text
ubt_sim/assets/robots/walker_c1/walker_astron_v2.urdf
ubt_sim/assets/robots/walker_c1/walker_astron_v2_fixed.urdf
ubt_sim/assets/robots/walker_c1/walker_astron_v2_hand_v3.urdf
ubt_sim/assets/robots/walker_c1/walker_astron_v2_hand_v3_no_sixforce_mesh.urdf
ubt_sim/assets/robots/walker_c1/walker_c1.usd
```

手部 mesh/URDF：

```text
ubt_sim/assets/robots/walker_c1/meshes/hand_v3/left/
ubt_sim/assets/robots/walker_c1/meshes/hand_v3/right/
ubt_sim/assets/robots/walker_c1/urdf/hand_v3/left/
ubt_sim/assets/robots/walker_c1/urdf/hand_v3/right/
```

## URDF Notes

原版：

```text
walker_astron_v2_hand_v3.urdf
```

当前不能直接可靠导入 Isaac，因为缺两个 mesh：

```text
L_sixforce_link.STL
R_sixforce_link.STL
```

推荐当前使用：

```text
walker_astron_v2_hand_v3_no_sixforce_mesh.urdf
```

这个版本只移除了 `L_sixforce_link` / `R_sixforce_link` 的 visual/collision mesh 引用，保留 link、inertial 和 fixed joint 结构。

## Completed Tests

### 1. URDF Import Smoke Test

新增脚本：

```text
ubt_sim/scripts/import_walker_c1_urdf_test.py
```

运行命令：

```bash
docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u /ubt_sim/scripts/import_walker_c1_urdf_test.py --headless
```

结果：

```text
[OK] Articulation is valid: /walker_astron_v1/root_joint
[INFO] dof_count=53
```

### 2. URDF -> USD Export

新增脚本：

```text
ubt_sim/scripts/export_walker_c1_usd.py
```

运行命令：

```bash
docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u /ubt_sim/scripts/export_walker_c1_usd.py --headless
```

输出：

```text
ubt_sim/assets/robots/walker_c1/walker_c1.usd
```

结果：

```text
[OK] Articulation is valid: /walker_astron_v1/root_joint
[INFO] dof_count=53
[OK] Exported USD: /ubt_sim/assets/robots/walker_c1/walker_c1.usd
```

`walker_c1.usd` 是 USD crate 文件，约 16 MB。

### 3. devices/walker_c1/config.py

新增：

```text
ubt_sim/source/ubt_sim/devices/walker_c1/config.py
ubt_sim/source/ubt_sim/devices/walker_c1/__init__.py
```

已定义：

```text
WALKER_C1_USD_PATH
WALKER_C1_URDF_PATH
WALKER_C1_LEFT_ARM_JOINTS
WALKER_C1_RIGHT_ARM_JOINTS
WALKER_C1_LEFT_HAND_JOINTS
WALKER_C1_RIGHT_HAND_JOINTS
WALKER_C1_HEAD_JOINTS
WALKER_C1_WAIST_JOINTS
WALKER_C1_LEFT_LEG_JOINTS
WALKER_C1_RIGHT_LEG_JOINTS
WALKER_C1_UPPER_BODY_JOINTS
WALKER_C1_HOME_POSE
WALKER_C1_CFG
```

关节校验结果：

```text
movable_joint_count=53
home_pose_count=53
home_pose_extra=[]
home_pose_missing_movable=[]
```

说明：`WALKER_C1_HOME_POSE` 正好覆盖 URDF 里的 53 个可动关节。

很多 home pose 值为 `0.0` 是第一版中立姿态，不代表关节不能动。膝盖因为 URDF 下限是 `0.08`，所以设置为：

```text
L_knee_pitch_joint = 0.08
R_knee_pitch_joint = 0.08
```

### 4. WALKER_C1_CFG Spawn Test

新增脚本：

```text
ubt_sim/scripts/test_walker_c1_cfg_spawn.py
```

运行命令：

```bash
docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u /ubt_sim/scripts/test_walker_c1_cfg_spawn.py --headless --device cpu
```

结果：

```text
[OK] robot_spawned=True
[INFO] joint_count=53
[INFO] home_pose_missing_from_spawn=[]
[INFO] spawn_extra_joints=[]
[OK] spawn_smoke_test_passed
```

注意：这个 smoke test 脚本在成功后使用 `os._exit(0)` 直接退出，原因是 Isaac/AppLauncher 在这个最小脚本里关闭阶段会卡住。这个处理只影响测试脚本，不影响正式仿真入口。

### 5. Walker C1 Parlor Load-Only Task

新增：

```text
ubt_sim/source/ubt_sim/task/walker_c1_parlor/__init__.py
ubt_sim/source/ubt_sim/task/walker_c1_parlor/walker_c1_parlor_env_cfg.py
ubt_sim/config/walker_c1/parlor.yaml
```

已注册 task：

```text
UBTSim-WalkerC1-Parlor-v0
```

同时小范围更新：

```text
ubt_sim/source/ubt_sim/__init__.py
ubt_sim/scripts/start_sim.sh
ubt_sim/scripts/sim_runner.py
```

更新目的：

```text
注册 Walker C1 task
start_sim.sh 识别 WalkerC1，不再误走 tienkung_pro
Walker C1 暂时跳过 ROS2-ZMQ bridge
start_sim.sh 自动加入 /ubt_sim/source 到 PYTHONPATH
start_sim.sh 使用 /isaac-sim/python.sh -u 输出实时日志
sim_runner.py 对 Walker C1 非 load-only 明确报错，避免误接旧 controller
```

验证命令：

```bash
docker start walker-c1-ubt-sim
docker exec walker-c1-ubt-sim bash -lc "cd /ubt_sim && UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0 UBT_SIM_LOAD_ONLY=1 timeout 120s bash scripts/start_sim.sh --headless"
```

结果：

```text
[INFO]: Parsing configuration from: ubt_sim.task.walker_c1_parlor.walker_c1_parlor_env_cfg:WalkerC1ParlorEnvCfg
[INFO] Action Manager: <ActionManager> contains 8 active terms.
Active Action Terms (shape: 53)
left_arm_action=7
right_arm_action=7
left_hand_action=11
right_hand_action=11
head_action=2
waist_action=3
left_leg_action=6
right_leg_action=6
[INFO] Observation Manager: policy joint_pos shape=(53,)
[INFO]: Completed setting up the environment...
[INFO] Walker C1 load-only mode: ROS control and action preprocessing are disabled.
[INFO] Load-only app update enabled: physics/action/observation stepping is disabled.
```

说明：

```text
Walker C1 USD 已能在 parlor 场景中通过 Isaac Lab task 正常加载
相机配置未报错
action manager 已识别全部 53 个可动关节
```

### 6. Tiangong Parlor Local Scene Assets

用户打开 GUI 后只看到机器人和空白背景。原因是新 clone 里的：

```text
ubt_sim/assets/scenes/parlor/scene_v2.usd
```

以及 parlor 下很多 USD/PNG 都是 Git LFS pointer 文件，当前机器没有 `git lfs`，所以实际客厅/桌子/水果资产没有拉下来。

为了不把 tracked LFS pointer 文件直接替换成大二进制，已从旧探索目录复制 Tienkung 可见的 parlor 真实资产到本地忽略目录：

```text
/home/changzhang/VLA/C1-IL-LAB/ubt_sim/assets/scenes/parlor/
  -> ubt_sim/assets/local_scenes/tiangong_parlor/
```

并在 `.gitignore` 增加：

```text
ubt_sim/assets/local_scenes/
```

`ubt_sim/config/walker_c1/parlor.yaml` 当前场景路径已改为：

```text
scene:
  usd_path: "local_scenes/tiangong_parlor/scene_v2.usd"
```

重新验证：

```bash
docker exec walker-c1-ubt-sim bash -lc "cd /ubt_sim && UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0 UBT_SIM_LOAD_ONLY=1 timeout 120s bash scripts/start_sim.sh --headless"
```

结果：

```text
环境 setup 完成
Action Manager shape=53
进入 Walker C1 load-only mode
```

注意：日志里仍有少量 MDL/默认贴图 warning，例如 `Clear_Glass.mdl` 和 fruit 默认贴图路径；这不阻止 scene setup。如果 GUI 中水果材质显示异常，后续再单独整理这些材质路径。

## ROS / SDK Notes

SDK 文档显示 Astron/C1 低层 ROS2 接口和当前 `walker_s2` 使用的 SDK message 基本一致：

身体状态：

```text
/mc/sdk/robot_state
mc_state_msgs/RobotState
```

身体控制：

```text
/mc/sdk/robot_command
mc_task_msgs/RobotCommand
JointCmd.MODE_POSITION = 2
```

灵巧手状态：

```text
/mc/left_hand/joint_states
/mc/right_hand/joint_states
sensor_msgs/JointState
```

灵巧手控制：

```text
/mc/left_hand/command
/mc/right_hand/command
mc_task_msgs/JointCommand
```

新仓库已有 vendored ROS message 源码：

```text
ubt_sim/teleoperation/msgs/walker_sdk_ros2_msgs_src/src/mc_task_msgs
ubt_sim/teleoperation/msgs/walker_sdk_ros2_msgs_src/src/mc_state_msgs
ubt_sim/teleoperation/msgs/walker_sdk_ros2_msgs_src/src/shm_msgs
ubt_sim/teleoperation/msgs/walker_sdk_ros2_msgs_src/src/ecat_task_msgs
```

后续重点不是重造 message，而是确认 C1/Astron 的真实 joint order、手部 JointCommand 顺序、相机 topic/type。

## Current Architecture Decision

现在已经从 URDF 路线推进到 USD/task 路线：

```text
URDF import OK -> USD export OK -> WALKER_C1_CFG spawn OK -> Walker C1 parlor load-only task OK
```

后续应按 `walker_s2` 结构继续新增同级目录，而不是覆盖已有文件：

```text
ubt_sim/source/ubt_sim/task/walker_c1_parlor/
ubt_sim/config/walker_c1/parlor.yaml
ubt_sim/teleoperation/bridges/walker_c1/
ubt_sim/teleoperation/control/walker_c1/
```

## Recommended Next Step

下一步建议继续保持小步：

```text
1. 确认 C1/Astron 真实 SDK joint order，尤其身体 RobotCommand 顺序和左右手 JointCommand 顺序。
2. 再按 walker_s2 结构新增 walker_c1 controller/action_process 的最小版本。
3. controller 通过后再新增 teleoperation/bridges/walker_c1/，不要先写 ROS bridge。
```

## Important Do Not Do Yet

暂时不要：

```text
不要直接改旧 ubt-sim 容器
不要覆盖 walker_s2 文件
不要先写 ROS bridge
不要先写 controller
不要动 ubt_IL
不要把原版 walker_astron_v2_hand_v3.urdf 当作默认加载文件
```

当前最稳顺序：

```text
asset -> URDF import -> USD export -> devices config -> CFG spawn test -> task -> controller -> ROS bridge
```

目前已经完成到：

```text
Walker C1 parlor load-only task
```
