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

## Commit / Push Status

本地分支：

```text
walker_c1
```

当前本地已完成 commit：

```text
41e2603 Add Walker C1 parlor load-only task
b6e8a57 Add Walker C1 assets and Isaac spawn config
```

当前工作区在 commit 后是 clean 的；如果后续只看到 `C1_HANDOFF.md` 变更，是因为本段接力信息是在 commit 后补写的。

用户尝试首次 push：

```bash
git push -u origin walker_c1
```

失败原因：

```text
remote: Invalid username or token. Password authentication is not supported for Git operations.
fatal: Authentication failed for 'https://github.com/UBTECH-Robot/TienKung-IL-LAB.git/'
```

说明：这不是代码或 commit 问题，是 GitHub HTTPS 不再支持账号密码 push。后续需要任选一种认证方式：

```text
1. 使用 GitHub Personal Access Token 作为 HTTPS password。
2. 改用 SSH remote，例如 git@github.com:UBTECH-Robot/TienKung-IL-LAB.git。
3. 使用 gh auth login 完成 GitHub CLI 认证。
```

认证完成后再执行：

```bash
git push -u origin walker_c1
```

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

## 2026-07-10 Update: C1 Controller / ZMQ / Hand Debug

本节是最新状态。前面的部分记录了早期 load-only 阶段；当前已经继续推进到 C1 controller、ZMQ 仿真控制和手部调试。

### Files Changed In This Round

新增：

```text
ubt_sim/source/ubt_sim/devices/walker_c1/action_process.py
ubt_sim/source/ubt_sim/devices/walker_c1/controller.py
```

修改：

```text
ubt_sim/source/ubt_sim/devices/walker_c1/__init__.py
ubt_sim/source/ubt_sim/devices/walker_c1/config.py
ubt_sim/scripts/sim_runner.py
ubt_sim/docker/env.sh
```

文档/参考文件：

```text
C1_joint_map.md
C1_ubt_sim改造顺序清单.md
【CC-API】Astron优必选SDK二次开发文档【对内】.docx
```

### Docker State

`ubt_sim/docker/env.sh` 已把容器名改为：

```text
CONTAINER_NAME="walker-c1-ubt-sim"
```

原因：旧的 `ubt-sim` 容器挂的是旧项目路径，C1 应该使用新容器 `walker-c1-ubt-sim`，避免 `bash run.sh init` / `check` 误操作旧容器。

用户已在新容器里跑过：

```bash
bash run.sh check
```

关键结果：

```text
[OK] Project mounted: /ubt_sim
[OK] Assets directory: /ubt_sim/assets
[OK] Isaac Sim Python: Python 3.11.13
[OK] ubt_sim package: installed
[OK] pyzmq: installed
[OK] numpy: 1.26.0 (< 2)
[OK] ROS2 Humble: installed
[OK] bodyctrl_msgs: installed
[FAIL] Walker SDK ROS2 messages: NOT built (run: bash run.sh init)
```

说明：当前 C1 仿真使用 `UBT_SIM_NO_BRIDGE=1`，暂时不走 ROS bridge，所以 `Walker SDK ROS2 messages` 未 build 不是当前 blocker。后续接真机/ROS bridge 时再处理。

### Controller / ZMQ Status

当前 C1 非 load-only 仿真已能启动 `WalkerC1Controller`：

```text
Command Sub: tcp://127.0.0.1:5655
Status Pub:  tcp://*:5656
Image Pub:   tcp://*:5657
JPEG Pub:    tcp://*:5658
```

推荐调试命令：

```bash
docker exec walker-c1-ubt-sim bash -lc "cd /ubt_sim && UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0 UBT_SIM_NO_BRIDGE=1 timeout 360s bash scripts/start_sim.sh --headless --device cpu --step_hz 30"
```

注意：GPU PhysX 曾出现 CUDA/PhysX allocator 相关问题；当前 debug 推荐用 `--device cpu`。

### Action Mapping Status

C1 当前外部第一版控制输入定义为 26 维：

```text
left_arm[7] + left_hand_sdk[6] + right_arm[7] + right_hand_sdk[6]
```

手部 SDK 6 维顺序来自 Astron 内部 SDK 文档：

```text
left_thumb_swing
left_thumb_mcp
left_index_mcp
left_middle_mcp
left_ring_mcp
left_little_mcp

right_thumb_swing
right_thumb_mcp
right_index_mcp
right_middle_mcp
right_ring_mcp
right_little_mcp
```

仿真内部手是每侧 11 个 revolute joints，所以 `action_process.py` 做了 6D -> 11D 展开：

```text
thumb_swing -> thumb_cmp
thumb_mcp   -> thumb_mpp + thumb_ip
index_mcp   -> index_mpp + index_ip
middle_mcp  -> middle_mpp + middle_ip
ring_mcp    -> ring_mpp + ring_ip
little_mcp  -> little_mpp + little_ip
```

控制器支持：

```text
26-list/tuple/tensor
dict: left_arm/right_arm/left_hand/right_hand/head/waist/left_leg/right_leg
{"walker_c1": ...}
body 或直接 joint-name dict
```

### `to_ros_data()` Note

C1 的 `to_ros_data()` 当前和 S2 一样，暂时不读 `robot.data.joint_vel`，而是发布 zero velocity：

```text
joint_vel = [0.0] * len(joint_names)
```

原因：Isaac Sim / Isaac Lab 在 CUDA 场景下读 velocity tensor 时可能触发 PhysX device/readback 问题，导致 status 发布失败。当前控制闭环主要需要 position feedback，所以先保证 status 稳定。

### Hand Debug Result

已验证：

```text
ZMQ command -> WalkerC1Controller -> action_process -> Action Manager target
```

链路是通的。发送：

```python
{"right_hand": [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]}
```

status 中右手 11 个 sim joints 的 target 都能正确变成 `0.8`。

关键发现：target 正确，但手部实际关节位置跟随很弱，不是命令链路问题。

在把 hand actuator 临时调强到：

```text
stiffness=200
damping=20
effort_limit_sim=50
velocity_limit_sim=10
```

之后，持续发送闭合命令 20 秒，结果仍大致停在：

```text
R_index_mpp_joint  target=0.8 pos=0.336
R_index_ip_joint   target=0.8 pos=0.035
R_middle_mpp_joint target=0.8 pos=0.327
R_middle_ip_joint  target=0.8 pos=0.034
R_ring_mpp_joint   target=0.8 pos=0.336
R_ring_ip_joint    target=0.8 pos=0.047
R_little_mpp_joint target=0.8 pos=0.353
R_little_ip_joint  target=0.8 pos=0.061
```

结论：

```text
1. C1 手部 command mapping 正确。
2. ZMQ/status/controller 链路正确。
3. 问题集中在仿真手模型/actuator/USD 物理驱动上。
4. 只继续改 ZMQ 或 6D->11D 映射没有意义。
```

下一步应重点查：

```text
walker_c1.usd 中右手 joint 的 physics drive / limit 是否正确
URDF -> USD 导出时是否没有正确设置手指关节 drive
手部 collision/约束是否导致关节卡在某个物理平衡点
是否需要重新导出 USD 或单独简化/禁用手部 collision
```

### Current Hand Actuator Params

当前 `config.py` 中 C1 hand actuator 是调试后的强参数：

```text
WALKER_C1_HAND_STIFFNESS = 200
WALKER_C1_HAND_DAMPING = 20
hand effort_limit_sim = 50
hand velocity_limit_sim = 10
```

这比原始值强很多。它让 MPP 关节从约 `0.22` 改善到约 `0.33~0.36`，但 IP 关节仍几乎不动。后续如果发现 USD/drive/collision 根因，可以再把这组参数收敛到更合理值。

### Important Runtime Notes

`robot.data.joint_names` 的顺序和 Action Manager action order 不一样。调试时必须按 joint name 建 map，不能按 index 对齐。

当前 Action Manager order 中右手段是：

```text
R_index_mpp_joint
R_little_mpp_joint
R_middle_mpp_joint
R_ring_mpp_joint
R_thumb_cmp_joint
R_index_ip_joint
R_little_ip_joint
R_middle_ip_joint
R_ring_ip_joint
R_thumb_mpp_joint
R_thumb_ip_joint
```

而 `robot.data.joint_names` 是 Isaac 内部顺序。status debug 必须使用：

```python
pos = dict(zip(status["joint_names"], status["joint_pos"]))
target = dict(zip(status["target_joint_names"], status["target_joint_pos"]))
```

### Validation Done

只读语法检查已通过：

```text
OK ubt_sim/source/ubt_sim/devices/walker_c1/config.py
OK ubt_sim/source/ubt_sim/devices/walker_c1/action_process.py
OK ubt_sim/source/ubt_sim/devices/walker_c1/controller.py
```

普通 `python3 -m py_compile` 在宿主侧会因为 `__pycache__` 权限报错：

```text
Permission denied: ubt_sim/source/ubt_sim/devices/walker_c1/__pycache__/...
```

这是容器/root 生成缓存导致的文件权限问题，不代表源码语法错误。

### Current Process State

手部 debug 用的 headless CPU sim 已停止。当前没有残留：

```text
/ubt_sim/scripts/sim_runner.py
```

如果后续 `timeout` 后残留 Isaac Python 进程，先查：

```bash
docker exec walker-c1-ubt-sim pgrep -af /ubt_sim/scripts/sim_runner.py
```

只杀精确 PID，例如：

```bash
docker exec walker-c1-ubt-sim kill -9 <pid>
```

不要对整个容器做宽泛 `pkill -f`。

## Morning Standup Summary For 2026-07-11

可以这样汇报：

```text
昨天我把 C1/Astron 从 load-only 推到了可通过 ZMQ 控制的仿真链路。

具体完成了三件事：
1. 新增了 C1 的 action_process 和 controller，sim_runner 已能启动 WalkerC1Controller。
2. 对齐了 Astron SDK 文档里的手部 6 维命令顺序，并实现了 6D SDK hand command 到仿真 11D hand joints 的映射。
3. 做了手部闭环 debug，确认 ZMQ 命令、Action Manager target 和 status 发布链路都是通的。

当前发现的问题是：手部 target 已经正确写入，但实际仿真关节跟随很弱。即使用更强 actuator 参数持续发闭合命令，MCP/MPP 只能动到约 0.33，IP 关节几乎不动。因此问题不在 ZMQ 或映射，而更可能在 C1 USD 资产的 joint drive、URDF->USD 导出参数或手部碰撞/物理约束上。

今天下一步我会集中查 walker_c1.usd 里手部关节的 drive/limit/collision 配置，必要时重新导出或修正 C1 USD。修完后再继续推进 ROS bridge/真机接口，不会先碰真机。
```

如果要更短，可以说：

```text
C1 仿真控制链路昨天已经跑通到 ZMQ controller，手部 6D 到 11D 映射也验证正确。当前 blocker 是手部物理模型：target 到了，但实际关节卡住或跟随很弱。下一步查 USD 里的手部 drive/collision，必要时重新导出 USD，然后再推进 ROS bridge。
```

## Immediate Next Step

下一步建议只做一件事：

```text
用 Isaac/pxr 工具读取 walker_c1.usd 中右手 11 个 hand joints 的 physics drive、limit、joint type、collision API 信息。
```

目标是回答：

```text
为什么 target=0.8 后 IP joints 基本不动？
是 USD drive 没设置/被覆盖？
是关节 limit 或单位转换问题？
是 collision/约束把手指卡住？
```

确认根因后，再决定：

```text
1. 只改 ArticulationCfg actuator 参数；
2. 重新导出 walker_c1.usd；
3. 修改导出脚本参数；
4. 单独处理手部 collision/drive。
```

## 2026-07-10 Update: Hand Gravity Root Cause / Temporary Fix

已完成上一节的 immediate next step，并继续做了运行时隔离测试。

### Added Debug Scripts

新增两个只读/探针脚本：

```text
ubt_sim/scripts/inspect_walker_c1_usd.py
ubt_sim/scripts/probe_walker_c1_hand_runtime.py
```

用途：

```text
inspect_walker_c1_usd.py
  读取 walker_c1.usd 中手部 joint 的 drive、limit、body mass/inertia、collision API。

probe_walker_c1_hand_runtime.py
  直接 spawn WALKER_C1_CFG，绕过 ZMQ/action manager，测试手部 actuator 运行时参数和关节响应。
```

### USD Inspection Result

右手 11 个 joints 在 USD 中都有：

```text
type = PhysicsRevoluteJoint
axis = Z
PhysicsDriveAPI:angular
PhysxJointAPI
IsaacJointAPI
```

limit 从 URDF 正确转换成 USD 角度值，例如：

```text
R_index_mpp_joint lower=0 upper=83.6518 deg  # 约 1.46 rad
R_index_ip_joint  lower=0 upper=92.8192 deg  # 约 1.62 rad
```

USD 里 hand drive 仍保留 URDF effort：

```text
stiffness=625
damping=0
max_force=1.35
type=acceleration
```

但 Isaac Lab 运行时 `config.py` 的 hand actuator 覆盖是生效的：

```text
stiffness=200
damping=20
effort_limit_sim=50
velocity_limit_sim=10
```

hand link mass/inertia 也正常，没有明显异常大质量：

```text
R_index_mpp_link mass ~= 0.0147 kg
R_index_ip_link  mass ~= 0.0133 kg
R_palm_link      mass ~= 0.3822 kg
```

runtime 还确认：

```text
num_fixed_tendons = 0
joint_armature = 0
joint_friction_coeff = 0
joint_dynamic_friction_coeff = 0
joint_viscous_friction_coeff = 0
```

所以当前已排除：

```text
1. ZMQ/action mapping 问题
2. Action Manager target 顺序问题
3. USD joint limit 单位问题
4. hand actuator config 没写入的问题
5. fixed tendon / friction / armature hidden constraint 问题
6. hand link mass/inertia 明显异常问题
```

### Root Cause Found

关键对照测试：

```bash
docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u /ubt_sim/scripts/probe_walker_c1_hand_runtime.py --headless --device cpu --steps 600 --target 0.8
```

在 gravity 开启时，即使直接对右手 11 个 joints 设置 target=0.8，非拇指关节仍明显跟随失败，effort 会打满：

```text
R_index_mpp_joint target=0.8 pos ~= 0.116
R_index_ip_joint  target=0.8 pos ~= 0.000
applied_effort    ~= 50 saturated
```

把 effort/stiffness 临时加到更高只能部分改善，但仍无法稳定到 target：

```text
--hand-effort 500 --hand-stiffness 1000 --hand-damping 40
R_index_mpp_joint pos ~= 0.447
R_index_ip_joint  pos ~= 0.125
```

禁用 robot gravity 后，同一个 target=0.8 能稳定跟随：

```text
R_thumb_cmp_joint  pos=0.806
R_thumb_mpp_joint  pos=0.801
R_thumb_ip_joint   pos=0.799
R_index_mpp_joint  pos=0.799
R_index_ip_joint   pos=0.797
R_middle_mpp_joint pos=0.799
R_middle_ip_joint  pos=0.798
R_ring_mpp_joint   pos=0.799
R_ring_ip_joint    pos=0.799
R_little_mpp_joint pos=0.800
R_little_ip_joint  pos=0.799
```

结论：

```text
当前 C1/Astron hand 在 Isaac 中的主要问题是 gravity 下的手指驱动不足/下垂，不是控制链路或 USD joint metadata 错误。
```

### Applied Temporary Fix

已修改：

```text
ubt_sim/source/ubt_sim/devices/walker_c1/config.py
```

C1 `ArticulationCfg.spawn.rigid_props` 现在为：

```python
sim_utils.RigidBodyPropertiesCfg(disable_gravity=True)
```

理由：

```text
C1 当前阶段是 fixed-root upper-body simulation，目标是先跑通 ZMQ/controller/task 链路。
开启 gravity 时手部姿态无法跟随，且会误导后续任务调试。
禁用 C1 robot gravity 后，手部 target 跟随稳定，物体/场景 gravity 不受影响。
```

保留当前 hand actuator 调试参数：

```text
WALKER_C1_HAND_STIFFNESS = 200
WALKER_C1_HAND_DAMPING = 20
hand effort_limit_sim = 50
hand velocity_limit_sim = 10
```

不要立刻降回原始 `10/2/2`；禁用 gravity 后低参数仍不能稳定驱动所有 IP joints。

### Validation After Fix

已运行：

```bash
PYTHONPYCACHEPREFIX=/tmp/c1_pycache python3 -m py_compile \
  ubt_sim/source/ubt_sim/devices/walker_c1/config.py \
  ubt_sim/scripts/inspect_walker_c1_usd.py \
  ubt_sim/scripts/probe_walker_c1_hand_runtime.py
```

结果：通过。

已运行默认配置探针：

```bash
docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u /ubt_sim/scripts/probe_walker_c1_hand_runtime.py --headless --device cpu --steps 600 --target 0.8
```

结果：右手 11 个关节全部稳定到约 `0.797~0.806`，证明 `config.py` 里的 `disable_gravity=True` 已生效。

### Recommended Next Step

下一步可以回到 C1 ZMQ 仿真链路，重新跑：

```bash
docker exec walker-c1-ubt-sim bash -lc "cd /ubt_sim && UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0 UBT_SIM_NO_BRIDGE=1 timeout 360s bash scripts/start_sim.sh --headless --device cpu --step_hz 30"
```

然后发送：

```python
{"right_hand": [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]}
```

预期：

```text
status target_joint_pos 和实际 joint_pos 都应接近 0.8。
```

如果这个验证通过，再继续推进：

```text
1. 清理/保留 debug scripts 的取舍；
2. C1 task/controller 的更完整 smoke test；
3. 后续 ROS bridge 或数据采集链路。
```

## 2026-07-10 Update: Mimic Ratio Trial With Gravity Enabled

用户指出 Tiankung/S2 都没有禁用 gravity。已确认：

```text
walker_s2/config.py      disable_gravity=False
tienkung_pro/config.py   disable_gravity=False
```

因此 C1 也已改回：

```text
ubt_sim/source/ubt_sim/devices/walker_c1/config.py
disable_gravity=False
```

尝试过“主关节 + mimic ratio”方向，但只在 probe 脚本里做测试，没有写入正式 ZMQ 映射。

新增/扩展 probe 参数：

```text
--ip-ratio
--thumb-ip-ratio
```

测试 1：

```bash
docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u /ubt_sim/scripts/probe_walker_c1_hand_runtime.py --headless --device cpu --steps 600 --target 0.8 --ip-ratio 0.5 --thumb-ip-ratio 0.5
```

结果：非拇指 MPP 仍然只有约 `0.11~0.15`，IP 仍接近 `0`：

```text
R_index_mpp_joint  target=0.8 pos=0.1167
R_index_ip_joint   target=0.4 pos≈0.0
R_middle_mpp_joint target=0.8 pos=0.1076
R_middle_ip_joint  target=0.4 pos≈0.0
R_ring_mpp_joint   target=0.8 pos=0.1224
R_little_mpp_joint target=0.8 pos=0.1475
```

测试 2：

```bash
docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u /ubt_sim/scripts/probe_walker_c1_hand_runtime.py --headless --device cpu --steps 600 --target 0.8 --ip-ratio 0.0 --thumb-ip-ratio 0.0
```

结果：只驱动主关节也没有明显改善：

```text
R_index_mpp_joint  target=0.8 pos=0.1168
R_middle_mpp_joint target=0.8 pos=0.1077
R_ring_mpp_joint   target=0.8 pos=0.1225
R_little_mpp_joint target=0.8 pos=0.1473
```

结论：

```text
mimic ratio 是以后让手指动作更自然的合理映射方式，但它不能解决当前 gravity 开启时非拇指 MPP 主关节本身就抬不起来的问题。
所以暂时不要把 mimic ratio 写进 C1 正式 action_process.py。
```

当前更可信的后续方向：

```text
1. 保持 C1 disable_gravity=False，和 Tiankung/S2 一致；
2. 不靠 mimic ratio 修 gravity 下垂；
3. 下一步查手指关节 frame/axis、手部姿态相对 gravity 的方向、是否需要只对 hand links 做重力补偿或修 USD/URDF 物理参数。
```

## 2026-07-10 Update: C1 Reset Semantics

这里明确区分两种 reset：

```text
1. 按 R / ZMQ {"reset": true}
   这是 Isaac 场景 reset：sim_runner.py 执行 env.sim.reset(); env.reset()。
   用途是把仿真状态回到 config.py 里的 WALKER_C1_HOME_POSE / 0 位。

2. ubt_sim/teleoperation/control/walker_c1/reset.py
   这是抓取任务开始前的 task reset pose。
   用途是把 C1/Astron 摆到当前 parlor/tabletop 抓取任务的初始姿态。
```

新增文件：

```text
ubt_sim/teleoperation/control/walker_c1/__init__.py
ubt_sim/teleoperation/control/walker_c1/constants.py
ubt_sim/teleoperation/control/walker_c1/reset.py
```

`constants.py` 里的 `TASK_RESET_BODY_POSE` 参考 Tiankung `ARM_HOME_PICK_PLACE` 的思路：

```text
头部先给任务视角；
手部张开；
双臂先经过 clear pose 避碰；
最后发布抓取任务初始姿态。
```

默认真机/ROS 用法：

```bash
/usr/bin/python3 /ubt_sim/teleoperation/control/walker_c1/reset.py
```

仿真 ZMQ 任务姿态用法：

```bash
/usr/bin/python3 /ubt_sim/teleoperation/control/walker_c1/reset.py --mode sim-task
```

如果只想做“按 R 那种场景 reset”，不要用默认 `reset.py`，用键盘 R；脚本里仅保留显式 fallback：

```bash
/usr/bin/python3 /ubt_sim/teleoperation/control/walker_c1/reset.py --mode sim-scene
```

注意：当前 `TASK_RESET_BODY_POSE` 只是按 Tiankung 抓放姿态语义映射到 C1 关节名的第一版，需要后续根据 C1 在 parlor 场景里的实际抓取高度、桌面位置、相机视角继续调。

## 2026-07-10 Update: C1 Action Offset Trial Reverted

用户反馈 C1 启动仿真后脚/初始姿态不对。排查结论：

```text
WALKER_C1_HOME_POSE 最近提交没有改腿部姿态；
腿部自然站姿仍是 hip/ankle=0，knee=0.08；
parlor.yaml root 高度仍是 z=0.90，和 MJCF base_link pos=0.91 基本一致。
```

试过一个假设：C1 controller/action_process 输出的是绝对 joint target，而 `walker_c1_parlor_env_cfg.py` 里除了手部以外，arm/head/waist/leg 没有显式 `use_default_offset=False`，可能导致 default offset 与绝对 target 叠加。

但用户反馈重启后腿部仍不对/疑似更差，所以该试验已撤回，当前不要保留这个改动。

```text
ubt_sim/source/ubt_sim/task/walker_c1_parlor/walker_c1_parlor_env_cfg.py
```

当前状态：

```text
只有 hand action 保持 use_default_offset=False，和 Walker S2 写法一致；
arm/head/waist/leg action 未显式设置 use_default_offset。
```

下一步不要继续猜 offset；应优先确认：

```text
1. 当前启动是否仍带 UBT_SIM_LOAD_ONLY=1；
2. 是否有多个 Isaac/sim_runner 残留进程；
3. load-only 下脚是否正常；
4. 非 load-only controller 启动后脚是否变化；
5. 如果 load-only 也飘，重点查 parlor.yaml 的 robot init_state.z / 场景地面高度 / USD root 定义。
```

## 2026-07-10 Update: 41e2603 Confirmed As Good Standing Baseline

用户临时切到：

```text
41e260371fe96206903a9805fe2bd5538c26b7a3
```

并用昨天的 load-only 命令观察，确认该状态下 C1/Astron 初始站姿自然：

```text
腿部正常站着；
脚尖朝前；
身体挺直。
```

因此用户之前关于“昨天看到的姿态是正常的”的记忆是有效 baseline，不应再假设用户看错。

随后用户已执行：

```bash
git switch walker_c1
git stash pop
```

当前回到：

```text
walker_c1
```

`stash pop` 成功，恢复了之前未提交改动，并删除了 `refs/stash@{0}`。

当前工作区仍有未提交内容：

```text
M  C1_HANDOFF.md
?? ubt_sim/teleoperation/control/walker_c1/
?? 【CC-API】Astron优必选SDK二次开发文档【对内】.docx
```

注意：`.docx` 是本地 SDK 文档参考文件，不要默认加入 git。

### Verified Diff From Good Baseline To walker_c1

已对比 `41e2603` 和当前 `walker_c1` 分支：

```bash
git diff --name-status 41e260371fe96206903a9805fe2bd5538c26b7a3..walker_c1 -- \
  ubt_sim/assets/robots/walker_c1 \
  ubt_sim/source/ubt_sim/devices/walker_c1 \
  ubt_sim/source/ubt_sim/task/walker_c1_parlor \
  ubt_sim/config/walker_c1 \
  ubt_sim/scripts
```

结果关键点：

```text
C1 robot assets 没变：
  ubt_sim/assets/robots/walker_c1/*.usd / *.urdf / *.xml 没有 diff

parlor task 配置没变：
  ubt_sim/config/walker_c1/parlor.yaml 没有 diff
  ubt_sim/source/ubt_sim/task/walker_c1_parlor/walker_c1_parlor_env_cfg.py 没有 diff

C1 HOME_POSE 腿部没变：
  hip/ankle 仍是 0
  knee 仍是 0.08

gravity/root 没变：
  disable_gravity=False
  fix_root_link=True
```

相对 good baseline，`walker_c1` 分支相关变化主要是：

```text
1. sim_runner.py 允许 Walker C1 非 load-only 启动 WalkerC1Controller；
2. 新增 ubt_sim/source/ubt_sim/devices/walker_c1/action_process.py；
3. 新增 ubt_sim/source/ubt_sim/devices/walker_c1/controller.py；
4. 新增 hand debug/inspect 脚本；
5. config.py 只改了 hand actuator 参数。
```

`config.py` 中唯一物理配置差异是手部 actuator 从原始弱参数改成调试强参数：

```text
41e2603:
  WALKER_C1_HAND_STIFFNESS = 10
  WALKER_C1_HAND_DAMPING = 2
  hand effort_limit_sim = 2
  hand velocity_limit_sim = 3

walker_c1:
  WALKER_C1_HAND_STIFFNESS = 200
  WALKER_C1_HAND_DAMPING = 20
  hand effort_limit_sim = 50
  hand velocity_limit_sim = 10
```

理论上手部 actuator 不应影响腿部站姿，但从 git diff 看，这是当前分支和 good baseline 之间唯一会影响 C1 articulation 物理配置的变更。事实优先，后续应做最小 A/B 验证。

### Load-only Command Status

用户昨天命令：

```bash
docker exec -it walker-c1-ubt-sim bash -lc \
  "cd /ubt_sim && UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0 UBT_SIM_LOAD_ONLY=1 bash scripts/start_sim.sh"
```

已确认当前脚本仍会正确展开成：

```text
/isaac-sim/python.sh -u /ubt_sim/scripts/sim_runner.py \
  --task UBTSim-WalkerC1-Parlor-v0 \
  --enable_cameras \
  --num_envs 1 \
  --load_only \
  --device cpu
```

短启动验证日志中确实出现：

```text
[INFO] Walker C1 load-only mode: ROS control and action preprocessing are disabled.
[INFO] Load-only app update enabled: physics/action/observation stepping is disabled.
```

所以 `UBT_SIM_LOAD_ONLY=1` 本身没有失效。

判断标准：

```text
看到 "Walker C1 load-only mode" => load-only 生效；
看到 "Walker C1 Controller: simulation ZMQ interface enabled" => 当前不是 load-only，或者另有非 load-only 进程。
```

load-only 下 `reset.py --mode sim-task` 不会生效，因为 load-only 不启动 `WalkerC1Controller`，也不会监听 ZMQ 命令。

### Current Best Next Step For Standing Pose

当前不要再大范围猜 gravity、root height、parlor init pose 或 action offset；这些和 good baseline 的 diff 不匹配。

建议下一步只做一个最小验证：

```text
在 walker_c1 分支，把 hand actuator 参数临时恢复成 41e2603 的值；
重新跑 load-only；
看腿部/身体初始姿态是否恢复自然。
```

如果恢复自然：

```text
说明强 hand actuator 对整机 articulation 初始稳定性产生了副作用；
应拆分手部跟随修复，不要直接保留强参数进主配置。
```

如果仍不自然：

```text
说明差异可能不在 git-tracked 源码，需检查：
1. 容器内是否真挂载当前工作区；
2. 是否有残留 Isaac 进程；
3. 是否有未跟踪/生成的 USD 或缓存状态；
4. 视觉观察是否在同一视角/同一启动模式下比较。
```

### Branch Safety

`41e2603` 是 detached HEAD baseline，只用于对照观察，不要在 detached HEAD 上继续开发。

继续开发应回到：

```bash
git switch walker_c1
```

如果需要永久保留 good baseline，可新建分支：

```bash
git switch -c walker_c1_good_baseline 41e260371fe96206903a9805fe2bd5538c26b7a3
```

## 2026-07-14 Update: 站姿根因修复 + 左腿碰撞（含一条死胡同）

本节是最新、最重要的状态。上面很多"站姿不对/内八"的猜测在这一轮被**数据推翻并定案**了。

### 一句话总览

```
站姿核心问题已修好并 push（commit f4a4c2e）：
  1. C1 actuator 刚度过弱 -> 对齐 S2；
  2. controller 把 reset 乱帧当保持目标 -> 改为锚定 HOME_POSE。
剩一个纯视觉小尾巴：左腿在 parlor 里撞家具被拧 ~35°（腿是固定基座、不参与任务）。
关腿碰撞的"运行时"做法被证明是死胡同（见下），已回退，sim 恢复正常。
```

### 关键结论 1：站姿"不对"不是代码回归，"脚尖朝前的好版本"= 关了重力

用 `dump_walker_c1_joint_state.py` 在 `41e2603`（用户记忆里"好"的 baseline）和 `dc2fc83`（当时 HEAD）各跑一次 load-only：

```
两次 env.reset() 后的 53 个关节值逐字节完全相同。
```

含义：

```
- load-only 下这两个 commit 在所有影响姿态的输入上等价，必然渲染成同一姿态。
- 用户记忆里"41e2603 脚尖朝前"其实是 41e2603 + 未提交的 disable_gravity=True
  （那次实验后来 revert 了）。commit 哈希是障眼法，真正区别是"重力关 vs 开"。
```

### 关键结论 2：load-only 不能用来判断站姿（physics 没跑）

```
sim_runner.py 的 load-only 分支只调 simulation_app.update()（纯渲染），
从不调 env.step()，所以 physics 不推进，机器人冻结在 reset 后那一帧。
改 actuator 刚度对 load-only 姿态零影响（实测 0 步 dump 改刚度前后逐字节相同）。
=> 判断站姿必须用 controller 模式（env.step 在跑），或用 dump 脚本 --steps 让物理 settle。
```

### 关键结论 3：站姿根因 = C1 刚度是没调过的占位弱值

对照 `walker_s2/config.py`（重力下能站住的成熟参考）：

```
部位   S2 stiffness        C1 修复前   C1 修复后(=对齐S2)
腿 hip_roll/yaw   1100     200        1100
腿 hip_pitch/knee 1500     200        1500
腿 ankle          1600     200        1600
头 head            600      80         600
腰 waist           600     120         600
臂 arm         500~600      80      500~600
手 hand      (夹爪1200)    200       200(未动，另一条线)
```

修复（`config.py`，已提交 f4a4c2e）：把臂/腰/腿/头刚度阻尼对齐 S2 量级，阻尼腿 55/65/70、头腰 60、臂 40。**手部刚度未动。**

### 关键结论 4：controller "拿 reset 乱帧当目标"是第二个 bug（已修）

现象：刚度提上去后，controller 模式启动时**左腿平飞到侧面、右腿甩到头顶**。

根因（`action_process.py` 的 `to_controller_data`）：

```
原逻辑：_hold_joint_targets 首次从"当前关节位置"捕获（好意：启动先 hold 当前位，别猛跳）。
但 env.reset() 后"当前位置"正是那一帧乱姿态（L_hip_roll≈2.94、R_hip_pitch≈2.5）。
弱刚度时电机够不到乱目标，只是软塌（看不出）；强刚度时电机有劲，
就把腿死命甩到乱目标：L_hip_roll 2.94->左腿平展、R_hip_pitch 2.5->右腿到头。
```

修复（`action_process.py`，已提交 f4a4c2e）：

```python
# 启动保持目标锚定 HOME_POSE，而不是读乱掉的当前位置
_hold_joint_targets = {name: WALKER_C1_HOME_POSE.get(name, 0.0) for name in action_joint_names}
```

验证（走**真实 controller 动作路径** `to_controller_data + env.step` 300 步）：

```
头 head_pitch 0.065、腰≈0、双臂<0.05、右腿全部≈0。
max|diff|=0.81 落在 L_thumb_mpp（手，未动）。
=> 头/躯干/臂/右腿全部 hold 住 HOME_POSE。
```

### 关键结论 5：reset 会甩腿的机制（为什么左右腿路径不同）

```
机器人固定在桌边(root z=0.9, fix_root_link=True)，腿垂下来时和 parlor 家具几何重叠；
PhysX 初始化时猛推消穿模 -> reset 出不对称乱帧（左腿甩得比右腿远）；
电机往 HOME 拉时，两腿从不同乱起点绕回：左腿"往前绕"路径扫过桌子、被别住；
右腿"往后绕"路径是空的、干净归位。
```

### 关键结论 6：左腿残留 = parlor 场景碰撞（已坐实，非机器人问题）

剩余现象：修好后左腿仍拧 `L_hip_yaw≈0.6`（~35°），右腿完美(<0.13)。诊断：

```
- 隔离测试 probe_walker_c1_leg_isolation.py（机器人单独 spawn、无场景）：
    两腿完美对称，max|L|-|R|=0.0000  => 排除机器人/刚度/控制，锁定"场景碰撞"。
- 定位 probe_walker_c1_leg_collision.py（parlor 内，dump 左右腿连杆世界坐标）：
    两脚同高 Z≈0.05（左脚没踩在桌面上），但左脚在 +Y 被撇出去(左+0.16 vs 右-0.10)
    => 左脚/左小腿在脚踝高度被侧向顶（大概率桌腿），右腿路径干净。
- 脚是悬空的（fix_root_link=True，上半身操作标准做法），所以不是穿地。
- URDF 里左右 hip_yaw 限位是正常镜像对，0 在合法范围内 => 不是关节限位。
```

### 死胡同（重要，别再走）：运行时关腿碰撞

目标：腿是固定不用的，关掉腿↔场景碰撞即可根治左腿被别。做了开关设计（默认关）：

```
config.py:   WALKER_C1_DISABLE_LEG_COLLISION = True
env 覆盖:     UBT_SIM_C1_DISABLE_LEG_COLLISION=0  (打开腿碰撞)
helper:      scene_setup.py  apply_leg_collision_setting(env) / disable_leg_collision(env)
```

尝试与失败链：

```
1. env.sim.stage 遍历 -> 找到 0 个碰撞 prim（错的 stage 句柄）。
   正解：omni.usd.get_context().get_stage()。碰撞 prim 在
   /World/envs/env_0/Robot/<link>/collisions/<link>/mesh（12 个腿部）。
2. 直接改碰撞属性 -> 报错 "authoring to an instance proxy is not allowed"
   （腿连杆子树是 USD 实例代理，不能直接改）-> 崩、机器人卡死。
3. 先 SetInstanceable(False) 去实例化再改 -> 能关掉 12 个、不报那个错，
   但 (a) 仿真变得极慢（120 步 9 分钟都跑不完，正常 300 步 5 分钟），
   (b) env.reset() 崩："Simulation view object is invalidated ...
       Failed to set DOF actuation forces"。
   => 结论：sim play 之后再改 USD（去实例化/碰撞）会让 PhysX 视图失效。
      运行时关碰撞在原理上走不通，不是调参能救的。
```

处置：

```
- 已从 sim_runner.py 撤回 apply_leg_collision_setting 调用，sim 恢复正常（能启动、不崩）。
- scene_setup.py / config.py 的 WALKER_C1_DISABLE_LEG_COLLISION flag / inspect 脚本
  仍留在工作区但【未提交】，标记为死胡同实验，勿再用运行时路径。
```

### 左腿的正确修法（尚未做）

```
在 sim 加载之前把腿碰撞关掉，而不是运行时改。方案：离线烤一个
walker_c1_no_leg_collision.usd（打开 walker_c1.usd，把 12 个腿部
collision mesh 的 collisionEnabled 设 False，另存），config 按
WALKER_C1_DISABLE_LEG_COLLISION 选择加载哪个 USD。
优点：解析时碰撞就是关的，PhysX 从头认，不崩、不慢，仍可切回原 USD 打开腿碰撞。
（编辑源 USD 时 prim 不是实例代理，可直接改，无 instance proxy 问题。）
```

或者：左腿纯视觉、腿不干活，**先搁置**当已知小尾巴也完全可以。

### 本轮 commit / 文件状态

已提交并 push（remote 已是 SSH `git@github.com:UBTECH-Robot/TienKung-IL-LAB.git`，push 通）：

```
f4a4c2e Fix Walker C1 standing pose: S2-aligned gains + HOME_POSE hold targets
  - config.py（刚度对齐 S2）
  - action_process.py（锚定 HOME_POSE）
  - scripts/dump_walker_c1_joint_state.py（新，只读诊断）
  - scripts/probe_walker_c1_leg_isolation.py（新）
  - scripts/probe_walker_c1_leg_collision.py（新）
1db36ba Add Walker C1 teleoperation reset scaffolding and handoff notes
  - teleoperation/control/walker_c1/{__init__,constants,reset}.py
  - C1_HANDOFF.md
```

工作区【未提交】（多为死胡同实验，谨慎处理）：

```
M dump_walker_c1_joint_state.py         (加了 --use-controller / --disable-leg-collision 开关)
M config.py                             (加了 WALKER_C1_DISABLE_LEG_COLLISION flag)
?? scene_setup.py                       (运行时关碰撞 helper —— 死胡同)
?? scripts/inspect_c1_collision_prims.py(找碰撞 prim 的探针)
?? 【CC-API】...SDK...【对内】.docx      (对内文档，勿入 git)
```

### 诊断脚本速查（都是只读/探针）

```
dump_walker_c1_joint_state.py
  建 parlor env、reset 后 dump 53 关节 vs HOME_POSE。
  --steps N            reset 后驱动 HOME_POSE 目标 step N 步再 dump（=物理 settle）
  --use-controller     走真实 controller 动作路径(to_controller_data+env.step) 而非手喂
  --disable-leg-collision  调用 scene_setup 关腿碰撞（死胡同，会崩，勿用于正式）
probe_walker_c1_leg_isolation.py  机器人单独 spawn（无场景），验左右腿对称
probe_walker_c1_leg_collision.py  parlor 内 dump 左右腿连杆世界坐标，定位撞点
inspect_c1_collision_prims.py     打印 body 名 + 碰撞 prim 路径（用 omni.usd stage + 实例代理遍历）
```

运行例（headless，注意这台机器 Isaac 启动+300步约 5 分钟）：

```bash
docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u \
  /ubt_sim/scripts/dump_walker_c1_joint_state.py \
  --headless --device cpu --enable_cameras --steps 300 --use-controller
```

GUI 看站姿（controller 模式，物理在跑）：

```bash
docker exec -it walker-c1-ubt-sim bash -lc \
  "cd /ubt_sim && UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0 UBT_SIM_NO_BRIDGE=1 \
   bash scripts/start_sim.sh --device cpu --step_hz 30"
```

### 运行时踩坑备忘

```
- 判断站姿别用 load-only（physics 不跑，冻在 reset 乱帧）；用 controller 模式。
- env.sim.stage 看不到 spawn 出来的碰撞 prim；要用 omni.usd.get_context().get_stage()。
- 机器人腿连杆是 USD 实例代理，不能直接改属性。
- sim play 之后改 USD（碰撞/实例化）会让 PhysX tensor view 失效 -> env.reset() 崩。
- 这台机器 Isaac 单次 headless（启动+300 步+相机）约 5 分钟，多实例会更慢/抢显存；
  timeout/残留进程用 pgrep -af 精确杀 PID，别宽泛 pkill。
```

### 下一步建议

```
1.（可选）左腿：走"烤 USD"正解，或先搁置（纯视觉、腿不用）。
2. 把未提交的死胡同实验清理或明确标注后再决定去留。
3. 回主线：确认 C1/Astron 真实 SDK joint order（身体 RobotCommand 顺序、
   左右手 JointCommand 顺序），这才是影响真机的关键；仿真物理调参不影响真机。
4. 手部 droop 仍是独立未结项（拇指等在重力下 hold 不住，disable_gravity 时正常）。
```

## 2026-07-14 Update（下午）：目标转仿真数据采集 + 上肢重力 droop 根因定案

本节是最新、最重要的状态。**注意：上面几节把"手部 droop / 手臂 hold 不住"当独立未结项，这一轮把它们
一锅端定案了——根因是 USD 关节 drive 是 acceleration 型，不是 force 型。**

### 一句话总览

```
1. 目标变了：用户明确"暂不做真机，先在仿真里跑通规划数据采集，只需要数据生成 HDF5"。
   （LeRobot 转换 out of scope；ROS bridge / 真机搁置。）
2. M1 已完成：脚本化动作 -> 录制 -> HDF5（26 维 obs/action + camera_head），已验证。
3. M2（真实抓取）挖出并定案了 droop 根因：
   C1 所有关节 drive 是 type=acceleration -> 等效力矩刚度 = stiffness×inertia ≈ 0
   -> 上肢重力下没劲、垂下去。改成 force 驱动后彻底 hold 住（已验证）。
4. 正式修法（烤 force-drive USD）脚本已写好，尚未执行（等用户过一遍再动）。
```

### 目标转向（重要）

用户新方向：**在仿真里跑通"脚本化运动规划 + 数据采集"，产物就是 HDF5**。
- 采集链路对标 `ubt_IL/dataset/sim_pick_place`（天工采的：26 维 obs/action + `camera_head`，fps15）。
- **只要 HDF5**：LeRobot 转换（`convert_to_lerobot.py` + 配置 `Walker_C1_26_1RGB.json`）备着但不跑。
- 驱动方式用户选定：**脚本化运动规划**（自动抓放、批量录，无人 teleop），后来进一步要求**真实物理抓取**（不接受吸附/关重力）。

### M1 已完成：HDF5 采集链路（in-process，不走 ZMQ/ROS）

新脚本 `ubt_sim/scripts/collect_walker_c1_pick_place.py`：
- 直接建 `UBTSim-WalkerC1-Parlor-v0` env（非 load-only，物理在跑），脚本化右臂波点 + 逐帧录制。
- 产出 `ubt_sim/dataset/walker_c1/<ts>/trajectory.hdf5`，schema 与天工一致
  （`puppet/*` + `action/*` + `camera_observations/color_images/camera_head` JPEG）。
- 已验证：`observation.state`=26、`action`=26、相机解码 (480,640,3)。复用 `action_process.py`
  的 `to_controller_data`/`to_ros_data`。
- 运行：`docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u /ubt_sim/scripts/collect_walker_c1_pick_place.py --headless --device cpu --enable_cameras --episodes 1`

### M2（真实抓取）路上发现的场景问题

- **parlor 场景自带的桌子没有碰撞**（只有外观）——物体会穿过去掉地上。
- 场景里的 **fruit USD 不是刚体**（没有 RigidBodyAPI），不能直接当 RigidObject spawn。
- 处理：在 `walker_c1_parlor_env_cfg.py` 里**自己加了一张静态碰撞桌 `GraspTable`（Cuboid）+ 一个球
  RigidObject**（先用 primitive sphere，保证是正经刚体），放在右手够得到的桌面。
- 抓取测试 `probe_walker_c1_grasp.py`：inconclusive——**卡在"手臂够不到球"**（手停在桌子近边后
  15cm，抬手也几乎抬不动），暴露出真正的上游问题是**手臂在重力下 hold 不住**。

### 关键结论：上肢 droop 根因 = 关节 drive 是 acceleration 型（已定案 + 已验证修复）

用隔离探针 `ubt_sim/scripts/probe_walker_c1_arm_tracking.py`（机器人单独 spawn、量 命令vs实际 关节角）
系统排查，**逐项确认正常并排除**：关节限位、config 刚度/阻尼/effort（runtime 确认生效 500/40/80）、
连杆质量（臂+手 4.85kg、全身 57kg）、惯量（~0.0015）、质心偏移（几 cm）、USD 单位（1.0/1.0）、
碰撞/tendon/friction/armature。**effort 从 25 提到 300 都扛不住，且停位与 effort 无关；重力关掉则一切正常。**

根因（读 USD drive type 定案）：

```
[USD DRIVE] R_elbow_pitch_joint: TYPE=acceleration  maxForce=25  stiffness=625  damping=0
```

C1 所有关节 drive 是 **`type=acceleration`（加速度驱动）**，不是 `force`。加速度驱动下 PhysX 把驱动按
关节等效惯量缩放：等效力矩刚度 ≈ stiffness×inertia ≈ 500×0.0015 ≈ 近 0 → 扛不住重力；提 effort 也
被惯量缩放故无效。之前"重力像放大 40 倍"是假象——是驱动力太弱。S2/天工用 force 驱动所以正常。
来源：URDF→USD 导入时 drive 默认成 acceleration。

**验证**（`probe_walker_c1_arm_tracking.py --force_drive`，spawn 前把 drive 改 force，重力开 + effort 80）：

```
             was(accel)   now(force)
ready  elbow  err 0.68  -> err 0.028   holding torque ~10 N·m
reach_fwd     err 0.49  -> err 0.026
lift_up       err 1.02  -> err 0.027
arm_down      err 0.35  -> err 0.021
```

**根因确认、修法有效。手指 droop 大概率同源（同一 acceleration 问题），改 force 后需一并复测。**

### 正式修法：烤 force-drive USD（脚本已写，尚未执行）

`ubt_sim/scripts/bake_walker_c1_force_drive_usd.py`：打开 `walker_c1.usd`，把每个关节 DriveAPI 的 type
从 acceleration 改 force，**另存 `walker_c1_force_drive.usd`（原文件不动）**；只改 type，不碰
stiffness/maxForce。之后 config 的 `WALKER_C1_USD_PATH` 指向新 USD 即可（回退就指回原文件）。

```bash
# 先 dry-run 看扫到多少关节（应 ~53 个可动关节）
docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u /ubt_sim/scripts/bake_walker_c1_force_drive_usd.py --dry_run
# 真烤
docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u /ubt_sim/scripts/bake_walker_c1_force_drive_usd.py
```

⚠️ 改 force 后**腿/腰/头刚度会真正生效**（之前 acceleration 下它们几乎没力、全靠 fix_root_link 挂着）。
烤完必须**重跑完整任务确认整机站姿/稳定**，再复测手指 droop（可能好了，也可能手部 stiffness 要重调）。

### 本轮提交内容（checkpoint push，force-drive 修复前）

以下改动在本节写入后作为一个 checkpoint 版本提交/推送（**force-drive 修复本身尚未落地**）：

```
M  config.py                       arm effort 60/25 -> 100/80（S2 对齐；对 acceleration 无用，等 force 后重估）
M  walker_c1_parlor_env_cfg.py     加 GraspTable 碰撞桌 + 球 RigidObject（M2 用）
M  C1_HANDOFF.md                   本文档
+  scripts/collect_walker_c1_pick_place.py     M1 采集主脚本
+  scripts/probe_walker_c1_workspace.py        右手工作空间探针
+  scripts/probe_walker_c1_grasp.py            抓取测试
+  scripts/probe_walker_c1_arm_tracking.py     ★手臂跟踪诊断（--no_gravity/--force_drive/读 drive type等）
+  scripts/bake_walker_c1_force_drive_usd.py   force-drive 烤 USD（脚本，未执行）
+  ubt_IL/scripts/convert/configs/Walker_C1_26_1RGB.json   LeRobot 转换配置（暂不用）
```

不入 git：`【CC-API】...SDK...docx`（对内文档）。更早的死胡同文件（scene_setup.py / inspect_c1_collision_prims.py）
已在早前清理时删除，不在工作区。`C1_joint_map.md` 已在 commit 3c90d98 提交（含 SDK cross-check）。

诊断脚本 `probe_walker_c1_arm_tracking.py` 是本轮最有用的资产：支持 `--no_gravity`、`--force_drive`，
并打印 sim 生效的 刚度/阻尼/effort、关节限位、连杆质量/惯量/质心、USD 单位、USD drive type。

### 下一步顺序

```
1. （等用户点头）跑 bake_walker_c1_force_drive_usd.py 烤 force-drive USD；config 指向它。
2. 重跑完整任务确认整机站姿/稳定（force 后腿/腰/头刚度真生效）。
3. 复测手指 droop（同源，大概率好转）。
4. 回到 M2：手臂能 hold 了，重做抓取（IK 或波点 -> 抓球 -> 移到目标），成功才存 HDF5。
5. 最后统一整理工作区未提交改动 + 提交。
```

## 2026-07-14 Update（傍晚）：force-drive 修复已落地 + 验证 + 准备姿势已调

### force-drive 修复已烤 USD 并落地（步骤 1、2 完成）

- dry-run 确认：`walker_c1.usd` 里 **全部 53 个可动关节** drive 都是 `acceleration`。
- 已烤 **`ubt_sim/assets/robots/walker_c1/walker_c1_force_drive.usd`**（15.8MB，53/53 acceleration→force，
  只改 type，不动 stiffness/damping/maxForce；**原 walker_c1.usd 不动**，回退就把 config 指回去）。
- `config.py` 的 `WALKER_C1_USD_PATH` 已指向 `walker_c1_force_drive.usd`。
- **完整 parlor 任务复测通过**（`probe_walker_c1_workspace.py`，force USD + 重力开）：
  root 稳在 [7.80, 6.08, 0.90]、不炸；R_palm 抬到 z=0.905（修复前垂在 ~0.80）；腿/腰因刚度真生效没抖没崩。
  **手臂在重力下能 hold 住准备姿势了。** 用户 GUI 目视确认满意。

### 准备姿势已"略微张开"（用户要求，已满意）

`constants.py::TASK_RESET_BODY_POSE` 的肩外展从近 0 调到 ±0.30 rad（约 17°）让双臂外张：
`L_shoulder_roll 0.068→0.30`、`R_shoulder_roll -0.003→-0.30`（方向：L 正/R 负 = 外展；range L[-0.139,1.884]/R[-1.884,0.139]）。
`collect_walker_c1_pick_place.py` 的 READY_*ARM 已同步同值，两边一致。

### 手部 droop：根因同源，已被同一修复覆盖，但【尚未复测】——下次做

- **手指 droop 和手臂 droop 是同一个 bug**：手指关节 drive 也是 acceleration。手指链惯量极小（~0.00002，
  手指 13g），acceleration 缩放后等效刚度 ≈ 200×0.00002 ≈ 0.004 N·m/rad ≈ 0 → 比手臂更彻底没劲。
  这解释了 handoff 早前"target 对但手指不跟、提 stiffness/effort 没用、关重力就正常"的全部现象。
- force-drive bake 已把 22 个手指关节一并改成 force，机制上应一并解决。
- **但两点未结（下次会话优先做）**：
  1. **手指闭合还没专门复测**（force USD 下发 `right_hand=[0.8]×6` 看手指跟不跟、稳不稳）。
  2. **手部 stiffness=200 / effort=50 是 acceleration 坏掉时代为硬凑调的强参数**，force 语义下对 13g 手指
     严重过强（200 N·m/rad），很可能手指会猛闭合/抖，**需要往回调到 force 下合理的一组**，再复测。

### 本轮新增/改动（本次提交）

```
+  ubt_sim/assets/robots/walker_c1/walker_c1_force_drive.usd   force-drive USD（15.8MB，修复核心）
M  ubt_sim/source/ubt_sim/devices/walker_c1/config.py          USD_PATH 指向 force USD
M  ubt_sim/teleoperation/control/walker_c1/constants.py        准备姿势肩外展 ±0.30
M  ubt_sim/scripts/collect_walker_c1_pick_place.py             READY_*ARM 同步
M  C1_HANDOFF.md                                               本节
```

### 更新后的下一步顺序

```
1.【下次先做】复测手指闭合（force USD）；若过猛/抖，把手部 stiffness/effort 从 200/50 回调到 force 合理值。
2. 回 M2：手臂能 hold 了，做"够到球"的 IK/波点，抓球→移到目标→成功才存 HDF5。
   （注意：目视 R_palm 离球还有 ~14cm，主要 Y 向 10cm，需要 reach 逻辑把手对准球。）
3. M3：批量随机化刷数据。
4. 收尾整理工作区。
```

## 2026-07-15 Update：M2 完成——真实物理抓放全流程 SUCCESS（正统抓法，非运气）

本节是最新状态。当天经历了 ~17 轮抓取调试（中途曾误报"完成"两次——deterministic
的运气 SUCCESS 在随机位置下 0/3），最终用**可解释的正统抓法**跑通：

```
下探: 苹果全程零扰动；提起: HELD (z 0.924->1.027)；放置: 离盘心 5.2cm SUCCESS。
```

### 制胜配方（collect_walker_c1_pick_place.py 当前实现）

```
1. 软手部增益 stiffness 10 / damping 1 / effort 2（近 URDF 原厂 force 语义值）。
   调试期 200/20/50 是接触弹射的元凶：空载跟踪正常 ≠ 增益正确。
2. 转腕 90°（wrist_roll = ready-1.57）掌心朝下，之后腕/肘yaw 冻结
   （IK joint_subset=(0,1,2,3) 只用肩3+肘pitch 做位置）。
3. 五指微张 [0.2]*6 当笼栅（掌下抓时收指反而把指节卷到罩腔正中央挡路——
   小指是全天惯犯，每轮首触都是 R_little_ip）。
4. 闭环罩口对准：servo_mouth_xy —— xy 误差用"罩口实时中心"（四指尖均值与
   拇指尖的中点）到球心度量，z 走 grasp center；消灭一切手系偏置盲猜。
5. 两段进场：高空(+0.22)对位 → 垂直降(+0.12) → 慢下探(ball+0.02, max_dq 0.006，
   靠接触自然停住即可，不必到底)。
6. 合围 [0.7,0.85,0.8,0.8,0.8,0.8] 一步完成 + 40 步稳定夹持再动臂。
7. 提升/搬运/下放全程 joint_subset 冻腕 + 慢速；放盘前先降到贴盘 1cm 再张手。
8. 球 r=0.027 m=0.1kg：实测这只手口袋有效孔径 5-6cm，7cm 球几何上装不下
   （满力闭合手图显示无任何指节能触到 7cm 球面）。
```

### IK/采集框架关键实现点

```
- 逐步 DLS（lambda=0.1, dq<=0.01 rad/步），palm jacobian；固定基座 jacobian 行号
  = body_idx-1。
- ★IK 增量必须累积在【命令】上（cmd_state），不能是 实测+dq：位置控制器有重力
  滞后，后者永远差 4-5cm 收敛不到。防饱和：命令超前实测 <=0.2 rad + 软限位内。
- 误差 <1.2cm 提前退出阶段；HOME->READY 用 100 步 ramp（一步跳变会甩臂扫到桌上物体）。
- --debug_watch：每 5 步打印 苹果位置+速度+最近机器人连杆+距离——本轮定案全靠它；
  _print_hand_map 打印 11 个手链节相对 grasp center 的坐标（口袋几何实测）。
- 成功判定：苹果落点距盘心 <=0.085m 且高度在盘面区间；成功才存 HDF5；
  --save_on_failure 调试用；--randomize 苹果初始位置随机（M3 主开关）。
```

### 抓取调试账本（17 轮的教训，按发现顺序）

```
1. 相机 rot=(1,0,0,0)+ros = 朝天花板拍（M1 就存在，只验过尺寸没验内容）。
   修正 rot=(0.40558,-0.57923,0.57923,-0.40558)=朝前+下俯20°，ready 低头 0.50。
2. 角阻尼 2.0 防滚 → 球不能滚就变"捏西瓜子"挤压弹射，更糟。回退。真正防滚 = 贴盘轻放。
3. 手部增益 200/20/50：接触时上百牛，球被弹飞 2-5 米。回调 10/1/2。
4. 张开指尖在斜线进场路径上扫飞球（位置 IK 不控姿态，腕自然低头）→ 两段进场。
5. 侧向口袋（掌心朝+y）：装 7cm 球孔径不够；口袋结构=指尖帘+高位拇指，
   拇指闭合弧根本够不到桌上球的高度 → 死路，转腕掌下才是正解。
6. 每轮首触都是小指(R_little_ip) → 掌下抓时五指微张，不收指。
7. deterministic 同种子成功 ≠ 可靠：必须 --randomize 多局验证。
```

### 本轮文件改动（待提交清单见 git）

```
M  ubt_sim/source/ubt_sim/task/walker_c1_parlor/walker_c1_parlor_env_cfg.py
   隐形桌面碰撞板/盘碰撞盘/刚体苹果(r0.027,100g)；两张桌子问题修复
M  ubt_sim/config/walker_c1/parlor.yaml   场景指向 scene_v2_c1.usda + 相机 rot 修正
M  ubt_sim/scripts/collect_walker_c1_pick_place.py   M2 完整抓放（上面的配方）
M  ubt_sim/scripts/probe_walker_c1_workspace.py      ready 同步
M  ubt_sim/teleoperation/control/walker_c1/constants.py  head_pitch 0.50
M  ubt_sim/source/ubt_sim/devices/walker_c1/config.py    手部增益 10/1/2
+  ubt_sim/assets/local_scenes/tiangong_parlor/scene_v2_c1.usda（gitignore 内：
   6 行 usda，subLayers 引 scene_v2.usd + over "apple" active=false，机器丢失按此重建）
```

### 随机化收尾（31e6a5d push 之后的迭代）

固定位置 SUCCESS 后，--randomize 连续 0/3 了四批，逐层修掉：

```
1. 冻腕后 IK 只剩肩3+肘pitch 4 关节，随机点（尤其 +x 远侧）出了可达域，
   下探停在差 5cm 处 -> 解冻 elbow_yaw（subset=(0,1,2,3,4)）。
2. 解冻后 yaw 自由漂移 -> 抓取时刻手朝向和成功局不一致，闭手落空 ->
   ★零空间偏置：DLS 加 null-space 项把 elbow_yaw 持续拉回转腕后的参考值
   （位置任务不受影响）。_ik_arm_step(null_ref={4: roll_arm[4]})。
3. 下探高度敏感 ±1cm：目标=球+0.04（加早退容差 ~1cm 恰落在成功高度 球+0.05）。
   低了指尖插桌被压住卷不动，高了笼子合在球顶上方。
4. 随机范围对齐可达域：x∈[-0.03,+0.01]、y∈[-0.05,+0.01]（+y 侧仍吃紧故收窄）。
```

结果：随机位置首次 SUCCESS（1/3，第 3 局 HELD+落点 7cm）。

### 随机化成功率现状（调参已到收益上限，2026-07-15 晚）

后续又加了三层（本 commit）：

```
1. 抓取校验+重试：提起后查苹果 z；没抓住 -> 张手回 [0.2] 预合拢，对苹果当前位置
   重来（最多 3 次）；苹果掉桌则提前中止。重试有实测兑现（一局第2次抓住并成功）。
   注意：失败大多是"位置确定性"的（同一点重试同样失败），重试主要救随机扰动型失败。
2. 苹果 r=0.027 -> 0.022（4.4cm 李子级）：罩笼孔径 5-6cm，5.4cm 球只剩毫米级余量，
   出生点差 6mm 结果就翻转；4.4cm 给 ±0.8cm 余量，和伺服精度匹配。成功率 1/5 -> 2/5。
3. 下探收敛容差 1.2cm -> 0.8cm；合围加深 [0.7,0.9,0.95x4]（副作用：抓空时更容易
   把苹果打飞出桌，中止该局——可接受，反正失败局不存）。
```

**当前测得成功率 ~30%（3/10，批间波动 20-40% 属噪声）。盲罩式抓取（位置 IK 无
姿态控制）已到上限，不要再花 50 分钟/轮磨 cm 级参数。**

### 下一步（按性价比）

```
1. 直接用：30-40% 成功率对刷数据可用（失败局不存，只费仿真时间）。
   一次启动多刷：--episodes 25 约 4 小时 -> ~8-10 条轨迹。
2. 提吞吐：精简阶段步数（很多是调试余量，可砍 30-40%）；num_envs>1 并行采集
   （标准做法，采集脚本要改批量版，半天活）。
3. 提成功率的正道：6D IK（位置+姿态同控，palm 姿态显式指定）代替
   "位置IK+转腕+零空间"的拼积木——这是把成功率推到 80%+ 的正确工程路线。
4. 抽查已存 HDF5（dataset/walker_c1/ 下已有多条成功轨迹）。
5. 归位→抓取语义已内嵌（每局 ramp 到 TASK_RESET_BODY_POSE 开始、结束回同一姿势），
   用户已 GUI 确认；姿势权威出处如需统一可抽 JSON。
6. 左腿碰撞小尾巴、真机 joint order 校对：老未结项不变。
```

## 2026-07-17 Update：优先级转向 ROS 工具链 + C1 bridge 完成并验证

**用户明确新目标：不是刷数据，是工具链——同一套控制代码通过 ROS SDK 话题，仿真和
真机行为一致（切 ROS_DOMAIN_ID 146↔0）。** in-process 采集器保留作调试用。

### 已完成（commit 820faee）

```
+ teleoperation/bridges/walker_c1/walker_c1_ros2_zmq_bridge.py + yaml
  SDK 话题面：/mc/sdk/robot_command|robot_state、/mc/{left,right}_hand/command|
  joint_states、/sensor/camera/head/color|depth/raw、/sim/cmd_reset（仿真 only）。
  身体命令按名映射（含 elbow_roll/elbow_pitch 双别名兼容——SDK 文档 vs URDF 命名
  歧义两头都认）；手部 6D SDK 名映射到 11 关节仿真手（复用 action_process 逻辑）。
M scripts/start_sim.sh  C1 分支启动 bridge；RMW 默认 rmw_fastrtps_cpp。
```

### 端到端验证（全部实测通过）

```
ros2 pub RobotCommand head_pitch 0.3      -> 仿真 head_pitch 0.301 ✓
ros2 pub RobotCommand R_elbow_roll -0.8   -> 仿真 R_elbow_pitch -0.788 ✓（别名兼容）
ros2 pub JointCommand right_hand 0.5x6    -> 仿真 SDK 手关节 0.495 ✓（与身体并发）
/mc/sdk/robot_state / hand joint_states   -> 数据流 ✓
/sensor/camera/head/color/raw             -> 25.4 Hz ✓（C++ image bridge）
```

### 踩坑记录（重要）

```
1. 容器没装 colcon —— run.sh init 的"Build Walker SDK ROS2 messages"一直静默失败。
   已 pip 装 colcon-common-extensions 并编译到 /opt/ubt_sim/walker_sdk_ros2_msgs。
   （注意 colcon 是装在容器里的，容器重建后要重装。）
2. S2 分支的 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 在本容器会让 rclpy 启动即退
   （只装了 fastrtps）。C1 分支默认 rmw_fastrtps_cpp。
3. bridge cmd socket 不能抄 S2 的 SNDHWM=1：身体+手是两条背靠背消息，1 深队列
   会系统性丢掉先发的一条（手部命令 100% 被丢）。C1 控制器 advance() 是
   "排干全部+合并"语义，SNDHWM=16 正确。
4. 杀 start_sim.sh 时 kit python 进程可能残留占住 5655/5656 端口
   （Address already in use）——pgrep -af sim_runner 精确清理。
```

### 启动方式

```bash
# 全栈（仿真+bridge），仿真域 146：
docker exec walker-c1-ubt-sim bash -lc "cd /ubt_sim && \
  UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0 ROS_DOMAIN_ID=146 \
  bash scripts/start_sim.sh --headless --device cpu --step_hz 30"
# ROS 侧环境：
source /opt/ros/humble/setup.bash
source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
export ROS_DOMAIN_ID=146   # 真机=0
```

### 下一步

```
1. 抓放控制移植成 rclpy 脚本（teleoperation/control/walker_c1/，Py3.10，只用 SDK 话题）：
   - IK 不能再用仿真雅可比 -> URDF + ikpy（参照天工 robot_controller.py）；
   - 苹果位置作参数/配置（仿真=已知出生点；真机=将来接感知）；
   - reset.py 已有 ROS 骨架，可从"归位"开始验证。
2. 上真机第一件事不变：dump /mc/sdk/robot_state 的 joint_states.name，
   核对肘部命名（bridge 已双兼容，但控制脚本侧要以真机名为准）。
3. 左腿碰撞小尾巴等老未结项不变。
```

## 2026-07-19 Update：★ROS 抓放总根因定案——仿真时间 vs 墙钟不同步（已修复+判决实验验证）

本节是全项目最重要的一条结论。ROS 版抓放此前连败十几轮（增益/摩擦/姿态/偏移怎么调
都在"打飞球↔握不住"间震荡），最终用"回放冠军轨迹"判决实验定案：

### 根因

```
本机 CPU 物理仅 ~28 步/秒（实测），每步 0.01s 仿真时间
=> 仿真时间流速 = 0.28x 真实时间（慢 3.6 倍）
=> ROS 侧按墙钟计时的一切动作，在仿真世界里被加速 3.6 倍执行
=> "2.5 秒缓降"在物理引擎里是 0.7 秒的快落，接触必炸
in-process 采集器从不受此影响：命令与 env.step() 逐步同锁。
```

### 判决实验（replay_trajectory.py，commit f5887e7）

```
同一条 in-process 冠军轨迹（1784105817）通过 ROS SDK 话题重放：
  按墙钟 45ms/帧发   -> 球被打飞下桌（复现 ROS 版全部症状）
  按物理步 3步/帧发  -> 抓→提→运→放全成功，落点距盘心 4.8cm ✓✓
```

### 修复设施（已提交 f5887e7）

```
- controller.py: ZMQ 状态带 sim_step 物理步计数器
- bridge: /sim/object_state JSON 透传 sim_step
- robot_controller.py: wait_sim_steps(k)——按物理步等待；无计数器时自动退化
  为墙钟等待（真机语义：仿真时间==真实时间，同代码两端通用）
- replay_trajectory.py: 数据集 HDF5 轨迹回放器（sim/真机通用，IL 演示重放工具）
- move_right_arm_joints: ramp 改为每 3 物理步发一条命令（=冠军轨迹录制节奏），
  duration 语义变为"仿真秒"
- 物理配置已回退 in-process 已证值（手 10/1/2 + force_drive.usd）；
  调试期的摩擦烘焙（bake_walker_c1_hand_friction_usd.py + grip.usd）保留备用未启用
```

### 教训（重要，写给后来者）

```
1. 慢于实时的仿真里，跨进程控制必须按仿真步对齐，不能按墙钟。这是所有
   "仿真里 ROS 控制"项目的通用陷阱。
2. 调试期间的增益/摩擦/姿态/偏移十几轮调参全是给这个根因打的错误补丁——
   当"多方向调参都在两种失败模式间震荡"时，应怀疑存在未建模的系统级因素。
3. 判决实验（回放已知成功的轨迹）比继续调参高效得多，应更早使用。
4. 感谢用户两次关键质疑："如果是摩擦为什么 in-process 能成功"（排除摩擦）、
   "频率可能有关系"（直指根因）。
```

### 当前状态与下一步

```
- ROS 工具链完整：bridge + ikpy 控制 + 闭环对准 + 安全下限 + 回放器 + 步同步。
- pick_place_controller 正在切换到全步同步节奏（每局墙钟时间变长 3.6 倍属预期，
  是正确性的代价；物理跑得快时自动缩短）。
- 待办：步同步版抓放成功率验证 -> --randomize 验证 -> step_hz 100 吞吐测试
  （CPU 已近饱和，预期增益有限）-> 更新采集链路。
- 真机 joint order 核对、左腿小尾巴等老未结项不变。
```

## 2026-07-19 Update（晚）：验收标准确定 + 主路线切换为示教再现

### 用户验收标准（重要）

```
"只需要保证苹果位置固定的时候能成功率比较高，随机的是锦上添花。"
=> 固定位置高成功率 = 硬指标；随机位置 = 加分项，不阻塞验收。
用户已授权完全自主工作（不删成果、保护仓库、勤记录为红线）。
```

### 主路线：示教再现（teach-and-repeat）

```
新增 teleoperation/control/walker_c1/pick_place_replay.py：
  go_ready -> 苹果摆到示教位置 (8.207,5.877) -> 按物理步重放冠军轨迹
  (dataset/walker_c1/1784105817) -> 成功判定 -> 多局统计。
判决实验已证此路端到端 100% 可行（落点距盘心 4.8cm）。
这就是工业标准的固定位置抓放做法，也直接对应真机示教流程。
```

### 示教再现调试进展（自主工作期间实时记录，含两次假设纠错——诚实过程）

```
- 判决性发现②：新启动仿真栈上回放稳定成功（裸回放+包装版 back-to-back 双成功,
  落点 1.0cm）,运行数小时的老栈上失败。
- 【已证伪①】最初怀疑"reset_sim（reset_scene_to_default）复位会污染物理状态"，
  加了显式复位调用。复位版 5 局=1 成 4 败（ep1 成功,ep2 起确定性同败）。
  关掉显式复位后重测：全程不调用 reset_sim，结果依然是 ep1-2 成功、ep3 起败光。
  => 与是否调用复位无关，证伪。
- 【已证伪②】"体内摇晃未散尽"（关节残余速度未衰减导致下一局失真）。加了速度
  探针诊断，实测速度读数在多个"1s/3s/5s 静置"检查点之间逐位数字冻结不变——
  说明探针本身有问题（未验证真实衰减），但顺带检查腰/头/腿角度也全程稳定在
  0 附近（<0.02 rad），排除了"腰部在抓取冲击下悄悄跑偏"这个更具体的猜测。
- 【重大修正,原结论过于乐观】原以为"全新重启的仿真栈上第 1 局必成功"（早期
  3-4 次巧合都成功导致误判为确定性规律）。扩大到 6 次独立全新栈试验后，
  真实成功率约 4/6 ≈ 65%（不是 100%！也测到过全新栈 ep1 直接失败的情况）。
  但"一旦某局在某进程上失败，同进程后续几乎必败"这条规律在所有多局测试里
  稳定重现，没被推翻。
  => 根本内部机制（大概率是 PhysX 接触缓存或求解器状态在某次失败性剧烈接触后
  进入了某种非典型/退化分支，且不会自愈）仍未查到底，留作后续可选深挖项。
- 【当前可靠方案，已按新数据修正】run_c1_teach_and_repeat_batch.sh：
  单次全新栈尝试成功率~65%，若失败则丢弃该进程、整套重启再试（进程内重试
  已证无效）。按此重试直到命中，理论 P(3 次内成功)≈96%。
  用法（N=目标成功局数，第二参数=每局最多重试次数,默认5）：
    docker exec walker-c1-ubt-sim bash /ubt_sim/scripts/run_c1_teach_and_repeat_batch.sh 3 5
  代价：~5 分钟启动/次尝试,平均约 1.5 次尝试换 1 个成功局。
```

### 支线（IK 编排版）状态：实验性，暂停打磨

```
步同步修好时序后，IK 版残余问题 = mode-Z 的 yaw 自由度让指笼落位旋转不定
（球有时压在无名指/小指下而非拇指对握区）。尝试"冠军关节构型作 IK 种子"
反而劣化（种子与路点目标体系不自洽，对准环报 447mm 级误差、手臂被拉飞）。
代码全部保留（pick_place_controller.py），标记实验性；随机位置需求提上来时
再继续，或改走"冠军轨迹+笛卡尔偏移扭曲"的路线。
```

### 场景改造（walker_c1_parlor_env_cfg.py + scene_v2_c1.usda + parlor.yaml）

```
- 场景装饰桌 /World/table：x[8.144,8.744] y[5.483,6.683] 桌面 z=0.897（视觉专用，无碰撞）。
- TableTopCollider：隐形薄板 (0.60,1.20,0.06)，顶面与视觉桌面齐平 z=0.897。
- 场景盘子 /World/plate：中心 (8.374,6.046)，半径~0.10，盘沿 z=0.931（视觉专用）。
- PlateCollider：隐形圆盘 r=0.085 h=0.05，顶面 z=0.925（放置面+成功判定区）。
- Object：红色刚体苹果 r=0.035、100g、摩擦 1.2，初始 (8.21,5.90,0.934)（桌面右手区）。
- 场景装饰苹果 /World/apple 用覆盖层 deactivate（避免画面里两个苹果干扰 IL）：
  ubt_sim/assets/local_scenes/tiangong_parlor/scene_v2_c1.usda
  （6 行 usda：subLayers 引 scene_v2.usd + over "apple" active=false；
   在 gitignore 的 local_scenes 里，机器丢了按此描述重建即可。）
- parlor.yaml scene.usd_path 已指向 scene_v2_c1.usda。
```

### 相机朝向 bug（重要教训）

```
parlor.yaml 头部相机 offset rot 原来是 (1,0,0,0) + convention=ros
= 沿 head_pitch_link +Z 看 = 朝天花板！（画面里的灰色大圆角块=顶灯）
M1 当时只验证了"能解码出 480x640"，没验证内容。
修正值 rot=(0.40558,-0.57923,0.57923,-0.40558)（朝前+额外下俯 20°），
加上 ready 低头 0.50 rad，合计约 49° 俯视桌面。
验证帧：桌面/粉盘/红苹果/双手都在画面内，carry 阶段能看到黄色右臂抓着苹果。
READY_HEAD pitch 0.35->0.50 已同步 collect 脚本 / workspace probe / constants.py。
```

### IK 抓放实现（collect_walker_c1_pick_place.py）

```
- 伺服点"抓取中心"= R_thumb_mpp/R_index_ip/R_middle_ip 三链节中点。
- 每步阻尼最小二乘（lambda=0.1，dq<=0.01 rad/步），用 R_palm 的 jacobian。
- 关键坑：IK 增量必须累积在【命令】上（cmd_state["right_arm"]），不能加在实测角上——
  位置控制器有重力滞后，命令=实测+dq 永远差 4~5cm 收敛不到，第一次试跑就因此
  把苹果拨飞到地上。加防饱和：命令超前实测<=0.20 rad + 关节软限位内。
- 误差<1.2cm 提前退出阶段；下探前手指预合拢 0.3 减少拨飞。
- 阶段：settle60 -> hover240(苹果上方12cm) -> 预合拢25 -> descend160 -> 闭手0.85x60
  -> lift100(+15cm) -> carry180(盘上方) -> 张手50 -> retreat60 -> 回READY120 -> settle40。
- 成功判定：苹果落点离盘心 <=0.085m 且高度在盘面区间 -> 才存 HDF5。
  --save_on_failure 失败也存（调试）；--randomize 苹果初始位置随机
  （x+[-0.02,0.04], y+[-0.06,0.06]），M3 直接用。
- 典型成功日志：hover err 0.010 / descend err 0.032（指笼罩住苹果）/
  lift 后苹果 z 0.932->1.063 HELD / 落点离盘心 0.073 SUCCESS / 280 帧。
```

### 本轮改动文件

```
M  ubt_sim/source/ubt_sim/task/walker_c1_parlor/walker_c1_parlor_env_cfg.py  隐形碰撞体+苹果
M  ubt_sim/config/walker_c1/parlor.yaml        场景指向 scene_v2_c1.usda + 相机 rot 修正
M  ubt_sim/scripts/collect_walker_c1_pick_place.py  M2 IK 抓放 + 成功判定 + randomize
M  ubt_sim/scripts/probe_walker_c1_workspace.py     ready 同步（肩外展/低头）
M  ubt_sim/teleoperation/control/walker_c1/constants.py  head_pitch 0.50
+  ubt_sim/assets/local_scenes/tiangong_parlor/scene_v2_c1.usda（gitignore 内，见上）
```

### 下一步

```
1. M3：批量 --randomize 刷数据（验证成功率后放大 episodes 数）。
2. 数据规模上来后抽查 HDF5（帧数/图像内容/obs-action 对齐）。
3. （可选）左腿碰撞小尾巴仍在（纯视觉）；真机 joint order 校对仍是回真机前的第一件事。
```


