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
