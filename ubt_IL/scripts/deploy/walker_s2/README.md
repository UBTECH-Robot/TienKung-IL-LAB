# Walker S2 部署（Rollout）

容器内执行，前置条件：Bridge2 已由容器 entrypoint 自动启动。

部署脚本：[rollout.sh](rollout.sh)

### 19D 仿真模型 -> 19D PGC 夹爪真机

```bash
cd /ubt_IL/lerobot

POLICY_PATH=/ubt_IL/model/Walker_S2_sim_act/checkpoints/last/pretrained_model \
ROBOT_MODEL=walker_s2_gripper_19d \
FPS=15 \
DURATION=30 \
ALLOW_DIM_ONLY_POLICY=1 \
bash /ubt_IL/scripts/deploy/walker_s2/rollout.sh
```

### 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `POLICY_PATH` | **必填，无默认值** | checkpoint 目录（含 `config.json`） |
| `ROBOT_MODEL` | `walker_s2_v4_hand_31d` | 机器人配置文件名前缀（不含 `.json`） |
| `ROBOT_CONFIG` | `configs/<ROBOT_MODEL>.json` | 机器人配置文件完整路径 |
| `STRATEGY` | `base` | rollout 策略类型 |
| `FPS` | `15` | 控制频率 |
| `DURATION` | `30` | 运行时长（秒） |
| `TASK` | `walker s2 rollout` | 任务描述 |
| `ALLOW_DIM_ONLY_POLICY` | `0` | 策略无 action names 时允许仅按维度匹配（设 `1` 开启） |

### 机器人配置

| 配置文件 | 维度 | 末端执行器 |
|----------|------|-----------|
| [walker_s2_gripper_19d.json](configs/walker_s2_gripper_19d.json) | 19D（17 body + 2 gripper） | PGC 1DOF 夹爪 |
| [walker_s2_v4_hand_31d.json](configs/walker_s2_v4_hand_31d.json) | 31D（17 body + 14 hands） | V4 灵巧手 7DOF |

### 安全预检

脚本启动后自动执行 action 维度匹配检查，不通过则拒绝部署：

1. 读取 robot config 的 `action_order`，计算期望维度
2. 读取 policy `config.json` 的 `output_features.action.shape`，获取实际维度
3. 维度不匹配 -> 报错退出
4. 维度匹配但有 action names -> 校验 names 顺序一致性
5. 维度匹配但无 action names -> 需 `ALLOW_DIM_ONLY_POLICY=1` 才放行

### 安全参数

配置文件中 `safety` 段：

- `max_relative_target: 0.02` - 单步相对目标限幅
- `disable_torque_on_disconnect: true` - 断开连接自动卸力
