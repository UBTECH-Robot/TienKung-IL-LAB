# C1 Joint Map for ubt_sim

本文档记录从 C1 新整机 URDF 中解析出来的关节分组，后续用于编写 `ubt_sim/source/ubt_sim/devices/c1/config.py`。

来源 URDF：

```text
ubt_sim/assets/robots/c1/walker_astron_v2_hand_v3.urdf
```

当前 URDF 统计：

```text
movable joints = 53
fixed joints   = 16
total joints   = 69
```

分组统计：

```text
waist       3
head        2
left_arm    7
right_arm   7
left_hand   11
right_hand  11
left_leg    6
right_leg   6
```

## Arm Joints

### Left Arm

```python
C1_LEFT_ARM_JOINTS = [
    "L_shoulder_pitch_joint",
    "L_shoulder_roll_joint",
    "L_shoulder_yaw_joint",
    "L_elbow_pitch_joint",
    "L_elbow_yaw_joint",
    "L_wrist_pitch_joint",
    "L_wrist_roll_joint",
]
```

### Right Arm

```python
C1_RIGHT_ARM_JOINTS = [
    "R_shoulder_pitch_joint",
    "R_shoulder_roll_joint",
    "R_shoulder_yaw_joint",
    "R_elbow_pitch_joint",
    "R_elbow_yaw_joint",
    "R_wrist_pitch_joint",
    "R_wrist_roll_joint",
]
```

## Hand Joints

C1 新三代手每只手有 11 个 `revolute` 手指关节。Tiankung 每只手是 12 个手指关节，并且有 mimic 映射，所以 C1 不能直接复用 Tiankung 的 hand action mapping。

### Left Hand

```python
C1_LEFT_HAND_JOINTS = [
    "L_thumb_cmp_joint",
    "L_thumb_mpp_joint",
    "L_thumb_ip_joint",
    "L_index_mpp_joint",
    "L_index_ip_joint",
    "L_middle_mpp_joint",
    "L_middle_ip_joint",
    "L_ring_mpp_joint",
    "L_ring_ip_joint",
    "L_little_mpp_joint",
    "L_little_ip_joint",
]
```

### Right Hand

```python
C1_RIGHT_HAND_JOINTS = [
    "R_thumb_cmp_joint",
    "R_thumb_mpp_joint",
    "R_thumb_ip_joint",
    "R_index_mpp_joint",
    "R_index_ip_joint",
    "R_middle_mpp_joint",
    "R_middle_ip_joint",
    "R_ring_mpp_joint",
    "R_ring_ip_joint",
    "R_little_mpp_joint",
    "R_little_ip_joint",
]
```

## Head and Waist Joints

### Head

```python
C1_HEAD_JOINTS = [
    "head_yaw_joint",
    "head_pitch_joint",
]
```

### Waist

```python
C1_WAIST_JOINTS = [
    "waist_yaw_joint",
    "waist_pitch_joint",
    "waist_roll_joint",
]
```

## Leg Joints

### Left Leg

```python
C1_LEFT_LEG_JOINTS = [
    "L_hip_pitch_joint",
    "L_hip_roll_joint",
    "L_hip_yaw_joint",
    "L_knee_pitch_joint",
    "L_ankle_pitch_joint",
    "L_ankle_roll_joint",
]
```

### Right Leg

```python
C1_RIGHT_LEG_JOINTS = [
    "R_hip_pitch_joint",
    "R_hip_roll_joint",
    "R_hip_yaw_joint",
    "R_knee_pitch_joint",
    "R_ankle_pitch_joint",
    "R_ankle_roll_joint",
]
```

## Joint Limits Summary

### Waist

| Joint | Lower | Upper | Effort | Velocity |
|---|---:|---:|---:|---:|
| `waist_yaw_joint` | -2.792 | 2.792 | 63 | 10.47 |
| `waist_pitch_joint` | -0.017 | 1.221 | 165 | 8.9 |
| `waist_roll_joint` | -0.75 | 0.75 | 110 | 10.47 |

### Head

| Joint | Lower | Upper | Effort | Velocity |
|---|---:|---:|---:|---:|
| `head_yaw_joint` | -1.658 | 1.658 | 4.5 | 5.24 |
| `head_pitch_joint` | -0.523 | 0.785 | 4.5 | 5.24 |

### Hands

| Joint Pattern | Lower | Upper | Effort | Velocity |
|---|---:|---:|---:|---:|
| `L_thumb_cmp_joint` | -0.96 | 0 | 1.35 | 2.09 |
| `R_thumb_cmp_joint` | 0 | 0.96 | 1.35 | 2.09 |
| `L/R_thumb_mpp_joint` | 0 | 1.04 | 1.35 | 2.09 |
| `L/R_thumb_ip_joint` | 0 | 1.05 | 1.35 | 2.09 |
| `L/R_*_mpp_joint` for index/middle/ring/little | 0 | 1.46 | 1.35 | 2.09 |
| `L/R_*_ip_joint` for index/middle/ring/little | 0 | 1.62 | 1.35 | 2.09 |

## Notes for C1 Config

1. `C1_HOME_POSE` 可以先全部设为 `0.0`，但腿部如果要站立，后续可能需要参考 MJCF 或 SDK 的默认站姿。
2. 仿真初期可以只控制双臂、双手、头和腰，腿部先用 home pose 固定。
3. C1 手没有沿用 Tiankung 的 mimic 配置。后续 `devices/c1/action_process.py` 应该先按 22 个手指关节独立映射。
4. 当前整机 URDF 还有两个缺失 mesh：`L_sixforce_link.STL` 和 `R_sixforce_link.STL`。它们影响腕部外观/碰撞，不影响 joint map。
5. Isaac Lab 运行时通常加载 USD，所以后续还需要把 `walker_astron_v2_hand_v3.urdf` 转成 C1 USD。
