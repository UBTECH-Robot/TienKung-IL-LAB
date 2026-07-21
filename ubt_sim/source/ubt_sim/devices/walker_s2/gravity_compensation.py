"""Walker S2 重力补偿前馈模块。

使用 pinocchio RNEA 从 URDF 计算关节空间重力矩，转换为等效位置偏移，
叠加到 ImplicitActuator 的位置指令上，消除承受重力负载关节的稳态跟踪误差。

原理：
  τ = stiffness × (θ_target_compensated − θ_actual)
    = stiffness × (θ_desired + τ_g/k − θ_actual)
    = stiffness × (θ_desired − θ_actual) + τ_g
  稳态时 τ = 0 → θ_actual = θ_desired  ✓

Isaac Sim 兼容注意事项：
  - pinocchio 采用延迟加载，避免启动阶段 libassimp 符号冲突
  - ctypes.CDLL 预加载 cmeel 自带的 libassimp（RTLD_GLOBAL 模式）
  - 因 RTLD_GLOBAL 污染 pybind11 类型注册导致 std::vector<string> 转换
    失败，改用硬编码的关节索引表（URDF 固定，pinocchio 编号确定）

参考实现：ubt_IL/lerobot/.../unitree_g1/g1_kinematics.py:solve_tau()
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np

from ubt_sim.utils.constant import ASSETS_ROOT

# ============================================================================
# 手臂关节刚度（与 config.py 中 WALKER_S2_ARM_STIFFNESS 保持一致）
# ============================================================================

_ARM_STIFFNESS = {
    "L_shoulder_pitch_joint": 600,
    "L_shoulder_roll_joint":  500,
    "L_shoulder_yaw_joint":   600,
    "L_elbow_roll_joint":     500,
    "L_elbow_yaw_joint":      600,
    "L_wrist_pitch_joint":    600,
    "L_wrist_roll_joint":     600,
    "R_shoulder_pitch_joint": 600,
    "R_shoulder_roll_joint":  500,
    "R_shoulder_yaw_joint":   600,
    "R_elbow_roll_joint":     500,
    "R_elbow_yaw_joint":      600,
    "R_wrist_pitch_joint":    600,
    "R_wrist_roll_joint":     600,
}

_ARM_JOINT_NAMES = list(_ARM_STIFFNESS.keys())

# ============================================================================
# pinocchio 关节索引表（硬编码，URDF 固定 → pinocchio 编号确定）
#
# 由 `pin.buildModelFromUrdf("s2.urdf")` 的深度优先遍历顺序决定：
#   腿(12) → 腰(2) → 左臂(7) → 右臂(7) → 头(2) = 30 DOFs
#
# 注意：不能通过 _model.names 迭代获取（Isaac Sim 下 pybind11
#       std::vector<string> 类型转换失败），改为硬编码此表。
# ============================================================================

# 关节名 → pinocchio q-index / v-index
_NAME_TO_Q_IDX: dict[str, int] = {
    # 左腿 (q=0-5)
    "L_hip_roll_joint":     0,
    "L_hip_yaw_joint":      1,
    "L_hip_pitch_joint":    2,
    "L_knee_pitch_joint":   3,
    "L_ankle_pitch_joint":  4,
    "L_ankle_roll_joint":   5,
    # 右腿 (q=6-11)
    "R_hip_roll_joint":     6,
    "R_hip_yaw_joint":      7,
    "R_hip_pitch_joint":    8,
    "R_knee_pitch_joint":   9,
    "R_ankle_pitch_joint":  10,
    "R_ankle_roll_joint":   11,
    # 腰 (q=12-13)
    "waist_yaw_joint":      12,
    "waist_pitch_joint":    13,
    # 左臂 (q=14-20)
    "L_shoulder_pitch_joint": 14,
    "L_shoulder_roll_joint":  15,
    "L_shoulder_yaw_joint":   16,
    "L_elbow_roll_joint":     17,
    "L_elbow_yaw_joint":      18,
    "L_wrist_pitch_joint":    19,
    "L_wrist_roll_joint":     20,
    # 右臂 (q=21-27)
    "R_shoulder_pitch_joint": 21,
    "R_shoulder_roll_joint":  22,
    "R_shoulder_yaw_joint":   23,
    "R_elbow_roll_joint":     24,
    "R_elbow_yaw_joint":      25,
    "R_wrist_pitch_joint":    26,
    "R_wrist_roll_joint":     27,
    # 头 (q=28-29)
    "head_yaw_joint":       28,
    "head_pitch_joint":     29,
}
_N_Q = 30  # pinocchio model.nq

# v-index = q-index（所有关节均为 revolute，nq=nv=1 per joint）
_NAME_TO_V_IDX = _NAME_TO_Q_IDX

# ============================================================================
# 延迟加载
# ============================================================================

_URDF_PATH = str(Path(ASSETS_ROOT) / "robots" / "walker_s2" / "s2.urdf")
_pin = None      # pinocchio module
_model = None    # pin.Model
_data = None     # pin.Data
_initialized = False


def _ensure_pinocchio() -> None:
    """延迟初始化 pinocchio（首次调用时触发）。

    1. ctypes.CDLL 预加载 cmeel libassimp（RTLD_GLOBAL），
       使 hpp-fcl 能解析 Assimp 符号。
    2. import pinocchio + buildModelFromUrdf
    3. 验证硬编码索引表与实际模型一致。
    """
    global _pin, _model, _data, _initialized

    if _initialized:
        return

    import ctypes

    _assimp_path = (
        "/isaac-sim/kit/python/lib/python3.11/site-packages/"
        "cmeel.prefix/lib/libassimp.so.5"
    )
    try:
        ctypes.CDLL(_assimp_path, mode=ctypes.RTLD_GLOBAL)
    except OSError:
        pass

    import pinocchio
    _pin = pinocchio
    _model = _pin.buildModelFromUrdf(_URDF_PATH)
    _data = _model.createData()

    # 一致性检查：硬编码表必须与 pinocchio 模型匹配
    if _model.nq != _N_Q:
        raise RuntimeError(
            f"Pinocchio model nq={_model.nq} != hardcoded {_N_Q}. "
            f"URDF may have changed; update _N_Q and _NAME_TO_Q_IDX."
        )

    _initialized = True
    print(
        f"[INFO] Walker S2 gravity compensation loaded: "
        f"{len(_ARM_JOINT_NAMES)} arm joints, {_model.nq} URDF DOFs"
    )


# ============================================================================
# 公开 API
# ============================================================================


def compute_gravity_offsets(joint_pos_dict: dict[str, float]) -> dict[str, float]:
    """计算手臂关节的重力补偿位置偏移。

    首次调用时初始化 pinocchio（约 50ms），后续 RNEA 约 0.1ms。

    Args:
        joint_pos_dict: {joint_name: position_rad}，含全关节（腿部影响质心）。

    Returns:
        {arm_joint_name: offset_rad}，offset = τ_gravity / stiffness。
        未识别关节名被忽略。
    """
    _ensure_pinocchio()

    q = np.zeros(_N_Q)
    for name, pos in joint_pos_dict.items():
        q_idx = _NAME_TO_Q_IDX.get(name)
        if q_idx is not None:
            q[q_idx] = float(pos)

    _pin.computeGeneralizedGravity(_model, _data, q)

    # 经验缩放因子：补偿 URDF → USD 转换中的质量/惯量偏差。
    # 实测 L_shoulder_pitch @ -1.0 rad 残留约 0.011 rad（补偿不足 ~40%），
    # 说明 USD 模型的实际重力矩比 URDF 大 ~1.4×，增大因子补偿差额。
    _GRAVITY_SCALE = float(os.environ.get("WALKER_S2_GRAVITY_SCALE", "1.4"))

    offsets: dict[str, float] = {}
    for name in _ARM_JOINT_NAMES:
        v_idx = _NAME_TO_V_IDX[name]
        tau_g = float(_data.g[v_idx]) * _GRAVITY_SCALE
        stiffness = float(_ARM_STIFFNESS[name])
        offsets[name] = tau_g / stiffness

    return offsets
