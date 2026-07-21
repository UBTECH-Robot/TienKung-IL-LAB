"""Pinocchio-based inverse kinematics for Walker S2 dual arms.

Targets use [x, y, z, roll, pitch, yaw] in the Walker S2 URDF base frame.
Units are meters and radians. This module is ROS-independent so it can be
imported and tested with system Python.

Key features:
  - Multi-seed fallback: previous frame → semantic (task_type) → random seeds
  - Hierarchical IK: torso (waist_yaw) → shoulder (3-DOF) → full arm (7-DOF)
  - Cross-task hot-start via semantic task seeds
"""

from __future__ import annotations

from typing import Any

import numpy as np

try:
    import pinocchio as pin
except ImportError as exc:  # pragma: no cover - exercised in runtime envs
    raise ImportError(
        "Walker S2 IK requires Pinocchio in the ROS/system Python environment. "
        "Verify with: /usr/bin/python3 -c 'import pinocchio as pin; print(pin.__version__)'. "
        "If missing, install the package that provides import name 'pinocchio' (often pip package 'pin')."
    ) from exc


# ── 语义任务种子 ──
# 键 = task_type，值 = {"left": [7 angles], "right": [7 angles]}
# 初始仅 pick_table 使用 READY_POSE 的臂关节值，后续由用户补充其他任务种子
_TASK_SEEDS: dict[str, dict[str, list[float]]] = {}


def register_task_seed(task_type: str, left_angles: list[float], right_angles: list[float]) -> None:
    """注册一个语义任务种子配置。

    Args:
        task_type: 任务类型标识（如 "pick_table", "pick_floor"）
        left_angles: 左臂 7 关节角度（弧度），顺序与 LEFT_ARM_JOINTS 一致
        right_angles: 右臂 7 关节角度（弧度），顺序与 RIGHT_ARM_JOINTS 一致
    """
    if len(left_angles) != 7:
        raise ValueError(f"left_angles must have 7 elements, got {len(left_angles)}")
    if len(right_angles) != 7:
        raise ValueError(f"right_angles must have 7 elements, got {len(right_angles)}")
    _TASK_SEEDS[task_type] = {"left": list(map(float, left_angles)), "right": list(map(float, right_angles))}


def _get_task_seed(task_type: str) -> dict[str, list[float]] | None:
    """获取语义种子，支持别名解析（"default" → "pick_table"）。"""
    resolved = task_type
    while isinstance(_TASK_SEEDS.get(resolved), str):
        resolved = _TASK_SEEDS[resolved]
    return _TASK_SEEDS.get(resolved)


def _init_default_task_seeds() -> None:
    """用 READY_POSE 初始化默认语义种子（pick_table）。

    在模块加载时调用，确保 READY_POSE 已被导入后执行。
    """
    try:
        from .constants import LEFT_ARM_JOINTS, READY_POSE, RIGHT_ARM_JOINTS
    except ImportError:
        return  # constants 不可用时跳过（如独立测试）
    left = [READY_POSE.get(j, 0.0) for j in LEFT_ARM_JOINTS]
    right = [READY_POSE.get(j, 0.0) for j in RIGHT_ARM_JOINTS]
    _TASK_SEEDS.setdefault("pick_table", {"left": left, "right": right})
    _TASK_SEEDS.setdefault("default", "pick_table")


class WalkerS2IK:
    """Dual-arm IK solver for Walker S2.

    Target poses are [x, y, z, roll, pitch, yaw] in the robot base frame defined
    by the URDF loaded into Pinocchio. The solver returns 7-DoF arm joint values
    in LEFT_ARM_JOINTS / RIGHT_ARM_JOINTS order by default. When unlock_waist is
    enabled for a single-arm solve, waist_yaw_joint is prepended to that side's
    returned joint list.
    """

    LEFT_ARM_JOINTS = [
        "L_shoulder_pitch_joint",
        "L_shoulder_roll_joint",
        "L_shoulder_yaw_joint",
        "L_elbow_roll_joint",
        "L_elbow_yaw_joint",
        "L_wrist_pitch_joint",
        "L_wrist_roll_joint",
    ]

    RIGHT_ARM_JOINTS = [
        "R_shoulder_pitch_joint",
        "R_shoulder_roll_joint",
        "R_shoulder_yaw_joint",
        "R_elbow_roll_joint",
        "R_elbow_yaw_joint",
        "R_wrist_pitch_joint",
        "R_wrist_roll_joint",
    ]

    WAIST_JOINT = "waist_yaw_joint"
    LEFT_EE_FRAME = "L_sixforce_link"
    RIGHT_EE_FRAME = "R_sixforce_link"

    def __init__(self, urdf_path: str, joint_limits=None, joint_limit_margin: float = 0.0):
        self.urdf_path = str(urdf_path)
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()

        self._validate_model()
        self._joint_limit_override_count = self._apply_joint_limits(joint_limits, joint_limit_margin)
        self.left_ee_id = self.model.getFrameId(self.LEFT_EE_FRAME)
        self.right_ee_id = self.model.getFrameId(self.RIGHT_EE_FRAME)

        self.left_arm_q_indices, self.left_arm_v_indices = self._joint_indices(self.LEFT_ARM_JOINTS)
        self.right_arm_q_indices, self.right_arm_v_indices = self._joint_indices(self.RIGHT_ARM_JOINTS)
        self.waist_q_indices, self.waist_v_indices = self._joint_indices([self.WAIST_JOINT])

        self.joint_name_to_q_idx: dict[str, int] = {}
        self._name_mapping_built = False
        self.q = pin.neutral(self.model)
        self.q_initial: np.ndarray | None = None
        self.q_neutral_left: np.ndarray | None = None
        self.q_neutral_right: np.ndarray | None = None
        self._ik_sync_joint_names = set(self.LEFT_ARM_JOINTS + self.RIGHT_ARM_JOINTS + [self.WAIST_JOINT])
        self._left_fail_count = 0
        self._right_fail_count = 0
        self._fail_reset_threshold = 30
        self.last_fail_info: dict[str, Any] | None = None

        # ── 语义种子（实例级覆盖，优先级高于模块级 _TASK_SEEDS）──
        self._task_seeds: dict[str, dict[str, list[float]]] = {}

        # ── 随机种子 RNG ──
        self._rng = np.random.RandomState(42)

        # ── 层级 IK 用 forearm 长度（从 shoulder_yaw 到 wrist_pitch 的沿链距离）──
        self._forearm_length: float | None = None

    def _validate_model(self) -> None:
        missing_joints = [
            name for name in self.LEFT_ARM_JOINTS + self.RIGHT_ARM_JOINTS + [self.WAIST_JOINT]
            if not self.model.existJointName(name)
        ]
        if missing_joints:
            raise ValueError(f"Walker S2 IK URDF is missing joints: {missing_joints}")

        frame_names = [frame.name for frame in self.model.frames]
        missing_frames = [
            name for name in (self.LEFT_EE_FRAME, self.RIGHT_EE_FRAME)
            if name not in frame_names
        ]
        if missing_frames:
            raise ValueError(f"Walker S2 IK URDF is missing frames: {missing_frames}")

    def _joint_indices(self, joint_names: list[str]) -> tuple[list[int], list[int]]:
        q_indices = []
        v_indices = []
        for name in joint_names:
            jid = self.model.getJointId(name)
            q_indices.append(self.model.joints[jid].idx_q)
            v_indices.append(self.model.joints[jid].idx_v)
        return q_indices, v_indices

    def _apply_joint_limits(self, joint_limits, margin: float = 0.0) -> int:
        if joint_limits is None:
            return 0
        margin = float(margin)
        applied = 0
        for name, limits in joint_limits.items():
            if not self.model.existJointName(name):
                continue
            jid = self.model.getJointId(name)
            joint = self.model.joints[jid]
            if joint.nq != 1:
                continue
            q_idx = joint.idx_q
            lower, upper = limits
            safe_lower = max(float(self.model.lowerPositionLimit[q_idx]), float(lower) + margin)
            safe_upper = min(float(self.model.upperPositionLimit[q_idx]), float(upper) - margin)
            if safe_lower >= safe_upper:
                raise ValueError(
                    f"Invalid constrained IK limits for {name}: "
                    f"[{safe_lower:.4f}, {safe_upper:.4f}]"
                )
            if (
                safe_lower != float(self.model.lowerPositionLimit[q_idx])
                or safe_upper != float(self.model.upperPositionLimit[q_idx])
            ):
                applied += 1
            self.model.lowerPositionLimit[q_idx] = safe_lower
            self.model.upperPositionLimit[q_idx] = safe_upper
        return applied

    # ── 辅助：forearm 长度（shoulder_yaw → wrist_pitch link 的沿链距离）──
    def _get_forearm_length(self) -> float:
        """计算从 shoulder_yaw 原点到 wrist 中心的沿链近似距离。

        用于层级 IK Layer 2：从目标 EE 位置沿接近方向回退此距离，
        得到 shoulder 子问题的目标 elbow 位置。
        """
        if self._forearm_length is not None:
            return self._forearm_length

        # 通过 FK 计算：neutral q 下，shoulder_yaw 原点 → wrist_pitch 原点的距离
        q_neutral = pin.neutral(self.model)
        pin.forwardKinematics(self.model, self.data, q_neutral)
        pin.updateFramePlacements(self.model, self.data)

        def _joint_origin(joint_name: str) -> np.ndarray:
            jid = self.model.getJointId(joint_name)
            return np.asarray(self.data.oMi[jid].translation, dtype=float)

        # 分别计算左右臂，取平均值
        lengths = []
        for side_joints in (self.LEFT_ARM_JOINTS, self.RIGHT_ARM_JOINTS):
            shoulder_yaw = side_joints[2]  # shoulder_yaw_joint
            wrist_pitch = side_joints[5]   # wrist_pitch_joint
            shoulder_origin = _joint_origin(shoulder_yaw)
            # wrist_pitch 的 origin 在 data.oMi 中对应的是 wrist_pitch_joint 的 placement
            wrist_origin = _joint_origin(wrist_pitch)
            lengths.append(float(np.linalg.norm(wrist_origin - shoulder_origin)))
        self._forearm_length = float(np.mean(lengths))
        return self._forearm_length

    # ── 随机种子生成 ──
    def _generate_random_seeds(self, n: int, side: str) -> list[np.ndarray]:
        """在关节限位内生成 n 个随机配置（均匀采样）。

        对 7-DOF 臂关节在各自限位内独立均匀采样，不做 LHS 以保持简单性。
        seed 独立性保证每次调用结果不同（依赖内部 _rng 状态）。
        """
        arm_joints = self.LEFT_ARM_JOINTS if side == "left" else self.RIGHT_ARM_JOINTS
        seeds = []
        for _ in range(n):
            q_seed = self.q.copy()
            for joint_name in arm_joints:
                if not self.model.existJointName(joint_name):
                    continue
                jid = self.model.getJointId(joint_name)
                q_idx = self.model.joints[jid].idx_q
                lo = float(self.model.lowerPositionLimit[q_idx])
                hi = float(self.model.upperPositionLimit[q_idx])
                q_seed[q_idx] = self._rng.uniform(lo, hi)
            seeds.append(q_seed)
        return seeds

    # ── Layer 1: Torso 解析解 ──
    def _analytical_waist(self, target_xyz: np.ndarray) -> float:
        """计算 waist_yaw 的解析解：使躯干朝向目标方位角。

        Args:
            target_xyz: 目标位置 [x, y, z]（URDF base frame）
        Returns:
            waist_yaw 角度（rad），裁剪到关节限位
        """
        x, y = float(target_xyz[0]), float(target_xyz[1])
        waist_yaw = np.arctan2(y, x)
        # 裁剪到限位
        waist_q_idx = self.waist_q_indices[0]
        lo = float(self.model.lowerPositionLimit[waist_q_idx])
        hi = float(self.model.upperPositionLimit[waist_q_idx])
        return float(np.clip(waist_yaw, lo, hi))

    def set_neutral_config(self, left_angles: list[float], right_angles: list[float]) -> None:
        if len(left_angles) != len(self.LEFT_ARM_JOINTS):
            raise ValueError(f"left neutral expects {len(self.LEFT_ARM_JOINTS)} joints")
        if len(right_angles) != len(self.RIGHT_ARM_JOINTS):
            raise ValueError(f"right neutral expects {len(self.RIGHT_ARM_JOINTS)} joints")
        self.q_neutral_left = np.asarray(left_angles, dtype=float)
        self.q_neutral_right = np.asarray(right_angles, dtype=float)

    def save_initial_q(self) -> None:
        self.q_initial = self.q.copy()

    @staticmethod
    def xyzrpy_to_se3(xyzrpy) -> pin.SE3:
        pose = np.asarray(xyzrpy, dtype=float)
        if pose.shape != (6,):
            raise ValueError(f"xyzrpy target must have shape (6,), got {pose.shape}")
        pos = pose[:3]
        roll, pitch, yaw = pose[3:]
        rot = pin.rpy.rpyToMatrix(float(roll), float(pitch), float(yaw))
        return pin.SE3(rot, pos)

    @staticmethod
    def se3_to_xyzrpy(se3: pin.SE3) -> np.ndarray:
        pos = np.asarray(se3.translation, dtype=float)
        rpy = pin.rpy.matrixToRpy(se3.rotation)
        return np.concatenate([pos, rpy])

    def _build_name_mapping(self, joint_names: list[str]) -> None:
        for name in joint_names:
            if name not in self.joint_name_to_q_idx and self.model.existJointName(name):
                jid = self.model.getJointId(name)
                self.joint_name_to_q_idx[name] = self.model.joints[jid].idx_q
        self._name_mapping_built = True

    def sync_joint_positions(self, joint_names: list[str], joint_positions: list[float]) -> None:
        if len(joint_names) != len(joint_positions):
            raise ValueError("joint_names and joint_positions must have the same length")
        if not self._name_mapping_built:
            self._build_name_mapping(joint_names)

        if self.q_initial is not None:
            self.q = self.q_initial.copy()

        for name, pos in zip(joint_names, joint_positions):
            q_idx = self.joint_name_to_q_idx.get(name)
            if q_idx is None:
                continue
            if self.q_initial is None or name in self._ik_sync_joint_names:
                self.q[q_idx] = float(pos)

    def get_ee_pose(self, side: str) -> np.ndarray:
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        if side == "left":
            return self.se3_to_xyzrpy(self.data.oMf[self.left_ee_id])
        if side == "right":
            return self.se3_to_xyzrpy(self.data.oMf[self.right_ee_id])
        raise ValueError(f"Invalid arm side '{side}', expected 'left' or 'right'")

    def get_both_ee_poses(self) -> dict[str, np.ndarray]:
        return {
            "left": self.get_ee_pose("left"),
            "right": self.get_ee_pose("right"),
        }

    def _reset_arm_warmstart(self, side: str) -> None:
        if self.q_initial is None:
            return
        indices = self.left_arm_q_indices if side == "left" else self.right_arm_q_indices
        for idx in indices:
            self.q[idx] = self.q_initial[idx]

    def solve_ik_single_arm(
        self,
        target_se3: pin.SE3,
        side: str,
        max_iter: int = 150,
        pos_tol: float = 5e-3,
        rot_tol: float = 5e-3,
        damping: float = 1e-4,
        dt: float = 1.0,
        dq_max: float = 0.5,
        pos_weight: float = 1.0,
        rot_weight: float = 1.0,
        rot_axis_weights=None,
        null_weight: float = 0.1,
        unlock_waist: bool = False,
        task_type: str = "default",
        num_random_seeds: int = 0,
        _hierarchical_warmstart: bool = False,
    ) -> tuple[np.ndarray, bool]:
        """单臂 IK 求解（加权 DLS + 多级种子回退）。

        种子回退策略（_hierarchical_warmstart=False 时启用）：
          1. 上一帧 warm-start (self.q)
          2. 语义种子 (task_type → TASK_SEEDS)
          3. N 个随机种子

        Args:
            target_se3: 目标位姿 (pin.SE3)
            side: "left" 或 "right"
            task_type: 语义种子标识（如 "pick_table"），默认 "default"
            num_random_seeds: 随机种子数量，默认 5
            _hierarchical_warmstart: 内部标志，True 表示 warm-start 已由层级 IK 设置，
                                     跳过种子回退，只做单次精修
        Returns:
            (active_q, success)
        """
        # ── 解析 side 参数 ──
        if side == "left":
            ee_id = self.left_ee_id
            q_indices = list(self.left_arm_q_indices)
            v_indices = list(self.left_arm_v_indices)
            q_neutral_active = self.q_neutral_left
        elif side == "right":
            ee_id = self.right_ee_id
            q_indices = list(self.right_arm_q_indices)
            v_indices = list(self.right_arm_v_indices)
            q_neutral_active = self.q_neutral_right
        else:
            raise ValueError(f"Invalid arm side '{side}', expected 'left' or 'right'")

        if unlock_waist:
            waist_q_idx = self.waist_q_indices[0]
            q_indices = list(self.waist_q_indices) + q_indices
            v_indices = list(self.waist_v_indices) + v_indices
            if q_neutral_active is not None:
                waist_neutral = self.q_initial[waist_q_idx] if self.q_initial is not None else self.q[waist_q_idx]
                q_neutral_active = np.concatenate([[float(waist_neutral)], q_neutral_active])

        if rot_axis_weights is None:
            rot_axis_weights = (1.0, 1.0, 1.0)

        # ── 构建种子列表 ──
        if _hierarchical_warmstart:
            seed_q_list = [self.q.copy()]
        else:
            seed_q_list = self._build_seed_list(side, task_type, num_random_seeds)

        # ── 级联尝试每个种子 ──
        last_active_q = None
        last_pos_err = float("inf")
        last_rot_err = float("inf")
        last_iter = 0

        for seed_idx, seed_q in enumerate(seed_q_list):
            active_q, ok, pos_err, rot_err, it = self._solve_ik_dls_core(
                target_se3, ee_id, q_indices, v_indices, q_neutral_active,
                seed_q, max_iter, pos_tol, rot_tol, damping, dt, dq_max,
                pos_weight, rot_weight, rot_axis_weights, null_weight,
            )
            last_active_q = active_q
            last_pos_err = pos_err
            last_rot_err = rot_err
            last_iter = it

            if ok:
                self.last_fail_info = None
                return active_q, True

        # 全部种子失败 → 返回 best-effort 结果
        self.last_fail_info = {
            "side": side,
            "pos_err": last_pos_err,
            "rot_err": last_rot_err,
            "iter": last_iter,
            "max_iter": int(max_iter),
            "seeds_tried": len(seed_q_list),
            "task_type": task_type,
        }
        return last_active_q, False

    def _build_seed_list(self, side: str, task_type: str, num_random_seeds: int) -> list[np.ndarray]:
        """构建种子回退列表：[上一帧, 语义种子, 随机种子 × N]。
        去重：如果语义种子与上一帧相同，跳过语义种子。
        """
        arm_joints = self.LEFT_ARM_JOINTS if side == "left" else self.RIGHT_ARM_JOINTS
        seeds = []

        # Seed 1: 上一帧 warm-start
        prev_q = self.q.copy()
        seeds.append(prev_q)

        # Seed 2: 语义种子（如果与上一帧不同）
        seed_config = self._resolve_task_seed(task_type)
        if seed_config is not None:
            side_key = "left" if side == "left" else "right"
            semantic_angles = seed_config.get(side_key)
            if semantic_angles is not None:
                semantic_q = prev_q.copy()
                for j, joint_name in enumerate(arm_joints):
                    if self.model.existJointName(joint_name):
                        q_idx = self.model.joints[self.model.getJointId(joint_name)].idx_q
                        semantic_q[q_idx] = float(semantic_angles[j])
                # 去重：如果语义种子与上一帧实质上相同，跳过
                if not np.allclose(semantic_q, prev_q):
                    seeds.append(semantic_q)

        # Seed 3+: 随机种子
        if num_random_seeds > 0:
            random_seeds = self._generate_random_seeds(num_random_seeds, side)
            seeds.extend(random_seeds)

        return seeds

    def _resolve_task_seed(self, task_type: str) -> dict | None:
        """解析 task_type → 语义种子配置（实例级优先，回退模块级）。"""
        # 实例级
        resolved = task_type
        while isinstance(self._task_seeds.get(resolved), str):
            resolved = self._task_seeds[resolved]
        if resolved in self._task_seeds:
            return self._task_seeds[resolved]
        # 模块级
        return _get_task_seed(task_type)

    # ── DLS 核心迭代（从种子 q 出发，运行 max_iter 次迭代）──
    def _solve_ik_dls_core(
        self,
        target_se3: pin.SE3,
        ee_id: int,
        q_indices: list[int],
        v_indices: list[int],
        q_neutral_active: np.ndarray | None,
        seed_q: np.ndarray,
        max_iter: int,
        pos_tol: float,
        rot_tol: float,
        damping: float,
        dt: float,
        dq_max: float,
        pos_weight: float,
        rot_weight: float,
        rot_axis_weights,
        null_weight: float,
    ) -> tuple[np.ndarray, bool, float, float, int]:
        """加权 DLS 核心迭代，从 seed_q 出发求解。

        Returns:
            (active_q, success, pos_err, rot_err, iterations)
        """
        n_dof = len(v_indices)
        q = seed_q.copy()
        rot_axis_weights = np.asarray(rot_axis_weights, dtype=float)
        if rot_axis_weights.shape != (3,):
            raise ValueError(f"rot_axis_weights must have shape (3,), got {rot_axis_weights.shape}")
        rot_weights = rot_weight * rot_axis_weights
        weight = np.diag([pos_weight] * 3 + rot_weights.tolist())
        max_rot_weight = float(np.max(np.abs(rot_weights)))
        check_rotation = max_rot_weight > 1e-9
        effective_rot_tol = float("inf") if not check_rotation else rot_tol / max(max_rot_weight, 0.01)
        use_null = q_neutral_active is not None and null_weight > 0.0
        pos_err = float("inf")
        rot_err = float("inf")
        it = 0

        for it in range(1, max_iter + 1):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            current_pose = self.data.oMf[ee_id]
            error = pin.log(current_pose.actInv(target_se3)).vector
            pos_err = float(np.linalg.norm(error[:3]))
            rot_err = float(np.linalg.norm(error[3:] * rot_axis_weights)) if check_rotation else 0.0
            if pos_err < pos_tol and (not check_rotation or rot_err < effective_rot_tol):
                active_q = np.array([q[idx] for idx in q_indices], dtype=float)
                for idx in q_indices:
                    self.q[idx] = q[idx]
                return active_q, True, pos_err, rot_err, it

            jac_full = pin.computeFrameJacobian(self.model, self.data, q, ee_id, pin.LOCAL)
            jac_active = jac_full[:, v_indices]
            jac_w = weight @ jac_active
            err_w = weight @ error

            jtj = jac_w.T @ jac_w + damping * np.eye(n_dof)
            dq_task = np.linalg.solve(jtj, jac_w.T @ (dt * err_w))

            if use_null:
                jac_w_pinv = np.linalg.solve(jtj, jac_w.T)
                null_projector = np.eye(n_dof) - jac_w_pinv @ jac_w
                q_active = np.array([q[idx] for idx in q_indices], dtype=float)
                dq_null = null_weight * null_projector @ (q_neutral_active - q_active)
                dq_active = dq_task + dq_null
            else:
                dq_active = dq_task

            dq_norm = float(np.linalg.norm(dq_active))
            if dq_norm > dq_max:
                dq_active = dq_active * (dq_max / dq_norm)

            for j, idx in enumerate(q_indices):
                q[idx] += dq_active[j]
                q[idx] = np.clip(q[idx], self.model.lowerPositionLimit[idx], self.model.upperPositionLimit[idx])

        active_q = np.array([q[idx] for idx in q_indices], dtype=float)
        for idx in q_indices:
            self.q[idx] = q[idx]
        return active_q, False, pos_err, rot_err, it

    def _update_fail_count(self, side: str, success: bool) -> None:
        if side == "left":
            if success:
                self._left_fail_count = 0
            else:
                self._left_fail_count += 1
                if self._left_fail_count >= self._fail_reset_threshold:
                    self._reset_arm_warmstart("left")
                    self._left_fail_count = 0
        else:
            if success:
                self._right_fail_count = 0
            else:
                self._right_fail_count += 1
                if self._right_fail_count >= self._fail_reset_threshold:
                    self._reset_arm_warmstart("right")
                    self._right_fail_count = 0

    # ── 子空间 IK（用于层级 IK Layer 2：shoulder 3-DOF 子问题）──
    def _solve_ik_subspace(
        self,
        target_se3: pin.SE3,
        side: str,
        active_joint_names: list[str],
        max_iter: int = 80,
        pos_tol: float = 3e-2,
        rot_tol: float = 5e-2,
        damping: float = 1e-4,
        dt: float = 1.0,
        dq_max: float = 0.5,
        rot_weight: float = 0.0,
        rot_axis_weights=None,
        seed_q: np.ndarray | None = None,
    ) -> tuple[np.ndarray, bool, float, float]:
        """对指定关节子集求解 IK（只优化 active_joint_names 指定的关节）。

        用于层级 IK 的 shoulder 子问题：只优化 shoulder_pitch/roll/yaw 3 个关节，
        其他关节保持 seed_q 中的值不变。

        Args:
            target_se3: 目标位姿（SE3）
            side: "left" 或 "right"
            active_joint_names: 要优化的关节名列表（如 shoulder 3 关节）
            seed_q: 初始 q 向量（默认使用 self.q）
            **kwargs: 其余参数与 solve_ik_single_arm 一致
        Returns:
            (active_q, success, pos_err, rot_err)
        """
        ee_id = self.left_ee_id if side == "left" else self.right_ee_id
        q = (seed_q if seed_q is not None else self.q).copy()

        # 收集活动关节的 q/v 索引
        q_indices = []
        v_indices = []
        for name in active_joint_names:
            jid = self.model.getJointId(name)
            q_indices.append(self.model.joints[jid].idx_q)
            v_indices.append(self.model.joints[jid].idx_v)

        n_dof = len(v_indices)
        if rot_axis_weights is None:
            rot_axis_weights = (1.0, 1.0, 1.0)
        rot_axis_weights = np.asarray(rot_axis_weights, dtype=float)
        rot_weights = rot_weight * rot_axis_weights
        weight = np.diag([1.0] * 3 + rot_weights.tolist())
        max_rot_weight = float(np.max(np.abs(rot_weights)))
        check_rotation = max_rot_weight > 1e-9
        effective_rot_tol = float("inf") if not check_rotation else rot_tol / max(max_rot_weight, 0.01)
        pos_err = float("inf")
        rot_err = float("inf")

        for it in range(1, max_iter + 1):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            current_pose = self.data.oMf[ee_id]
            error = pin.log(current_pose.actInv(target_se3)).vector
            pos_err = float(np.linalg.norm(error[:3]))
            rot_err = float(np.linalg.norm(error[3:] * rot_axis_weights)) if check_rotation else 0.0
            if pos_err < pos_tol and (not check_rotation or rot_err < effective_rot_tol):
                active_q = np.array([q[idx] for idx in q_indices], dtype=float)
                for idx in q_indices:
                    self.q[idx] = q[idx]
                return active_q, True, pos_err, rot_err

            jac_full = pin.computeFrameJacobian(self.model, self.data, q, ee_id, pin.LOCAL)
            jac_active = jac_full[:, v_indices]
            jac_w = weight @ jac_active
            err_w = weight @ error

            jtj = jac_w.T @ jac_w + damping * np.eye(n_dof)
            dq_active = np.linalg.solve(jtj, jac_w.T @ (dt * err_w))

            dq_norm = float(np.linalg.norm(dq_active))
            if dq_norm > dq_max:
                dq_active = dq_active * (dq_max / dq_norm)

            for j, idx in enumerate(q_indices):
                q[idx] += dq_active[j]
                q[idx] = np.clip(q[idx], self.model.lowerPositionLimit[idx], self.model.upperPositionLimit[idx])

        active_q = np.array([q[idx] for idx in q_indices], dtype=float)
        for idx in q_indices:
            self.q[idx] = q[idx]
        return active_q, False, pos_err, rot_err

    # ── 层级 IK ──
    def solve_ik_hierarchical(
        self,
        target_se3: pin.SE3,
        side: str,
        unlock_waist: bool = False,
        max_iter_full: int = 80,
        pos_tol: float = 5e-3,
        rot_tol: float = 5e-3,
        damping: float = 1e-4,
        dt: float = 1.0,
        dq_max: float = 0.5,
        pos_weight: float = 1.0,
        rot_weight: float = 0.2,
        rot_axis_weights=None,
        null_weight: float = 0.1,
        task_type: str = "default",
        num_random_seeds: int = 0,
    ) -> tuple[np.ndarray, bool]:
        """层级 IK：torso → shoulder → full arm 三级求解。

        Layer 1 (Torso): 解析 waist_yaw（仅 unlock_waist=True）
        Layer 2 (Shoulder): 3-DOF 子空间 IK 定位 elbow（粗定位）
        Layer 3 (Full arm): 7-DOF 精修，从 Layer 2 结果出发

        Args:
            target_se3: 目标位姿
            side: "left" 或 "right"
            **kwargs: 其余参数与 solve_ik_single_arm 一致
        Returns:
            (active_q, success)
        """
        arm_joints = self.LEFT_ARM_JOINTS if side == "left" else self.RIGHT_ARM_JOINTS
        shoulder_joints = arm_joints[:3]  # shoulder_pitch, shoulder_roll, shoulder_yaw

        # ── Layer 1: Torso 解析解 ──
        if unlock_waist:
            target_xyz = np.asarray(target_se3.translation, dtype=float)
            waist_yaw = self._analytical_waist(target_xyz)
            waist_q_idx = self.waist_q_indices[0]
            self.q[waist_q_idx] = waist_yaw

        # ── Layer 2: Shoulder 3-DOF 子问题 ──
        # 从 EE 目标沿接近方向回退 forearm 长度，得到 shoulder 子问题的目标位置
        target_pos = np.asarray(target_se3.translation, dtype=float)
        target_rot = target_se3.rotation
        # EE 接近方向 = 局部 z 轴在 world 中的方向
        approach_dir = target_rot[:, 2]  # 3rd column = local z
        forearm_len = self._get_forearm_length()
        # 目标 wrist center 位置（沿接近方向回退到 elbow 附近）
        # 注意：shoulder_yaw → wrist_pitch 的距离 ≈ forearm_length
        # 从 EE 沿 -z 方向回退一个 wrist_offset（sixforce → wrist_pitch）
        # 使 shoulder 子问题定位的目标点接近 elbow 位置
        wrist_offset = 0.08  # sixforce_link → wrist_roll_link 的近似距离
        target_elbow_xyz = target_pos - approach_dir * (forearm_len + wrist_offset)
        target_elbow_se3 = pin.SE3(target_rot, target_elbow_xyz)

        # 用 shoulder 3 关节优化 elbow 位置（只关心位置，rot_weight=0）
        shoulder_q, shoulder_ok, _, _ = self._solve_ik_subspace(
            target_elbow_se3, side, shoulder_joints,
            max_iter=60, pos_tol=pos_tol * 3.0, rot_weight=0.0,
            dq_max=dq_max,
        )

        # ── Layer 3: Full arm 7-DOF 精修 ──
        # warm-start 来自 Layer 2（shoulder 已粗定位）
        active_q, ok = self.solve_ik_single_arm(
            target_se3, side,
            max_iter=max_iter_full, pos_tol=pos_tol, rot_tol=rot_tol,
            damping=damping, dt=dt, dq_max=dq_max,
            pos_weight=pos_weight, rot_weight=rot_weight,
            rot_axis_weights=rot_axis_weights, null_weight=null_weight,
            unlock_waist=unlock_waist,
            task_type=task_type, num_random_seeds=num_random_seeds,
            _hierarchical_warmstart=True,  # 已从 Layer 2 获得粗解
        )
        return active_q, ok

    def solve_dual_arm(
        self,
        left_target_xyzrpy=None,
        right_target_xyzrpy=None,
        joint_names: list[str] | None = None,
        joint_positions: list[float] | None = None,
        unlock_waist: bool = False,
        task_type: str = "default",
        use_hierarchical: bool = False,
        **ik_kwargs,
    ) -> dict[str, Any]:
        if joint_names is not None and joint_positions is not None:
            self.sync_joint_positions(joint_names, joint_positions)

        if unlock_waist and left_target_xyzrpy is not None and right_target_xyzrpy is not None:
            return {
                "error": "unlock_waist_not_supported_for_dual_arm_sequential_ik",
                "diagnostics": {
                    "reason": "waist_yaw_joint is shared by both arms; current solver solves arms sequentially",
                },
            }

        result: dict[str, Any] = {}
        diagnostics: dict[str, Any] = {}

        if left_target_xyzrpy is not None:
            target = self.xyzrpy_to_se3(left_target_xyzrpy)
            if use_hierarchical:
                active_q, ok = self.solve_ik_hierarchical(
                    target, "left", unlock_waist=unlock_waist, task_type=task_type, **ik_kwargs,
                )
            else:
                active_q, ok = self.solve_ik_single_arm(
                    target, "left", unlock_waist=unlock_waist, task_type=task_type, **ik_kwargs,
                )
            self._update_fail_count("left", ok)
            result["left_joint_names"] = ([self.WAIST_JOINT] if unlock_waist else []) + list(self.LEFT_ARM_JOINTS)
            result["left_joint_positions"] = active_q
            result["left_success"] = ok
            if self.last_fail_info is not None:
                diagnostics["left"] = dict(self.last_fail_info)

        if right_target_xyzrpy is not None:
            target = self.xyzrpy_to_se3(right_target_xyzrpy)
            if use_hierarchical:
                active_q, ok = self.solve_ik_hierarchical(
                    target, "right", unlock_waist=unlock_waist, task_type=task_type, **ik_kwargs,
                )
            else:
                active_q, ok = self.solve_ik_single_arm(
                    target, "right", unlock_waist=unlock_waist, task_type=task_type, **ik_kwargs,
                )
            self._update_fail_count("right", ok)
            result["right_joint_names"] = ([self.WAIST_JOINT] if unlock_waist else []) + list(self.RIGHT_ARM_JOINTS)
            result["right_joint_positions"] = active_q
            result["right_success"] = ok
            if self.last_fail_info is not None:
                diagnostics["right"] = dict(self.last_fail_info)

        if diagnostics:
            result["diagnostics"] = diagnostics
        return result

    def reset_runtime_state(self) -> None:
        self.q = pin.neutral(self.model)
        self.q_initial = None
        self._left_fail_count = 0
        self._right_fail_count = 0
        self.last_fail_info = None

    def register_task_seed(self, task_type: str, left_angles: list[float], right_angles: list[float]) -> None:
        """注册实例级语义任务种子（优先级高于模块级 _TASK_SEEDS）。

        Args:
            task_type: 任务类型标识（如 "pick_table", "pick_floor"）
            left_angles: 左臂 7 关节角度（弧度）
            right_angles: 右臂 7 关节角度（弧度）
        """
        if len(left_angles) != 7:
            raise ValueError(f"left_angles must have 7 elements, got {len(left_angles)}")
        if len(right_angles) != 7:
            raise ValueError(f"right_angles must have 7 elements, got {len(right_angles)}")
        self._task_seeds[task_type] = {"left": list(map(float, left_angles)), "right": list(map(float, right_angles))}


# ── 模块加载时初始化默认语义种子 ──
_init_default_task_seeds()
