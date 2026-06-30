"""Pinocchio-based inverse kinematics for Walker S2 dual arms.

Targets use [x, y, z, roll, pitch, yaw] in the Walker S2 URDF base frame.
Units are meters and radians. This module is ROS-independent so it can be
imported and tested with system Python.
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
    ) -> tuple[np.ndarray, bool]:
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

        n_dof = len(v_indices)
        q = self.q.copy()
        if rot_axis_weights is None:
            rot_axis_weights = (1.0, 1.0, 1.0)
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
                self.last_fail_info = None
                return active_q, True

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
        self.last_fail_info = {
            "side": side,
            "pos_err": pos_err,
            "rot_err": rot_err,
            "effective_rot_tol": float(effective_rot_tol),
            "iter": it,
            "max_iter": int(max_iter),
        }
        return active_q, False

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

    def solve_dual_arm(
        self,
        left_target_xyzrpy=None,
        right_target_xyzrpy=None,
        joint_names: list[str] | None = None,
        joint_positions: list[float] | None = None,
        unlock_waist: bool = False,
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
            active_q, ok = self.solve_ik_single_arm(target, "left", unlock_waist=unlock_waist, **ik_kwargs)
            self._update_fail_count("left", ok)
            result["left_joint_names"] = ([self.WAIST_JOINT] if unlock_waist else []) + list(self.LEFT_ARM_JOINTS)
            result["left_joint_positions"] = active_q
            result["left_success"] = ok
            if self.last_fail_info is not None:
                diagnostics["left"] = dict(self.last_fail_info)

        if right_target_xyzrpy is not None:
            target = self.xyzrpy_to_se3(right_target_xyzrpy)
            active_q, ok = self.solve_ik_single_arm(target, "right", unlock_waist=unlock_waist, **ik_kwargs)
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
