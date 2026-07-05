"""Arm-command publisher node.

控制左手肘偏航关节(L_elbow_yaw_joint)做正弦往复运动或固定在指定位置。

Usage:
  # 位置模式 - 正弦运动
  ros2 run s2_example_py pub_arm_command

  # 位置模式 - 固定在指定位置 (单位: rad)
  ros2 run s2_example_py pub_arm_command --ros-args -p fixed_position:=0.3

  # 速度模式 - 正弦运动
  ros2 run s2_example_py pub_arm_command --ros-args -p mode:=1

  # 力矩模式 - 正弦运动
  ros2 run s2_example_py pub_arm_command --ros-args -p mode:=0

  # PVT模式 - 正弦运动
  ros2 run s2_example_py pub_arm_command --ros-args -p mode:=7 -p kp:=50.0 -p kd:=3.0

  # PVT模式 - 固定在指定位置 (单位: rad)，如有外力再去转动这个关节，它会自动回到这个位置
  ros2 run s2_example_py pub_arm_command --ros-args -p mode:=7 -p kp:=20.0 -p kd:=2.0 -p fixed_position:=0.38

  # 分段移动到预备姿态（叠加在 500Hz 持续发布之上，平滑不抖动）
  python3 pub_arm_command.py --init --staged-init --init-duration 20

  # 直达预备姿态
  python3 pub_arm_command.py --init --init-duration 3

参数:
  mode: 控制模式 (默认2)
    - 0: 力矩模式 (effort) - 带位置限位保护，effort_amplitude 控制力矩幅度
    - 1: 速度模式 (velocity) - 带位置限位保护
    - 2: 位置模式 (position) - 底层自动控制
    - 7: PVT模式 (CUSTOM_MODE_1) - 可调 kp/kd 的位置控制
  effort_amplitude: 力矩幅度 (Nm)，力矩模式下使用，默认5.0
  kp: 位置增益，PVT模式下使用，默认100.0
  kd: 阻尼增益，PVT模式下使用，默认5.0
  fixed_position: 固定目标位置 (rad)，设置后不做正弦运动，而是固定在该位置

注意：
- 先用 ros2 topic echo /sys/state/walker_mode 查看是否返回 true , 如果是 true 再运行，如果是 false 说明机器人未进入开发者模式，还不能控制手臂电机
- 需要先将遥控器开机，用遥控器F下拨，按D键让机器人回零/启动控制器，成功后，机器人会播报对应的提示音的。回零/启动控制器成功后，才能进入开发者模式。
- 然后用 ros2 service call /sys/task/developer_mode std_srvs/srv/SetBool "{data: true}" 命令进入开发者模式
- 如果进入开发者模式失败，可能是因为没有用遥控器控制机器人回零/启动控制器成功，请先用遥控器控制机器人回零/启动控制器成功后再进入开发者模式。启动控制器成功后，机器人会播报对应的提示音的。
- 首次运行时注意观察关节运动是否正常
- 按 Ctrl+C 停止运动
"""

import argparse
import math
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from mc_task_msgs.msg import JointCmd, RobotCommand
from mc_state_msgs.msg import RobotState

# ============================================================================
# 全身关节定义 + 预备姿态（与 robot_control.py 保持一致）
# ============================================================================

BODY_JOINT_NAMES = [
    "L_elbow_roll_joint",
    "L_elbow_yaw_joint",
    "L_shoulder_pitch_joint",
    "L_shoulder_roll_joint",
    "L_shoulder_yaw_joint",
    "L_wrist_pitch_joint",
    "L_wrist_roll_joint",
    "R_elbow_roll_joint",
    "R_elbow_yaw_joint",
    "R_shoulder_pitch_joint",
    "R_shoulder_roll_joint",
    "R_shoulder_yaw_joint",
    "R_wrist_pitch_joint",
    "R_wrist_roll_joint",
    "head_pitch_joint",
    "head_yaw_joint",
    "waist_yaw_joint",
]

BODY_JOINT_LIMITS = {
    "L_elbow_roll_joint":       (-2.6180, 0.0),
    "L_elbow_yaw_joint":        (-2.9147, 2.9147),
    "L_shoulder_pitch_joint":   (-2.8274, 2.8274),
    "L_shoulder_roll_joint":    (-1.85,   0.0873),
    "L_shoulder_yaw_joint":     (-2.8972, 2.8972),
    "L_wrist_pitch_joint":      (-1.5882, 1.5882),
    "L_wrist_roll_joint":       (-1.9897, 1.9897),
    "R_elbow_roll_joint":       (-2.6180, 0.0),
    "R_elbow_yaw_joint":        (-2.9147, 2.9147),
    "R_shoulder_pitch_joint":   (-2.8274, 2.8274),
    "R_shoulder_roll_joint":    (-1.85,   0.0873),
    "R_shoulder_yaw_joint":     (-2.8972, 2.8972),
    "R_wrist_pitch_joint":      (-1.5882, 1.5882),
    "R_wrist_roll_joint":       (-1.9897, 1.9897),
    "head_pitch_joint":         (-0.6807, 0.5061),
    "head_yaw_joint":           (-1.6406, 1.6406),
    "waist_yaw_joint":          (-2.7925, 2.7925),
}

DEFAULT_LOCK_JOINTS = {"head_pitch_joint", "head_yaw_joint", "waist_yaw_joint"}

READY_POSE = {
    "L_elbow_roll_joint":       -1.700,
    "L_elbow_yaw_joint":        1.5000,
    "L_shoulder_pitch_joint":   0.0000,
    "L_shoulder_roll_joint":    -0.1500,
    "L_shoulder_yaw_joint":     -1.5600,
    "L_wrist_pitch_joint":      0.0000,
    "L_wrist_roll_joint":       0.0000,
    "R_elbow_roll_joint":       -1.700,
    "R_elbow_yaw_joint":        -1.5000,
    "R_shoulder_pitch_joint":   0.0000,
    "R_shoulder_roll_joint":    -0.1500,
    "R_shoulder_yaw_joint":     1.5600,
    "R_wrist_pitch_joint":      0.0000,
    "R_wrist_roll_joint":       0.0000,
    "head_pitch_joint":         -0.6500,
    "head_yaw_joint":           0.0000,
    "waist_yaw_joint":          0.0000,
}

READY_STAGE_1_PITCH_ROLL_POSE = {
    "L_shoulder_yaw_joint": -1.5600,
    "R_shoulder_yaw_joint": 1.5600,
    "L_elbow_yaw_joint": 1.5000,
    "R_elbow_yaw_joint": -1.5000,
}

READY_STAGE_1_ELBOW_YAW_POSE = {
    "L_shoulder_pitch_joint":   -2.000,
    "R_shoulder_pitch_joint":   2.000,
    "L_wrist_pitch_joint": 0.8000,
    "R_wrist_pitch_joint": -0.8000,
    "L_elbow_roll_joint":        -2.6000,
    "R_elbow_roll_joint":        -2.6000,
}

READY_STAGE_2_POSE = {
    "L_shoulder_pitch_joint": READY_POSE["L_shoulder_pitch_joint"],
    "R_shoulder_pitch_joint": READY_POSE["R_shoulder_pitch_joint"],
}

# init 状态机参数
INIT_CONTROL_HZ = 500          # 与主循环一致（2ms/tick）
INIT_SETTLE_TOLERANCE = 0.03   # rad，阶段到位判定
INIT_SETTLE_MIN = 2.0          # s
INIT_SETTLE_MAX = 3.0          # s


class ArmCommandPublisher(Node):
    # 关节位置限位 (rad)，防止超出机械范围
    JOINT_POSITION_LIMIT = 0.5  # ±0.5 rad 安全范围
    # 力矩幅度 (Nm)，需要足够大才能驱动关节
    EFFORT_AMPLITUDE = 5.0
    # PVT控制默认增益
    DEFAULT_KP = 50.0  # 位置增益，响应速度
    DEFAULT_KD = 2.0    # 阻尼增益，抑制震荡

    def __init__(self, cli_args=None):
        super().__init__('pub_arm_command')
        self._cli_args = cli_args

        # 控制模式参数
        self.declare_parameter('mode', JointCmd.MODE_POSITION)
        self.control_mode = self.get_parameter('mode').value

        # 力矩幅度参数
        self.declare_parameter('effort_amplitude', self.EFFORT_AMPLITUDE)
        self.effort_amplitude = self.get_parameter('effort_amplitude').value

        # PVT控制增益参数
        self.declare_parameter('kp', self.DEFAULT_KP)
        self.kp = self.get_parameter('kp').value
        self.declare_parameter('kd', self.DEFAULT_KD)
        self.kd = self.get_parameter('kd').value

        # 固定位置参数（可选，设置后不做正弦运动）
        self.declare_parameter('fixed_position', -999.0)  # 默认值表示不启用
        fixed_pos = self.get_parameter('fixed_position').value
        self.fixed_position = None if fixed_pos < -100.0 else fixed_pos

        if self.fixed_position is not None:
            self.get_logger().info(
                f'固定位置模式: 目标位置 {self.fixed_position:.3f} rad '
                f'(Fixed position mode: target {self.fixed_position:.3f} rad)'
            )

        self.publisher = self.create_publisher(RobotCommand, '/mc/sdk/robot_command', 10)

        # 订阅关节状态，用于速度模式和力矩模式的闭环控制
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        qos.history = HistoryPolicy.KEEP_LAST
        self.state_subscription = self.create_subscription(
            RobotState,
            '/mc/sdk/robot_state',
            self.state_callback,
            qos,
        )

        # 目标关节名（可指定任意身体关节，如 L_shoulder_pitch_joint）
        self.declare_parameter('joint_name', 'L_elbow_yaw_joint')
        self.joint_name = self.get_parameter('joint_name').value

        # 当前关节位置 (速度/力矩模式下使用)
        self.current_position = None

        self.time_cnt = 0.0
        # 使用 Timer 替代 Rate，频率 500Hz (周期 2ms)
        self.timer = self.create_timer(0.002, self.timer_callback)

        # 日志限速：每秒最多输出一次状态日志
        self.last_log_time = 0.0
        self.log_interval = 1.0  # 秒

        # ---- init / staged-init 模式（叠加在 500Hz 持续发布之上）----
        # 与 robot_control.py --init --staged-init 等价，但保留 pub_arm_command
        # 永不断流的防抖特性：阶段间/settle 期间持续发布全部未锁定关节。
        self.all_positions = np.zeros(len(BODY_JOINT_NAMES), dtype=float)
        self.all_positions_received = False
        cli = cli_args
        self.lock_joints = (
            set() if (cli and getattr(cli, 'no_lock', False)) else set(DEFAULT_LOCK_JOINTS)
        )

        # run_mode: 'legacy'(单关节 sin/fixed) 或 'init'
        self.run_mode = 'legacy'
        self.init_stages = deque()        # [(label, pose_dict, duration_sec)]
        self.init_traj = None             # np.ndarray (N, 17)
        self.init_traj_idx = 0
        self.init_target_vec = None       # np.ndarray(17,) 当前阶段目标
        self.init_stage_joints = []       # 当前阶段涉及的关节名（settle 检查用）
        self.init_state = 'idle'          # idle | running | settling | done | failed
        self.init_settle_deadline = 0.0
        self.init_hold_vec = None         # done/failed 时的保持位姿

        if cli is not None and getattr(cli, 'init', False):
            self._configure_init(cli)

    def state_callback(self, msg: RobotState):
        """更新关节当前位置（legacy 单关节 + init 全身）。"""
        name_to_pos = {n: p for n, p in zip(msg.joint_states.name, msg.joint_states.position)}
        # init 模式：跟踪全部 17 关节
        for i, name in enumerate(BODY_JOINT_NAMES):
            if name in name_to_pos:
                self.all_positions[i] = name_to_pos[name]
        self.all_positions_received = True
        # legacy 模式：单关节
        if self.joint_name in name_to_pos:
            self.current_position = name_to_pos[self.joint_name]

    def timer_callback(self):
        """500Hz 主回调：按 run_mode 分发到 legacy 单关节或 init 状态机。"""
        if self.run_mode == 'init':
            self._init_tick()
        else:
            self._legacy_tick()

    def _legacy_tick(self):
        """legacy 单关节模式（sin / fixed_position，原 pub_arm_command 行为）。"""
        cmd = RobotCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()

        joint = JointCmd()
        joint.name = self.joint_name
        joint.control_mode = self.control_mode

        # 计算目标位置：固定位置或正弦运动
        if self.fixed_position is not None:
            target_position = self.fixed_position
        else:
            target_position = math.sin(self.time_cnt) * 0.5

        if self.control_mode == JointCmd.MODE_EFFORT:
            # 力矩模式：闭环位置限位控制
            joint.effort = self._compute_safe_command(
                target_position * self.effort_amplitude / 0.5, 'effort'
            )
        elif self.control_mode == JointCmd.MODE_VELOCITY:
            # 速度模式：闭环位置限位控制
            joint.velocity = self._compute_safe_command(target_position, 'velocity')
        elif self.control_mode == JointCmd.CUSTOM_MODE_1:
            # PVT模式：设置位置和增益
            joint.position = target_position
            joint.velocity = 0.0  # 不做速度前馈，kd仅起阻尼作用抑制震荡
            joint.v1 = self.kp  # kp
            joint.v2 = self.kd  # kd
        else:
            # 位置模式
            joint.position = target_position

        cmd.joint_cmd.append(joint)
        self.publisher.publish(cmd)

        # 仅在正弦运动模式下更新时间计数
        if self.fixed_position is None:
            self.time_cnt += 0.002

    # ------------------------------------------------------------------
    # init / staged-init 状态机
    # ------------------------------------------------------------------

    def _configure_init(self, cli_args):
        """根据 CLI 构建 init 阶段队列。"""
        self.run_mode = 'init'
        duration = cli_args.init_duration
        if cli_args.staged_init:
            if duration is None:
                duration = 20.0
            duration = max(1.5, float(duration))
            pr = duration * 0.35
            ey = duration * 0.35
            ot = duration * 0.2
            rs = duration * 0.1
            self.init_stages.extend([
                ('1a/3 肩 pitch + elbow roll', READY_STAGE_1_PITCH_ROLL_POSE, pr),
                ('1b/3 elbow yaw', READY_STAGE_1_ELBOW_YAW_POSE, ey),
                ('2/3 肩 pitch 回到预备姿态', READY_STAGE_2_POSE, ot),
                ('3/3 执行完整 READY_POSE', READY_POSE, rs),
            ])
        else:
            if duration is None:
                duration = 3.0
            self.init_stages.append(('READY_POSE', READY_POSE, float(duration)))
        self.get_logger().info(
            f'init 模式: {"staged" if cli_args.staged_init else "direct"}, '
            f'总时长 ~{duration:.1f}s, locked={sorted(self.lock_joints)}'
        )

    def _init_tick(self):
        """init 状态机：500Hz 每 tick 调度，永不断流。"""
        if not self.all_positions_received:
            return  # 尚未收到状态，不知道位姿，不发

        if self.init_state == 'idle':
            self._start_next_stage()

        if self.init_state == 'running':
            if self.init_traj_idx < len(self.init_traj):
                self._publish_all(self.init_traj[self.init_traj_idx])
                self.init_traj_idx += 1
            else:
                # 轨迹结束 → settling（持续 hold 目标，等收敛或超时）
                self.init_state = 'settling'
                self.init_settle_deadline = time.time() + self._settle_timeout()
                self.get_logger().info(
                    f'Stage 轨迹结束，进入 settle（超时 {self._settle_timeout():.1f}s）'
                )
                self._publish_all(self.init_target_vec)
        elif self.init_state == 'settling':
            self._publish_all(self.init_target_vec)  # 持续 hold 目标
            if self._settle_converged():
                self.get_logger().info('Stage 到位，进入下一阶段')
                self._start_next_stage()
            elif time.time() > self.init_settle_deadline:
                self._log_settle_errors()
                self.get_logger().error('Stage 未到位，init 失败（abort，hold 当前实际位姿）')
                self.init_hold_vec = self.all_positions.copy()
                self.init_state = 'failed'
        elif self.init_state in ('done', 'failed'):
            vec = self.init_hold_vec if self.init_state == 'failed' else self.init_target_vec
            if vec is not None:
                self._publish_all(vec)

    def _start_next_stage(self):
        """弹出下一阶段，从当前位姿构建 quintic 轨迹。"""
        if not self.init_stages:
            self.init_state = 'done'
            self.init_hold_vec = self.init_target_vec  # 最终 READY_POSE
            self.get_logger().info('✓ 全部阶段完成，持续 hold READY_POSE')
            return
        label, pose, duration = self.init_stages.popleft()
        # 目标向量：从当前位姿出发，覆盖阶段关节（限位裁剪）
        target = self.all_positions.copy()
        for name, angle in pose.items():
            idx = BODY_JOINT_NAMES.index(name)
            lo, hi = BODY_JOINT_LIMITS[name]
            target[idx] = max(lo, min(hi, float(angle)))
        self.init_traj = self._build_trajectory(target, duration)
        self.init_traj_idx = 0
        self.init_target_vec = target
        self.init_stage_joints = list(pose.keys())
        self.init_state = 'running'
        self.get_logger().info(
            f'Ready pose stage {label}: {duration:.2f}s, joints={self.init_stage_joints}'
        )

    def _build_trajectory(self, target_vec, duration_sec):
        """quintic 插值：(N,17) 轨迹，起止速度/加速度为 0。"""
        n_pts = max(2, int(duration_sec * INIT_CONTROL_HZ))
        tau = np.linspace(0.0, 1.0, n_pts)
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        start = self.all_positions
        return start[None, :] + s[:, None] * (target_vec - start)[None, :]

    def _publish_all(self, point_vec):
        """发布全部未锁定关节（init 模式持续发布，防抖动核心）。"""
        cmd = RobotCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = ''
        for idx, name in enumerate(BODY_JOINT_NAMES):
            if name in self.lock_joints:
                continue
            jc = JointCmd()
            jc.name = name
            if self.control_mode == JointCmd.CUSTOM_MODE_1:
                # PVT：velocity=0 纯阻尼（与 legacy PVT 一致）
                jc.control_mode = JointCmd.CUSTOM_MODE_1
                jc.position = float(point_vec[idx])
                jc.velocity = 0.0
                jc.v1 = self.kp
                jc.v2 = self.kd
            else:
                jc.control_mode = JointCmd.MODE_POSITION
                jc.position = float(point_vec[idx])
            cmd.joint_cmd.append(jc)
        if cmd.joint_cmd:
            self.publisher.publish(cmd)

    def _settle_timeout(self):
        if self.init_traj is not None:
            dur = len(self.init_traj) / INIT_CONTROL_HZ
            return max(INIT_SETTLE_MIN, min(dur, INIT_SETTLE_MAX))
        return INIT_SETTLE_MAX

    def _settle_converged(self):
        for name in self.init_stage_joints:
            idx = BODY_JOINT_NAMES.index(name)
            if abs(self.all_positions[idx] - self.init_target_vec[idx]) > INIT_SETTLE_TOLERANCE:
                return False
        return True

    def _log_settle_errors(self):
        errs = []
        for name in self.init_stage_joints:
            idx = BODY_JOINT_NAMES.index(name)
            actual = float(self.all_positions[idx])
            target = float(self.init_target_vec[idx])
            errs.append((name, actual, target, abs(actual - target)))
        errs.sort(key=lambda e: e[3], reverse=True)
        text = ', '.join(
            f'{n}: actual={a:+.4f}, target={t:+.4f}, err={e:.4f}' for n, a, t, e in errs
        )
        self.get_logger().warn(f'Settle 未到位: {text}')

    def _compute_safe_command(self, desired_value: float, mode_name: str) -> float:
        """计算安全的指令值，防止关节超出位置限位。

        Args:
            desired_value: 期望的速度或力矩值
            mode_name: 模式名称，用于日志 ('velocity' 或 'effort')

        Returns:
            安全的指令值：
            - 速度模式：超出限位时置零，等待自然反向
            - 力矩模式：超出限位时主动反向，驱动关节回安全范围
        """
        if self.current_position is None:
            # 未收到位置反馈，发送零指令以保安全
            self.get_logger().warn(
                f'未收到关节位置反馈，{mode_name}指令置零 '
                f'(No joint position feedback, {mode_name} set to zero)'
            )
            return 0.0

        pos = self.current_position
        limit = self.JOINT_POSITION_LIMIT
        is_effort_mode = (mode_name == 'effort')

        # 位置到达或超出限位范围时处理
        if pos > limit and desired_value > 0:
            # 已在上限位处且指令为正向
            if is_effort_mode:
                # 力矩模式：主动反向，驱动关节回安全范围
                self.get_logger().info(
                    f'位置 {pos:.3f} 达到上限 {limit:.3f}，力矩反向 '
                    f'(Position {pos:.3f} reaches upper limit {limit:.3f}, reversing effort)'
                )
                return -abs(desired_value)
            else:
                # 速度模式：置零停止，等待正弦波自然反向
                self.get_logger().info(
                    f'位置 {pos:.3f} 达到上限 {limit:.3f}，速度置零 '
                    f'(Position {pos:.3f} reaches upper limit {limit:.3f}, velocity set to zero)'
                )
                return 0.0
        elif pos < -limit and desired_value < 0:
            # 已在下限位处且指令为反向
            if is_effort_mode:
                # 力矩模式：主动反向，驱动关节回安全范围
                self.get_logger().info(
                    f'位置 {pos:.3f} 达到下限 {-limit:.3f}，力矩反向 '
                    f'(Position {pos:.3f} reaches lower limit {-limit:.3f}, reversing effort)'
                )
                return abs(desired_value)
            else:
                # 速度模式：置零停止，等待正弦波自然反向
                self.get_logger().info(
                    f'位置 {pos:.3f} 达到下限 {-limit:.3f}，速度置零 '
                    f'(Position {pos:.3f} reaches lower limit {-limit:.3f}, velocity set to zero)'
                )
                return 0.0

        # 限速日志：每秒最多输出一次
        current_time = self.time_cnt
        if current_time - self.last_log_time >= self.log_interval:
            self.get_logger().info(
                f'当前位置 {pos:.3f} 在安全范围内，{mode_name} 值为 {desired_value:.3f}，指令正常执行 '
                f'(Current position {pos:.3f} within safe range, executing {mode_name} command with value {desired_value:.3f})'
            )
            self.last_log_time = current_time

        # 在安全范围内，正常执行指令
        return desired_value


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Walker S2 手臂指令发布（单关节调试 + --init/--staged-init 预备姿态）',
    )
    parser.add_argument('--init', action='store_true', help='移动到预备姿态')
    parser.add_argument('--staged-init', action='store_true',
                        help='分段移动到预备姿态（建议 --init-duration 20~30）')
    parser.add_argument('--init-duration', type=float, default=None,
                        help='预备姿态时长(s)；直达默认 3.0，分段默认 20.0')
    parser.add_argument('--no-lock', action='store_true', help='不锁定 head/waist')
    cli_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = ArmCommandPublisher(cli_args=cli_args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()