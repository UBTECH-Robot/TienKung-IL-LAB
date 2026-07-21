#!/usr/bin/env python3
"""Walker S2 统一控制器 CLI 入口（合并原 6 个 controller 家族脚本）。

子命令（各子命令保留原有 argparse，直接转发，行为与原脚本一致）：

  state      关节/夹爪/末端状态、单关节移动、预备姿态、夹爪控制（原 walker_s2_controller.py）
  joint      关节/手部调试（原 walker_s2_joint_test.py）
  endpoint   末端/TCP 位姿测试（原 walker_s2_endpoint_pose_test.py）
  reset      回 home + 张开双手（原 walker_s2_reset.py）
  analyze    关节阶跃/正弦响应分析 + CSV（原 joint_analysis.py）
  camera     相机话题信息（原 walker_s2_camera.py）

无子命令或首参为 flag 时默认走 ``state``，兼容旧调用：

    python walker_s2_controller.py --print-state
    python walker_s2_controller.py joint --print
    python walker_s2_controller.py reset

类实现见 ``utils.controller``（``WalkerS2Controller`` / ``RobotController``）；
消费方（carry_box / pick_part 等）应 ``from utils.controller import ...``。
"""

import importlib
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# (module, entry_func) — 同一模块可有多个入口函数
_SUBCOMMANDS = {
    "state":    ("utils.controller", "main"),
    "joint":    ("utils.controller", "main_joint"),
    "endpoint": ("utils.controller", "main_endpoint"),
    "reset":    ("utils.controller", "main_reset"),
    "analyze":  ("utils.joint_analyzer", "main"),
    "camera":   ("utils.camera", "main"),
}


def main():
    argv = sys.argv[1:]
    if argv and not argv[0].startswith("-") and argv[0] in _SUBCOMMANDS:
        sub, rest = argv[0], argv[1:]
    else:
        sub, rest = "state", argv  # 默认 state，兼容旧 --print-state 等无子命令调用
    mod_name, func_name = _SUBCOMMANDS[sub]
    getattr(importlib.import_module(mod_name), func_name)(rest)


if __name__ == "__main__":
    main()
