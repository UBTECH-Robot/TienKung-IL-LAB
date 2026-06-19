#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Walker S2 复位控制脚本。"""

import os
import sys
import threading
from time import sleep

import rclpy
from rclpy.executors import MultiThreadedExecutor

try:
    from .walker_s2_controller import WalkerS2Controller
except ImportError:
    _dir = os.path.dirname(os.path.abspath(__file__))
    if _dir not in sys.path:
        sys.path.insert(0, _dir)
    from walker_s2_controller import WalkerS2Controller


class WalkerS2ResetController(WalkerS2Controller):
    """最小化控制器：等待状态后回 home 并张开双手。"""

    def run_reset(self):
        if not self.wait_for_state(timeout=5.0):
            self.get_logger().warning("No Walker S2 state received; only sending sim reset command")
            self.reset_sim()
            return False
        self.home(duration_sec=2.0, wait=True)
        self.open_hand("left", duration_sec=1.0, wait=True)
        self.open_hand("right", duration_sec=1.0, wait=True)
        return True


def main():
    rclpy.init()
    node = WalkerS2ResetController(node_name="walker_s2_reset_node")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        sleep(0.5)
        node.run_reset()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
