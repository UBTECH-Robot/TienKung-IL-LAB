#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""天工 Pro 控制器兼容入口。"""

try:
    from .tienkung.robot_controller import *  # noqa: F401,F403
except ImportError:
    from tienkung.robot_controller import *  # noqa: F401,F403
