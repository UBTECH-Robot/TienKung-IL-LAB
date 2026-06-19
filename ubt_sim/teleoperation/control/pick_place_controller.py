#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""天工抓放任务控制器兼容入口。"""

try:
    from .tienkung.pick_place_controller import *  # noqa: F401,F403
except ImportError:
    from tienkung.pick_place_controller import *  # noqa: F401,F403
