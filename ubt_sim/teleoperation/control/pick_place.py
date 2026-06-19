#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""天工抓放任务入口脚本（向后兼容）。"""

try:
    from .tienkung.pick_place import PickPlaceController, main
except ImportError:
    from tienkung.pick_place import PickPlaceController, main

if __name__ == "__main__":
    main()
