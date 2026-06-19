#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""天工抓放数据采集入口脚本（向后兼容）。"""

try:
    from .tienkung.pick_place_save_data import PickPlaceSaveDataController, main
except ImportError:
    from tienkung.pick_place_save_data import PickPlaceSaveDataController, main

if __name__ == "__main__":
    main()
