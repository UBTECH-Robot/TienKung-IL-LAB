#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""天工复位控制器入口脚本（向后兼容）。"""

try:
    from .tienkung.reset import ResetController, main
except ImportError:
    from tienkung.reset import ResetController, main

if __name__ == "__main__":
    main()
