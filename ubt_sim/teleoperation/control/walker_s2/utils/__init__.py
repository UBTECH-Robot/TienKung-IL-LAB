"""Walker S2 控制层库：父类 + 工具类（被顶层入口脚本引用，不直接运行）。

包内模块用相对导入（``from .constants import ...``）；顶层入口脚本通过
``from utils.X import ...`` 使用（顶层脚本运行时其所在目录在 sys.path 上，
``utils`` 作为子包可被导入）。

注意：``ik`` 依赖 pinocchio，按需导入（controller 在 initialize_ik 时懒加载），
此处不急切 re-export，避免把 pinocchio 耦合到 ``from utils.constants import``。
"""

from .constants import *  # noqa: F401,F403
