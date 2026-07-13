# train — 模型训练

容器内运行(`cd /ubt_IL/lerobot` 后执行)。

| 机型 | 入口 | 配置 |
|------|------|------|
| `tienkung_pro` | [tienkung_pro/train.sh](tienkung_pro/train.sh)(ACT,命令行参数式) | [tienkung_pro/configs/](tienkung_pro/configs/) |
| `walker_s2` | [walker_s2/train.sh](walker_s2/train.sh) / [README.md](walker_s2/README.md)(ACT + Pi0.5,`lerobot-train --config_path`) | [walker_s2/configs/](walker_s2/configs/) |

配置命名:`train_config_<machine>_<variant>.json`,variant 含模型(`act`/`pi05`)+ 数据集(`all`/`real_merged`/`sim_pick_place`/`sim`)+ 可选 dim(`19d`/`10d`)。

> 继续训练时 `--config_path` 须指向 checkpoint 内的 `train_config.json`,不是这里的源配置。
