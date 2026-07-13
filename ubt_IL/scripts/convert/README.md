# convert — HDF5 -> LeRobot 数据转换

宿主机 conda 环境运行。

| 机型 | 脚本 | 配置 |
|------|------|------|
| `tienkung_pro` | [tienkung_pro/](tienkung_pro/)(`convert.sh`、`convert_grasp_bottle.sh`) | [tienkung_pro/configs/](tienkung_pro/configs/) |
| `walker_s2` | [walker_s2/](walker_s2/)(`convert.sh`、`convert_real_to_lerobot_v3.py`、`subset_real_19_to_10.py`) | [walker_s2/configs/](walker_s2/configs/) |

通用转换器(非机型专属)在 [common/](common/):

- `convert_to_lerobot.py` — 通用 HDF5->LeRobot 转换器(被 `tienkung_pro/*.sh` 调用)
- `isaaclab2lerobot.py` / `isaaclab2lerobotv3.py` — IsaacLab 仿真数据转换(v2/v3)
- `lerobot2isaaclab.py` — LeRobot -> 单 HDF5 反向转换
- `all_robot_h5_info*.md` — 跨机型 HDF5 布局参考文档

`tienkung_pro` 的 shell 脚本通过 `$SCRIPT_DIR/../common/convert_to_lerobot.py` 调用通用转换器;`PROJECT_ROOT` 由 `$SCRIPT_DIR/../../..` 推导。
