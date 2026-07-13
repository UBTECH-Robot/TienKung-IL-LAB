# deploy — 真机/仿真部署

容器内运行。

| 机型 | 脚本 | 配置 |
|------|------|------|
| `tienkung_pro` | [tienkung_pro/](tienkung_pro/):`rollout.sh`、`replay.py`、`reset.py`、`image_server.py`、`image_client.py` | — |
| `walker_s2` | [walker_s2/](walker_s2/):`rollout.sh`、`preview_camera.py`、[README.md](walker_s2/README.md) | [walker_s2/configs/](walker_s2/configs/) |

- `tienkung_pro/image_server.py` 部署到机器人端(相机 JPEG 流),`image_client.py` 在容器内验证图像流。
- `walker_s2` 的 robot 配置命名:`<machine>_<effector>_<dim>d.json`(如 `walker_s2_gripper_19d.json`、`walker_s2_v4_hand_31d.json`)。
