# ubt_IL/scripts

按 **功能 × 机型** 组织的脚本与配置。

| 功能 | 说明 | 运行环境 |
|------|------|----------|
| [convert/](convert/) | HDF5 -> LeRobot 数据转换 | 宿主机 conda |
| [train/](train/) | 模型训练配置与脚本 | 容器内 |
| [deploy/](deploy/) | 真机/仿真部署(rollout)、回放、复位、相机 | 容器内 |

每个功能目录下按机型管理:

- `tienkung_pro` — 天工(26 维)
- `walker_s2` — Walker S2(19/31 维)

跨机型通用的转换器放在 [convert/common/](convert/common/)。

机型 token 与代码一致:`robot.type=tienkung` / `walker`、根目录 `ubt_IL/tienkung/` / `ubt_IL/walker/`。

## 命名规范

- 目录:`<function>/<machine>/` + 通用脚本 `<function>/common/`
- convert 配置:`<machine>_<source?>_<dof>d_<sensor>.json`
- train 配置:`train_config_<machine>_<variant>.json`
- deploy robot 配置:`<machine>_<effector>_<dim>d.json`
- 机型目录内的脚本不带冗余机型后缀(如 `rollout.sh` 而非 `rollout_walker.sh`)
