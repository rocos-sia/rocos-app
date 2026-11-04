# 整机辨识迁移的离线夹具

两轴有界 URDF，无旧版 `<hardware>` 标签。硬件 YAML 的顺序与模型相反，使用非零位置偏移、
非单位 ratio 与 torque_per_unit，验证接入程序不依赖轴数或文件顺序，不重复做单位转换。

`reference.json` 使用 dynamics-id-studio `payload_id` 分支的 `RobotModel.regressor` 与
`friction_matrix` 生成。刚体参数为名义参数的 1.1 倍，用于区分辨识回归参数与名义 URDF 惯量。
`trajectory_reference.json` 使用该分支的 `evaluate_fourier_trajectory` 生成。
测试只读取固定参考，不依赖本机 Studio 路径或 Python/Pinocchio 运行环境。

这些参数仅用于自动化离线验证，不是机器人部署配置或辨识结果。
