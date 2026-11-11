# 使用 Hardware 执行整机辨识与阻抗验证

完整说明统一维护在 [examples/dynamics_id/README.md](../examples/dynamics_id/README.md)。

该 README 包含：

- 独立程序的构建与 Hardware 调用流程。
- CSP 激励执行、实测 `q/dq/tau` CSV 采集。
- `export_impedance_model.py` 的用途、输入输出、`Pβ` 计算与使用方法。
- 辨识模型与摩擦参数的配置、CST 阻抗验证。
- 位置与速度的零点规则、周期等待与离线回归测试。

速度读取使用 `raw_velocity / vel_per_unit / ratio`，不扣除位置零点。
`torque_per_unit` 保持 Hardware YAML 中的现有配置，标定另行处理。
