# 回归测试数据

`talon_kdl_reference.json` 于 2026-11-02 在“ROCOS 自有类型 + KDL 后端”阶段运行生成，使用本仓库 Talon URDF（已改为 revolute 和标准限位），base=`base_link`、tip=`link_7`，重力 `(0, 0, -9.81)`。它不是用 Pinocchio 生成的预期值。

四组样本 `k=0..3`，关节下标 `j=0..6`：

- `q[j] = 0.1 * k * (j 为奇数 ? -1 : 1) * (j+1)`；`v[j] = 0.03 * (j+1)`；`a[j] = -0.07 * (j+1)`。
- 仅 link_7 有外力，局部 force=`(0.3*k, 0.2, -0.4)`，torque=`(0.1, -0.2, 0.3)`，关于 link_7 原点。
- 记录 KDL FK、Jacobian、ID(q,v,a,external)、ID(q,0,0,{}) 和 FD(q,v,tau,external)。rotation 与 Jacobian 按行展开。

原始 JSON 的 SHA-256：`116904136b428e37bf8120cc7de214737b2efafab0e9f33cc1a1e0d1f26f2260`。第一阶段 `model.cpp` 的 SHA-256：`9719cb5d57cef16732f898e62f88fb56cabdefdde39eec4b10dc2bb00c4c7b98`；本机阶段源码归档在 `build/validation/kdl-stage-source.tar.gz`。更改机器人几何或惯量时应重新评估基准，不能用待测后端自动覆盖这些值。

`hardware/` 的五份 YAML 来自仓库提交 `2979c3b^` 下同名的 `config/` 文件，移除说明注释，保留测试所需配置数据。仅供 `hardware_test` 解析，不作为生产入口或设备配置。
