# TOOL 笛卡尔点动的 TCP 参考点

本说明记录 v3 实现行为。HTTP 接口契约以在线 Apifox「ROCOS API」为准；本次核对的在线描述仅说明方向转至 BASE，尚未明确速度参考点和旋转中心，需同步补充下述 TOOL 语义。

`POST /api/robot/jog/cartesian` 的 `frame="TOOL"` 使用当前活动工具的 TCP 作为速度参考点：纯旋转时保持 TCP 位置，法兰随工具偏置运动。工具包使用注册表中已组合的法兰到 TCP 变换。接口参数无需增加。

Robot 将 TOOL 的线速度、角速度方向转至 BASE，并把法兰系表达的 TCP 偏移传入 MoveJog。MoveJog 先按原有规则归一化输入方向并生成速度，再在每个周期按当前规划关节位置计算：

```text
r_base = R_base_flange(q) * p_flange_tcp
omega_base = R_base_tool * omega_tool
v_flange_base = v_tcp_base - omega_base × r_base
```

随后使用法兰 Jacobian 求关节速度。因此工具长度不参与输入方向归一化，纯旋转的 `speed` 不会因偏置变小。补偿在显式停止和点动喂入超时后的减速阶段持续生效。方向坐标旋转仍沿用原有每次请求时采样的行为。

`BASE`、`FLANGE`、`OBJECT` 以及直接传入 JogVec 的入口保持法兰速度参考点。`TOOL` 零平移偏置时退化为原有行为。同一次点动不能改变速度参考点偏移，须先停稳后重新启动。

离线回归：`ctest --test-dir build -R '^move_jog_tcp_test$' --output-on-failure`。测试只调用规划器和运动学模型，不连接硬件；覆盖带旋转和平移偏置的三个旋转轴、显式停车、超时停车、平移与混合 twist、速度尺度和零偏置行为。离散关节积分仍有数值误差，测试的 TCP 位置漂移阈值为 0.1 mm，不能据此承诺真机精度。
