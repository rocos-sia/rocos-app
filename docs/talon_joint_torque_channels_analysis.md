# Talon 关节扭矩传感器通道分析与待修复问题

分析日期：2026-10-22。以下保留当时的分析记录；本文不代表已完成硬件验证。

2026-11-11 更新：负载力矩换算已改为独立的 `raw / load_torque_per_unit`，两种 `torque_source` 均不再使用位置零点、位置比例或传动比。下文的位置污染问题属于旧实现；当前字段与默认值见 [配置说明](../config/README.md#驱动器单位变换)。

## 结论

硬件约定：`load_torque_value` 是未滤波的关节扭矩传感器数值，`secondary_position_value` 是底层滤波后的同一类数值，借用了辅助位置 PDO 通道。

当前应用将两者作为同一个负载扭矩接口的可选来源，由每个驱动器的 `torque_source` 二选一。Talon 配置中 7 个关节均选择 `load_torque`，因此当前上层得到未滤波通道，滤波通道虽已绑定但未被该接口选用。

已知 bug：选择 `secondary_position` 后，应用使用位置换算函数处理滤波扭矩，将编码器比例和位置零点引入扭矩结果。此外，`GetJointTorqueFilter()` 只是负载扭矩接口的别名，不能保证返回滤波值。

## 配置解析和 PDO 读取

入口配置 [config/config.yaml](../config/config.yaml) 的 `robot.hardware_config` 指定 [hardware_talon_config.yaml](../config/hardware_talon_config.yaml)。[app_config.hpp](../src/app_config.hpp) 将相对路径解析到启动配置文件所在目录。

[hardware.cpp](../src/hardware.cpp) 中的调用链为：

```text
Hardware 构造函数
  → loadFromYAML()：YAML::LoadFile，遍历顶层 hardware 列表
  → 对 type: driver 调用 parseDrive()
  → 读取 id、torque_source、transform 和 inputs 等配置
  → buildPDOCache()：按从站 id 和 PDO 变量名缓存输入指针
  → 运行时直接解引用共享内存中的 PDO 指针
```

这两个 YAML 字段保存的是 PDO 变量名，而非传感器数值：

| 配置字段 | Talon PDO 名称 | 缓存类型 |
| --- | --- | --- |
| `inputs.load_torque_value` | `Analog Input 1` | `int16_t*` |
| `inputs.secondary_position_value` | `Auxiliary position actual value` | `int32_t*` |

`buildPDOCache()` 会尝试绑定两个已配置的通道，不受 `torque_source` 选择影响。应用在这条路径上没有实施滤波，也没有把原始通道写入滤波通道；滤波后的数据由底层提供。

## 数据源选择和换算

`parseDrive()` 仅在配置字符串精确等于 `secondary_position` 时选择滤波通道。缺失时默认 `load_torque`，其他字符串也会落入原始通道，不报非法枚举值错误。

`Hardware::GetJointLoadTorque(id)` 的实际分支如下：

| torque_source | 读取通道 | 返回公式 |
| --- | --- | --- |
| `load_torque` | `load_torque_value` | `raw / torque_per_unit` |
| `secondary_position` | `secondary_position_value` | `(raw - offset_pos_cnt) / cnt_per_unit / ratio` |

前者调用 `torqueToUnit()`，后者调用 `cntToUnit()`。驱动器 id 无效或选中通道的缓存指针为空时返回 `0.0`，没有自动切换到另一通道的机制。

当前 Talon 各轴的 `torque_per_unit` 都是 `1.0`，因此原始通道返回的是 int16 数值转 double。仅凭接口名称不能认定其已换算为 N·m，仍需核对底层数值的单位和标定。

## 滤波通道的位置换算问题

若只把 `torque_source` 改成 `secondary_position`，当前参数会产生以下计算：

| 关节 | 返回值 |
| --- | --- |
| 1～5 | `raw / 156455.678` |
| 6 | `raw / 130379.73` |
| 7 | `(raw + 133826946) / 130379.73` |

所有轴的 `ratio` 都是 `1.0`。关节 7 的 `offset_pos_cnt` 为 `-133826946`，因此即使滤波 PDO 为零，也会返回约 `1026.44`。这是位置零点污染扭矩结果的直接例子。

按两通道均为扭矩传感器数据的约定，滤波通道应采用与其底层编码匹配的扭矩比例和零点，不能直接复用位置变换。滤波前后数值是否采用相同比例、是否存在放大或定点编码，尚需核对底层实现，不能直接假定。

## 上层使用情况

| 使用位置 | 实际行为 |
| --- | --- |
| `Hardware::GetLoadTorque()` | 按模型关节顺序调用 `GetJointLoadTorque()` |
| `Robot::getJointLoadTorque()` | 从上述数组取关节值 |
| HTTP 关节状态 `load_torque` | 使用所选通道；经 Robot 状态快照输出 |
| MoveServo UDP `RobotState::load[i]` | 使用所选通道 |
| Lua `GetJointLoadTorque(index)` | 调用 Robot 负载扭矩接口，Lua 索引为 1 基 |
| Lua `GetJointTorqueFilter(index)` | 调用下述别名，结果与 `GetJointLoadTorque()` 相同 |

相关实现位于 [robot.hpp](../src/robot.hpp)、[robot.cpp](../src/robot.cpp)、[robot_http_server.cpp](../src/robot_http_server.cpp)、[move_servo.cpp](../src/move_servo.cpp) 和 [lua_script_engine.cpp](../src/lua_script_engine.cpp)。

`robot.hpp` 当前实现：

```cpp
inline double getJointTorqueFilter(int id) { return getJointLoadTorque(id); }
inline double getJointSecondaryPositionInCnt(int id) { return 0.0; }
```

因此 `GetJointTorqueFilter()` 在当前配置下仍返回未滤波通道，注释“获取滤波后的数据”与实际保证不符。`getJointSecondaryPositionInCnt()` 是固定返回零的占位实现，无法据此读取滤波 PDO 原值。目前上述接口没有分别同时提供两个传感器通道。

另一个独立通道 `torque_actual_value` 映射 `Torque actual value`，由 `Hardware::GetJointTorque()` 读取，同样使用 `torqueToUnit()`。它流向 HTTP `torque`、UDP `tau[i]` 和 Lua `GetJointTorque()`，不受 `torque_source` 控制，不能与这里的两个传感器通道混淆。

## 后续修复与验证建议

1. 核对底层两个 PDO 的实际类型、符号、单位、比例、零点和滤波实现，明确滤波值是否存在定点放大。
2. 将滤波扭矩换算与位置变换解耦，保证不受 `offset_pos_cnt`、`cnt_per_unit`、位置传动比影响；必要时引入独立标定参数。
3. 明确 `GetJointTorqueFilter()` 的接口语义：保证读取滤波通道，或按兼容性要求处理其当前别名行为；如需同时观察滤波前后数据，提供明确的双通道读取方式。
4. 补充原始/滤波通道选择、负数值、比例和零点换算、通道缺失的回归用例，尤其覆盖关节 7 非零位置偏移不能影响扭矩的场景。
5. 在准备好的硬件环境中对照 PDO 原值、HTTP、Lua 和 UDP 输出，验证单位一致及滤波效果。

本次仅记录静态代码分析及待修复问题，未修改运行行为、未启动硬件、未验证传感器实际标定。
