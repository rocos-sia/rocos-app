# pin_ik 与 TRAC-IK 独立闭合轨迹测试

测试日期为 2026-11-09。测试对象为工作区当前第三方源码，pin_ik HEAD 为 `92e2d56185a843d48ade921809fa1fa681a34cf7`。完整源码指纹见 [manifest.json](/home/rocos/Documents/GitHub/rocos-app/build/ik_closed_path_comparison/manifest.json)。

## 结论

两库均能在独立运动学测试中产生逐圈关节漂移。相同非奇异 Talon 姿态、闭合末端轨迹和上一周期解作为初值时，两库 Newton 后端运行 100 圈，J1 分别漂移 −0.101470336 rad 与 −0.101470430 rad，约 −5.814°。两者累计值相差约 9.42×10⁻⁸ rad。

纯 SQP 后端也出现同方向漂移。Distance 模式选取靠近本次初值的候选解，在本实验中仍逐圈累积偏移。反向绕圈使漂移反向，直线往返的漂移显著减小，固定目标与固定初值对照均得到零累计漂移。

这些结果支持七自由度冗余运动学的路径依赖解释。运动学层本身足以复现该现象。测试进程直接加载第三方 IK、FK 与数值库，全部结果来自离线虚拟时间输入。

## 新增文件与测试边界

- [独立测试文件](/home/rocos/Documents/GitHub/rocos-app/test/ik_closed_path_test.cc) 使用项目 doctest，记录逐步候选关节解、返回码、耗时、FK 残差和最小奇异值。
- [独立 CMake 工程](/home/rocos/Documents/GitHub/rocos-app/test/ik_closed_path/CMakeLists.txt) 编译两个可执行程序，分别链接 pin_ik 和 TRAC-IK。两库含同名 `NLOPT_IK` 符号，因此分进程运行。
- 新增内容覆盖测试与本报告。测试前后的生产源码、两库源码及 Talon 配置由 SHA-256 对照核验。
- 单核编译采用 `-j1`。所有对照顺序执行，Speed 与 Distance 保留库内双求解策略的线程行为。
- App 的 J=10000 配置保持原样。纯 IK 测试的计算输入为模型、位姿和关节初值，jerk 参数不参与计算。
- 每步虚拟时间为 1 ms，程序尽快求解全部样本。100 圈表示 100 秒的轨迹采样，加上 5 秒渐入段。实际耗时见日志。
- 本轮完成性断言覆盖返回码、有限值、关节范围、FK 残差与非奇异性。函数墙钟耗时另行记录。

## 固定实验条件

| 项目 | 设置 |
|---|---|
| 模型 | `config/robots/talon/robot.urdf`，`base_link` 到 `link_7` |
| 起始关节角 rad | `[0.0151544516013, 0.785398163397, 0.0024351944581, -1.570796326795, -0.0188807465332, 1.570796326795, -0.0017947575133]` |
| 初值规则 | 每次采用上一次成功解，固定初值对照采用上述起始角 |
| 主测试轨迹 | 固定姿态，XYZ 幅值 10、5、2.5 mm，相位 0、π/2、π/4，频率 1 Hz |
| 渐入 | 前 5 s 使用 `sin²(πt/10)` 幅值渐入 |
| 采样 | 每圈 1000 点，另测 500、2000 点 |
| 单次预算 | 主测试 1 ms，TRAC-IK 补充对照 5 ms |
| 容差 | 主测试 1e-5，另测 1e-6、1e-9 |
| 每步处理 | 第一次失败立即保存结果并结束该用例 |
| 关节范围 | 两库使用同一 URDF 的 double 精度上下界 |
| 模型校验 | 起点及邻域 100 个确定性关节样本交叉核对 FK |
| 非奇异核验 | 每步计算共同 KDL Jacobian 的最小奇异值，要求大于 0.2 |

pinocchio FK 与共同 KDL FK 的最大矩阵元素差为 2.22×10⁻¹⁶。非奇异阈值 0.2 仅用于这个固定模型、单位和测试邻域。100 圈 Newton 主测试中最小奇异值约 0.232477。

漂移以渐入结束时 t=5 s 的关节解为基准，比较以后同相位的关节解。直线往返使用 XYZ 同相位正弦，反向绕圈将主轨迹相位方向反转。

## 主测试结果

以下 J1 值为第 20 圈相对第 0 圈的累计漂移。

| 求解方式 | pin_ik | TRAC-IK |
|---|---:|---:|
| Speed | -0.020393645 rad，1 ms | -0.020383346 rad，5 ms |
| Distance | -0.020356339 rad，1 ms | -0.020307412 rad，5 ms |
| 直接 Newton | -0.020394525 rad，1 ms | -0.020394554 rad，1 ms |
| 直接 SQP | -0.019984099 rad，1 ms | -0.017328618 rad，1 ms |

TRAC-IK 的 Speed 和 Distance 主测试在 1 ms 下分别于第 2064、8933 步返回 −3。表中相应值来自独立的 5 ms 补充用例。保留了两次 1 ms 失败的全部原始记录。

| 100 圈结果 | pin_ik | TRAC-IK |
|---|---:|---:|
| 直接 Newton，均为 1 ms | -0.101470336 rad | -0.101470430 rad |
| Speed，pin 为 1 ms、TRAC 为 5 ms | -0.101467935 rad | -0.101421107 rad |

100 圈直接 Newton 测试中，最终同相位 TCP 位置与第 0 圈位置相差约 8.69×10⁻¹⁰ m。用两端关节中点处 Jacobian 投影，累计关节位移的零空间比例约 0.99999997。这里的零空间比例是有限位移的局部线性诊断。

## 定位对照

以下均使用直接 Newton、1 ms 预算，比较 20 圈的 J1 累计变化。

| 对照 | pin_ik rad | TRAC-IK rad |
|---|---:|---:|
| 正向闭合轨迹 | -0.020394525009 | -0.020394554331 |
| 反向闭合轨迹 | +0.020463773670 | +0.020463823778 |
| 直线往返 | -0.000022298699 | -0.000022338088 |
| 固定目标 | +0.000000000000 | +0.000000000000 |
| 每步使用固定初值 | +0.000000000000 | +0.000000000000 |
| 精度收紧至 1e-6 | -0.020394525006 | -0.020394552933 |
| 每圈 500 点 | -0.020394114064 | -0.020394086805 |
| 每圈 2000 点 | -0.020394629736 | -0.020394647469 |

- 反向轨迹得到约 +0.02046 rad，支持漂移随路径方向变化的解释。渐入段与随关节构型变化的 Jacobian 会使正反向数值略有差异。
- 直线往返降至约 −2.23×10⁻⁵ rad，约为正向闭合轨迹的 1/915。
- 采样密度从 500 提升到 2000，漂移仍约 −0.020394 rad。
- 容差从 1e-5 收紧到 1e-6，漂移量几乎一致。
- 固定初值对照在每圈同一目标处回到同一解，验证了本例解的路径记忆来自逐步更新的初值。该对照只验证这个小范围轨迹。

## 数学与源码对应

设任务位姿为 x=f(q)，q 有 7 个分量，完整末端位姿提供 6 个约束。在满行秩邻域，Newton 迭代近似为

```math
q_{k+1}=q_k+J(q_k)^+ e_k
```

这里 e_k 为局部位姿误差，J⁺ 为 Jacobian 的广义逆。小步误差极限下对应

```math
dq=J(q)^+dx
```

每一步的最小范数增量可以位于当时 Jacobian 的行空间。沿路径运动时 J 随 q 变化，闭合任务路径的积分一般满足

```math
\Delta q=\oint J(q)^+dx\ne 0
```

末端回到相同位姿时，七自由度机械臂仍存在冗余构型自由度。闭合路径可以积累关节构型变化，这种路径依赖也称几何相位。本例中的绕向反转、直线往返和固定初值结果与该机制一致。

pin_ik 的 Newton 后端使用 `CompleteOrthogonalDecomposition` 求解局部 Jacobian 线性方程，随后执行 `q_curr = q_out + delta_q`。TRAC-IK 的对应后端通过 KDL `ChainIkSolverVel_pinv` 求关节增量，并将增量加到当前关节角。两者均从本次提供的 seed 开始迭代。[pin Newton](/home/rocos/Documents/GitHub/rocos-app/3rdparty/pin_ik/src/pinocchio_tl.cpp:135)，[TRAC Newton](/home/rocos/Documents/GitHub/rocos-app/3rdparty/trac_ik/src/kdl_tl.cpp:103)，[KDL 后端类型](/home/rocos/Documents/GitHub/rocos-app/3rdparty/trac_ik/include/trac_ik/kdl_tl.hpp:75)

两库 Distance 的候选评分使用本次输入初值与候选解之间的关节距离

```math
E(q)=\sum_i(q_i-q_{\mathrm{seed},i})^2
```

当 seed 每周期更新为上次成功解时，该代价约束的是局部变化。整段轨迹起点的构型并未作为固定参考加入评分。因此 Distance 在本实验中仍约每圈漂移 0.001 rad。[pin 评分](/home/rocos/Documents/GitHub/rocos-app/3rdparty/pin_ik/src/pin_ik.cpp:377)，[TRAC 评分](/home/rocos/Documents/GitHub/rocos-app/3rdparty/trac_ik/src/trac_ik.cpp:263)

直接 SQP 对照采用两库常用的 `SumSq` 目标，即笛卡尔误差平方和。它们分别在 20 圈产生约 −0.01998 与 −0.01733 rad 漂移。这说明应同时关注冗余解选择和 seed 更新规则，逐圈偏移也出现在 SQP 策略中。[pin SQP 目标](/home/rocos/Documents/GitHub/rocos-app/3rdparty/pin_ik/src/nlopt_ik.cpp:161)，[TRAC SQP 目标](/home/rocos/Documents/GitHub/rocos-app/3rdparty/trac_ik/src/nlopt_ik.cpp:242)

## 完成性与耗时

共运行 26 个对照用例、906027 次求解调用。22 个用例完成，4 个用例保留失败结果。独立 CTest 的两个 Newton 冒烟用例通过。

| 未完成用例 | 首次失败步 | 返回码 | 现象 |
|---|---:|---:|---|
| TRAC Speed，1 ms，eps=1e-5 | 2064 | −3 | 渐入段未完成即退出 |
| TRAC Distance，1 ms，eps=1e-5 | 8933 | −3 | 完成 3 个完整圈后失败 |
| pin Newton，1 ms，eps=1e-9 | 2 | −3 | 极小步目标下耗尽预算 |
| TRAC Newton，1 ms，eps=1e-9 | 2 | −3 | 极小步目标下耗尽预算 |

eps=1e-9 两个失败样本位于渐入初期。源码中两库都用约 1.19×10⁻⁷ 的 float epsilon 判断关节增量停滞，并可能随机重启，这与该极小步场景有关。定位到具体迭代和重启次数仍需额外埋点。eps=1e-6 对照均完成。

TRAC-IK 两个 1 ms 用例返回 −3 说明该次调用未返回成功解。本测试只测量整体求解墙钟耗时，具体线程调度、迭代和回收的耗时占比尚未测量。

| 用例 | p50 μs | p99 μs | 最大 μs | 超过配置预算次数 |
|---|---:|---:|---:|---:|
| pin speed，1 ms | 14.804 | 46.312 | 1994.465 | 3 / 105001 |
| trac speed，1 ms | 51.311 | 176.142 | 1568.006 | 1 / 2065 |
| trac speed，5 ms | 56.371 | 188.033 | 3287.104 | 0 / 105001 |
| pin distance，1 ms | 1014.822 | 1172.288 | 2407.361 | 25001 / 25001 |
| trac distance，5 ms | 5051.386 | 5223.336 | 6405.975 | 25001 / 25001 |
| pin newton，1 ms | 2.645 | 5.489 | 28.136 | 0 / 105001 |
| trac newton，1 ms | 12.740 | 20.008 | 231.044 | 0 / 105001 |
| pin sqp，1 ms | 39.183 | 70.445 | 201.701 | 0 / 25001 |
| trac sqp，1 ms | 33.702 | 64.302 | 219.683 | 0 / 25001 |

Distance 会使用搜索时间收集候选，外层函数还有返回与线程同步开销。本次 pin Distance 的所有调用墙钟时间均超过配置的 1 ms，TRAC Distance 的 5 ms 对照也有相同现象。完成性通过与整体函数硬实时截止是不同的检查项，本报告分别给出结果。计时采集环境为 Intel i7-1260P、普通 Linux 调度，测试数据反映这次运行。

## 复现

从仓库根目录执行。此独立工程复用现有 `build/deps/prefix` 的 Pinocchio 和 `build/lib` 的 NLopt 依赖，可通过 `ROCOS_BUILD_DIR` 指定其他依赖构建目录。

```bash
cmake -S test/ik_closed_path -B build/ik_closed_path_comparison -DCMAKE_BUILD_TYPE=Release
cmake --build build/ik_closed_path_comparison -j1
ctest --test-dir build/ik_closed_path_comparison --output-on-failure

IK_DRIFT_BACKEND=newton IK_DRIFT_CYCLES=100 \
IK_DRIFT_OUTPUT=build/ik_closed_path_comparison/retest_pin \
./build/ik_closed_path_comparison/bin/pin_ik_closed_path_test

IK_DRIFT_BACKEND=newton IK_DRIFT_CYCLES=100 \
IK_DRIFT_OUTPUT=build/ik_closed_path_comparison/retest_trac \
./build/ik_closed_path_comparison/bin/trac_ik_closed_path_test
```

环境参数支持以下取值。

| 参数 | 默认值与可选值 |
|---|---|
| `IK_DRIFT_BACKEND` | 默认 `speed`，另有 `distance`、`newton`、`sqp` |
| `IK_DRIFT_SHAPE` | 默认 `ellipse`，另有 `reverse`、`line`、`hold` |
| `IK_DRIFT_SEED` | 默认 `previous`，另有 `fixed` |
| `IK_DRIFT_CYCLES` | 默认 20，自动增加 5 s 渐入 |
| `IK_DRIFT_SAMPLES` | 默认 1000，每秒采样数 |
| `IK_DRIFT_TIMEOUT_MS` | 默认 1，单位 ms |
| `IK_DRIFT_EPS` | 默认 1e-5 |
| `IK_DRIFT_OUTPUT` | CSV、JSON 输出文件前缀 |

同一输出前缀会覆盖原结果，重测应使用新前缀。测试断言失败时进程返回非零，同时保留已写入的数据和首次失败位置。

## 数据索引

- [26 组结果汇总](/home/rocos/Documents/GitHub/rocos-app/build/ik_closed_path_comparison/analysis.json)
- [主测试执行日志](/home/rocos/Documents/GitHub/rocos-app/build/ik_closed_path_comparison/matrix.log)
- [补充对照日志](/home/rocos/Documents/GitHub/rocos-app/build/ik_closed_path_comparison/controls.log)
- [逐周期 CSV、JSON、doctest 日志](/home/rocos/Documents/GitHub/rocos-app/build/ik_closed_path_comparison/results)
- [主测试批量脚本](/home/rocos/Documents/GitHub/rocos-app/build/ik_closed_path_comparison/run_matrix.py)
- [补充对照批量脚本](/home/rocos/Documents/GitHub/rocos-app/build/ik_closed_path_comparison/run_controls.py)
- [最终源码与测试核验](/home/rocos/Documents/GitHub/rocos-app/build/ik_closed_path_comparison/final_verification.json)
