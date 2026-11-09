# 固定构型约束 IK 独立对照报告

日期为 2026-11-09。方法为增广雅可比矩阵法（Augmented Jacobian Method，AJM）。

## 结果摘要

方案已保存到 [固定构型约束方案](/home/rocos/Documents/GitHub/rocos-app/docs/ik_fixed_configuration_plan.md)，并在独立测试工程内完成实现。新增求解器使用固定构型约束、上一解热启动和单次同步求解，每次预算为 1 ms。

26 组轨迹对照全部完成，共 4,635,026 次外层 IK 调用。新增版本覆盖 11 组轨迹、2,107,511 次调用，成功率为 100%，测得墙钟超预算次数为 0，最大调用耗时为 192.859 μs。四项 CTest 回归通过。

1000 圈闭合轨迹中，原 pin_ik Newton 的最大同相位关节偏移为 1.012040613 rad，新版本为 1.05714×10⁻¹³ rad。新版本每周期最大关节步长为 0.00012946742 rad，构型约束最大残差为 1.49620×10⁻¹⁷ rad。

这轮验收覆盖独立 IK 的位姿精度、关节范围、单步速度连续性和构型重复性。原始候选的高阶导数另有实测结果，见后文渐入段 jerk 分析。

![1000 圈漂移与求解耗时对照](/home/rocos/Documents/GitHub/rocos-app/build/ik_fixed_configuration/comparison.png)

## 实现范围

| 文件 | 内容 |
|---|---|
| [方案文档](/home/rocos/Documents/GitHub/rocos-app/docs/ik_fixed_configuration_plan.md) | 固定构型约束、数学条件、测试与验收流程 |
| [求解器声明](/home/rocos/Documents/GitHub/rocos-app/test/ik_closed_path/fixed_configuration_ik.hpp) | 固定参考、工作区、诊断与返回码 |
| [求解器实现](/home/rocos/Documents/GitHub/rocos-app/test/ik_closed_path/fixed_configuration_ik.cpp) | 增广 Newton、截止检查、步长控制和失败处理 |
| [独立测试](/home/rocos/Documents/GitHub/rocos-app/test/ik_closed_path_test.cc) | 新旧算法统一输入、记录、几何与连续性校验、故障回归 |
| [构建入口](/home/rocos/Documents/GitHub/rocos-app/test/ik_closed_path/CMakeLists.txt) | 单独生成测试二进制，注册四项 CTest |
| [批量脚本](/home/rocos/Documents/GitHub/rocos-app/test/ik_closed_path/run_fixed_configuration.py) | 顺序执行 26 组实验并保存所有退出结果 |

测试以当前第三方库为基线，pin_ik 子模块 HEAD 为 `92e2d56185a843d48ade921809fa1fa681a34cf7`。运行进程直接调用 Pinocchio、pin_ik、TRAC-IK 和 KDL，使用同一 Talon URDF。生产源码与第三方源码的前后 SHA-256 校验见最终核验文件。

## 方程与求解流程

会话开始时，从初始构型 q_a 的 Jacobian 求单位零空间方向 c，固定参考条件为

```math
J(q_a)c=0,\qquad \|c\|=1,\qquad h(q)=c^T(q-q_a)=0
```

每周期以上一接受解热启动，内部同时求解六维位姿误差和一个构型误差。使用 M=T(q)⁻¹T_d、e=log6(M) 和 LOCAL Jacobian，增广迭代为

```math
\begin{bmatrix}
Jlog6(M^{-1})J_{local}\\
c^T
\end{bmatrix}\Delta q
=
\begin{bmatrix}
e\\-h(q)
\end{bmatrix}
```

采用固定大小 7×7 SVD 求解和回溯步长控制。Data、配置向量与矩阵工作区由实例持有。每周期一次外层调用，内部允许多次 Newton 更新与残差检查。本次全部新版本轨迹每步最多接受一次 Newton 更新，保持点使用零次更新。

预算从 Solve 入口计时，FK、Jacobian、SVD、回溯和成功提交前均检查截止时间。失败路径保留调用方的输出向量。独立验证用的共同 KDL FK 与 Jacobian 计算位于求解计时之外。

### 测试参数选择

| 参数 | 取值与说明 |
|---|---|
| 单次预算 | 全部轨迹用例均为 1 ms |
| 位姿容差 | 主测试 1e-5，位置单位 m、旋转单位 rad，补充对照 1e-6 |
| 构型容差 | 1e-10 rad，独立于位姿容差 |
| 数值尺度 | 平移特征长度为 1 m |
| 增广退化阈值 | 最小奇异值小于 1e-4 时拒绝，属于实验数值阈值 |
| Newton 更新上限 | 32 次 |
| 回溯次数上限 | 每次更新最多 16 次 |
| 内部单次更新幅度 | 最大单关节 0.05 rad，最终候选另外按 URDF 速度上限验证 |
| 轨迹非奇异检查 | 共同 KDL Jacobian 最小奇异值大于 0.2，仅用于该固定测试邻域 |

位置、旋转、构型残差均使用绝对判定。测试同时检查所有候选关节处于 URDF 范围内，以及 `abs(q[k]-q[k-1])/dt` 满足 URDF 速度限制。

## 固定输入与运行环境

- Talon URDF 的 `base_link` 到 `link_7` 七自由度链。
- 起点 rad 为 `[0.0151544516013, 0.785398163397, 0.0024351944581, -1.570796326795, -0.0188807465332, 1.570796326795, -0.0017947575133]`。
- 主轨迹为固定姿态的 XYZ 正弦，幅值 10、5、2.5 mm，相位 0、π/2、π/4，频率 1 Hz。
- 前 5 s 使用 sin² 幅值渐入，之后记录整圈变化。主采样间隔为 1 ms，补充 2 ms 与 0.5 ms 采样。所有求解预算均保持 1 ms。
- 位姿联动对照在主轨迹上增加绕局部归一化 `[1,2,3]` 轴的 0.03 rad 正弦转动。
- J=10000 为现有 App 配置基线，本实验只生成 IK 候选。高阶导数通过日志离线差分计算。
- Linux 普通调度，Intel i7-1260P，Release，`cmake --build ... -j1` 单核编译。全部用例顺序执行。
- 轨迹时间为虚拟时间。1000 圈加渐入共 1005 s 采样，新版本实际运行约 24.984 s。

## 1000 圈主对照

“同相位偏移”以 t=5 s 的关节角为基准，取每圈检查点各关节偏移的最大绝对值。“全圈逐点差”比较第二个完整周期与最后一个完整周期的每个对应采样点。

| 指标 | 原 pin_ik Newton | 增广雅可比版本 |
|---|---:|---:|
| 调用数 | 1,005,001 | 1,005,001 |
| 最大同相位关节偏移 rad | 1.01204061308 | 1.05714048626e-13 |
| J1 最终偏移 rad | -1.01204061308 | -6.9468736319e-14 |
| 全圈逐点最大关节差 rad | 1.02045300468 | 2.209343819e-14 |
| 最大单步关节变化 rad | 0.00013873127936 | 0.00012946741958 |
| 最大位置残差 μm | 9.99568075 | 9.99568075 |
| 最大旋转残差 rad | 1.2347177e-08 | 5.77235151e-09 |
| 最小任务 Jacobian 奇异值 | 0.228195501 | 0.232501034 |
| 最大构型约束偏离 rad | 1.39706868 | 1.496199e-17 |
| 求解 p50 μs | 2.546 | 14.379 |
| 求解 p99 μs | 4.385 | 21.049 |
| 求解 p999 μs | 7.205 | 29.753 |
| 求解 max μs | 98.196 | 192.859 |
| 超过 1 ms 次数 | 0 | 0 |

新版本的最小增广奇异值为 0.23250088，全部关节的逐圈线性拟合斜率绝对值最大约 6.66e-18 rad/圈。原版本的对应最大斜率约 0.00100635512 rad/圈。

上述极小的关节差是数值求解的重复性指标，物理机器人的定位精度需要依照实际反馈另行测量。

## 100 圈重复对照与连续性

三轮相同输入重复实验中，原 Newton 的最大同相位偏移均为 0.101470336 rad，新版本均为 1.05655×10⁻¹³ rad。新版本的最长求解耗时分别为 157.137、52.833、84.424 μs。

| 100 圈第 1 轮 | 原 Newton | 增广雅可比版本 |
|---|---:|---:|
| 最大单步关节变化 rad | 0.0001294450679 | 0.0001294674196 |
| p99 单步关节变化 rad | 0.0001289256092 | 0.000129400606 |
| 最大候选速度 rad/s | 0.1294450679 | 0.1294674196 |
| URDF 速度上限最大占用比例 | 0.04314835595 | 0.04315580653 |

所有用例通过候选关节范围和单步速度检查。Speed 三轮也均成功返回，但有两次完整调用墙钟耗时超过 1 ms，最大为 1537.280 μs。新版本全部 2,107,511 次轨迹调用的墙钟耗时均低于 1 ms。普通操作系统的后续抢占和系统负载仍需在对应部署环境中测量。

## 路径和精度对照

下表为每组 100 圈的最大同相位关节偏移。

| 对照 | 原 Newton rad | 增广雅可比 rad |
|---|---:|---:|
| 正向闭合轨迹 | 0.101470336 | 1.05655068e-13 |
| 反向闭合轨迹 | 0.102968581 | 1.05401798e-13 |
| 直线往返 | 0.000111501553 | 3.55271368e-15 |
| 固定目标 | 0 | 0 |
| 位置姿态联动 | 0.0587528356 | 6.85441287e-14 |
| 每秒 500 点 | 0.101475468 | 8.46066273e-13 |
| 每秒 2000 点 | 0.101467271 | 1.32081845e-14 |
| 位姿容差 1e-6 | 0.101470336 | 1.05655068e-13 |

TRAC-IK 原始 Newton 的 100 圈交叉参考也完成，最大同相位关节偏移为 0.101470430 rad，与此前独立结论一致。

## 渐入段原始候选的 jerk

日志按以下离散定义计算候选导数。

```math
v_k=(q_k-q_{k-1})/\Delta t
```

```math
a_k=(v_k-v_{k-1})/\Delta t,\qquad j_k=(a_k-a_{k-1})/\Delta t
```

| 100 圈主输入 | 最大原始加速度 rad/s² | 最大原始 jerk rad/s³ | 超过 J=10000 的帧数 |
|---|---:|---:|---:|
| 原 Newton，eps=1e-5 | 44.413912 | 88825.919970 | 936 |
| 增广雅可比，eps=1e-5 | 44.418515 | 88835.164254 | 936 |
| 增广雅可比，eps=1e-6 | 3.675672 | 7255.825502 | 0 |

两版 eps=1e-5 主测试的超限均发生在 0.113～1.798 s 渐入区间。渐入结束后的最大候选 jerk 分别约 5.15850、5.15967 rad/s³。

在新版本第 1793、1795 步，位置残差分别约 9.882、9.951 μm，满足 10 μm 容差，求解器直接接受上一关节角，Newton 更新次数为 0。随后第 1794、1796 步分别更新 J3 约 4.442×10⁻⁵ rad。这种保持点与更新点交替形成三阶差分尖峰。收紧至 1 μm 容差的对照中，最大候选 jerk 降至约 7255.826 rad/s³。

因此，固定构型约束已通过当前周期重复性和位置连续性实验。后续下发参考的加速度、jerk 约束需要结合位姿容差和现有限速流程继续验证。主测试容差与所有原始尖峰均保留在结果中。

## 故障回归与返回码

独立回归验证 NaN 初值、越界初值、非法旋转矩阵、不可达目标和奇异起点。检查失败后输出向量与固定参考的完整性，并确认随后合法保持目标仍可成功求解。全部检查通过。

| 返回值 | 名称 |
|---|---|
| 1 | success |
| -3 | deadline_exceeded |
| -4 | seed_joint_limit |
| -5 | augmented_jacobian_singular |
| -6 | line_search_stalled |
| -7 | invalid_input |
| -8 | iteration_limit |

不可达目标的回归要求明确失败且保留原输出，具体失败原因允许由停滞、迭代上限或预算检查决定。已记录的正常轨迹全数成功。

## 完整实验清单

全部采用 1 ms 求解预算。表中“最大偏移”是最大同相位关节偏移。

| 用例 | 调用数 | 最大偏移 rad | p99 μs | 最大 μs | 超预算次数 |
|---|---:|---:|---:|---:|---:|
| `pin_anchored_ellipse_c1000_n1000_e1e-5_r0` | 1005001 | 1.05714e-13 | 21.049 | 192.859 | 0 |
| `pin_anchored_ellipse_c100_n1000_e1e-5_r0` | 105001 | 1.05655e-13 | 21.208 | 157.137 | 0 |
| `pin_anchored_ellipse_c100_n1000_e1e-5_r1` | 105001 | 1.05655e-13 | 19.430 | 52.833 | 0 |
| `pin_anchored_ellipse_c100_n1000_e1e-5_r2` | 105001 | 1.05655e-13 | 19.746 | 84.424 | 0 |
| `pin_anchored_ellipse_c100_n1000_e1e-6_r0` | 105001 | 1.05655e-13 | 21.798 | 120.939 | 0 |
| `pin_anchored_ellipse_c100_n2000_e1e-5_r0` | 210001 | 1.32082e-14 | 19.216 | 77.614 | 0 |
| `pin_anchored_ellipse_c100_n500_e1e-5_r0` | 52501 | 8.46066e-13 | 18.733 | 56.546 | 0 |
| `pin_anchored_hold_c100_n1000_e1e-5_r0` | 105001 | 0 | 10.610 | 179.455 | 0 |
| `pin_anchored_line_c100_n1000_e1e-5_r0` | 105001 | 3.55271e-15 | 19.824 | 149.070 | 0 |
| `pin_anchored_pose_c100_n1000_e1e-5_r0` | 105001 | 6.85441e-14 | 20.535 | 137.023 | 0 |
| `pin_anchored_reverse_c100_n1000_e1e-5_r0` | 105001 | 1.05402e-13 | 18.796 | 159.478 | 0 |
| `pin_newton_ellipse_c1000_n1000_e1e-5_r0` | 1005001 | 1.01204 | 4.385 | 98.196 | 0 |
| `pin_newton_ellipse_c100_n1000_e1e-5_r0` | 105001 | 0.10147 | 4.518 | 31.226 | 0 |
| `pin_newton_ellipse_c100_n1000_e1e-5_r1` | 105001 | 0.10147 | 4.265 | 165.251 | 0 |
| `pin_newton_ellipse_c100_n1000_e1e-5_r2` | 105001 | 0.10147 | 3.715 | 54.504 | 0 |
| `pin_newton_ellipse_c100_n1000_e1e-6_r0` | 105001 | 0.10147 | 4.435 | 18.491 | 0 |
| `pin_newton_ellipse_c100_n2000_e1e-5_r0` | 210001 | 0.101467 | 4.890 | 163.704 | 0 |
| `pin_newton_ellipse_c100_n500_e1e-5_r0` | 52501 | 0.101475 | 3.712 | 24.280 | 0 |
| `pin_newton_hold_c100_n1000_e1e-5_r0` | 105001 | 0 | 1.018 | 12.440 | 0 |
| `pin_newton_line_c100_n1000_e1e-5_r0` | 105001 | 0.000111502 | 3.624 | 24.775 | 0 |
| `pin_newton_pose_c100_n1000_e1e-5_r0` | 105001 | 0.0587528 | 4.125 | 24.404 | 0 |
| `pin_newton_reverse_c100_n1000_e1e-5_r0` | 105001 | 0.102969 | 3.832 | 193.926 | 0 |
| `pin_speed_ellipse_c100_n1000_e1e-5_r0` | 105001 | 0.101468 | 27.455 | 1042.143 | 1 |
| `pin_speed_ellipse_c100_n1000_e1e-5_r1` | 105001 | 0.101468 | 27.110 | 1537.280 | 1 |
| `pin_speed_ellipse_c100_n1000_e1e-5_r2` | 105001 | 0.101467 | 27.252 | 901.300 | 0 |
| `trac_newton_ellipse_c100_n1000_e1e-5_r0` | 105001 | 0.10147 | 16.988 | 171.909 | 0 |

## 复现

从仓库根目录执行，复用现有 Pinocchio 和 NLopt 依赖构建目录。

```bash
cmake -S test/ik_closed_path -B build/ik_closed_path_comparison -DCMAKE_BUILD_TYPE=Release
cmake --build build/ik_closed_path_comparison -j1
ctest --test-dir build/ik_closed_path_comparison --output-on-failure

python3 test/ik_closed_path/run_fixed_configuration.py \
  --output build/ik_fixed_configuration/retest
```

批量脚本要求输出目录尚未存在，以保留此前实验。任何用例失败都会保留 CSV、JSON、doctest 日志并使批量脚本最终返回非零。

单独运行 1000 圈增广雅可比对照。

```bash
IK_DRIFT_BACKEND=anchored IK_DRIFT_CYCLES=1000 IK_DRIFT_TIMEOUT_MS=1 \
IK_DRIFT_OUTPUT=build/ik_fixed_configuration/single_retest \
./build/ik_closed_path_comparison/bin/pin_ik_closed_path_test \
  '--test-case=Standalone Talon closed path IK comparison'
```

## 原始资料

- [逐周期 CSV、JSON 与日志](/home/rocos/Documents/GitHub/rocos-app/build/ik_fixed_configuration/results)
- [完整统计与差分结果](/home/rocos/Documents/GitHub/rocos-app/build/ik_fixed_configuration/analysis.json)
- [26 组执行状态](/home/rocos/Documents/GitHub/rocos-app/build/ik_fixed_configuration/results/runs.json)
- [单核编译日志](/home/rocos/Documents/GitHub/rocos-app/build/ik_fixed_configuration/final_build.log)
- [四项 CTest 结果](/home/rocos/Documents/GitHub/rocos-app/build/ik_fixed_configuration/final_ctest.log)
- [源码指纹与最终核验](/home/rocos/Documents/GitHub/rocos-app/build/ik_fixed_configuration/final_verification.json)

本次结果的适用范围为上述模型、固定参考和非奇异轨迹邻域。较大运动范围需验证构型约束的可行性和增广矩阵条件，在线参考与实际反馈的验证属于后续阶段。
