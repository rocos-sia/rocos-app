# librocos 关节伺服 Franka 方案对比测试报告

## 结果摘要

在线测试于 **2026-11-10 00:27:43 至 00:44:12，Asia/Shanghai** 完成。600 s 连续运动、60 次启停及辅助验证共 **86 个会话**，全部正常停止，故障码为 0。

- 连续运动最大关节 RMS 误差由 **3.7540 mrad 降至 1.7994 mrad，下降 52.1%**，峰值由 **6.7644 mrad 降至 3.3071 mrad**。
- 限速环节移位拟合滞后由 **31.5 ms 降至 0.0 ms**，拟合分辨率为 0.5 ms。包含一阶低通的原始目标到参考总滞后拟合为 **1.7 ms**。
- 实际反馈相对输出的拟合滞后为 **27.5 ms**，两轮相同，当前主要误差仍来自输出到实际反馈的响应。
- 60 次启停后，J2 累计偏移由 **−120 个计数降至 0**，J4、J6 也为 0。保留旧版相对原点逻辑的额外 20 次启停，各轴累计偏移同样为 0。
- **48 项伺服回归通过，另有 2 项原有低 jerk 阶跃回归失败**。连续轨迹在线结果与这一兼容性问题均纳入本报告。

## 修改内容

本轮统一关节位置伺服的参考处理，修复位置计数转换中的向零截断，并让重复运动使用固定任务原点。修改涉及 ROCOS-App、librocos examples 和相邻的 rocos-mujoco 仓库。Talon 各轴伺服 jerk 上限为 **10000 rad/s³**。

| 位置 | 本轮修改 |
|---|---|
| `src/model_limits.cpp` | `LimitServoPositionStep` 以目标位置差分生成期望速度，调用既有逐周期速度、加速度、jerk 限制器 |
| `src/servo_signal_processor.cpp` | 关节一阶低通以此前已接受参考 `q_d` 为历史，关节与笛卡尔 IK 后的关节限速统一调用上述入口 |
| `src/hardware.cpp` | 位置下发采用最近整数取整，检查有限值、编码器范围和物理边界，全轴及联动轴预检后写入 PDO |
| `rocos-mujoco/src/mujoco_simulator.cpp` | 位置反馈采用最近整数取整，计数转换检查有限值与 int32 范围 |
| `librocos/examples/common.hpp`、`generate_consecutive_motions.cpp` | 连续点到点示例保存任务原点，每段从当前参考平滑运动到绝对目标 |
| `test/librocos_online_test.cc` | 多会话周期测试使用一次确定的任务原点 |
| 回归测试 | 增加连续可行轨迹的无跟踪律滞后测试，以及 App/MuJoCo 计数取整、非法输入和批量输出拒绝测试 |

### 逐周期计算

设原始关节目标为 \(u_k\)，上一周期已接受的位置、速度、加速度分别为 \(q_{k-1},v_{k-1},a_{k-1}\)，周期为 \(h\)。一阶低通及期望速度为

\[
\alpha=\frac{h}{h+1/(2\pi f_c)},\qquad
q_{f,k}=q_{k-1}+\alpha(u_k-q_{k-1}),\qquad
v_k^*=\frac{q_{f,k}-q_{k-1}}{h}.
\]

随后计算 jerk 请求并约束加速度

\[
j_k^*=\frac{(v_k^*-v_{k-1})/h-a_{k-1}}{h},\qquad
\widetilde a_k=a_{k-1}+h\,\operatorname{clip}(j_k^*,-J,J).
\]

沿用 Model 中的位置相关速度边界 \(v_{\min},v_{\max}\)，构造允许的加速度区间

\[
a_{\min}=\max\left(-A,a_{k-1}-Jh,\frac{J}{A}(v_{\min}-v_{k-1})\right),
\]

\[
a_{\max}=\min\left(A,a_{k-1}+Jh,\frac{J}{A}(v_{\max}-v_{k-1})\right).
\]

区间可行时积分

\[
a_k=\operatorname{clip}(\widetilde a_k,a_{\min},a_{\max}),\quad
v_k=v_{k-1}+ha_k,\quad q_k=q_{k-1}+hv_k.
\]

若当前请求及其停止轨迹均可行，且导数限制未介入，则 \(q_k=q_{f,k}\)。这样消除了此前三阶反馈项对平滑目标持续产生的额外滞后。100 Hz 一阶低通在 1 ms 递推周期下仍有约 **1.5915 个控制周期**的低频群延迟。

libfranka 的关节位置路径同样采用位置差分、jerk 限制和积分，其低通在初始化后使用机器人返回的 `q_d`。本实现还保留了 jerk 区间交集、模型位置边界、停止可行性、首点容差、实际反馈和通信保护。公式对应可行区间内的处理，区间或制动检查失败时继续执行既有保护分支。[libfranka 限速源码](https://github.com/frankarobotics/libfranka/blob/main/src/rate_limiting.cpp) · [libfranka 控制循环](https://github.com/frankarobotics/libfranka/blob/main/src/control_loop.cpp)

### 计数量化与任务原点

位置转换使用

\[
n=\operatorname{round}(qC)+n_0,\qquad C=\text{cnt\_per\_unit}\times\text{ratio}.
\]

在边界处选择物理范围内可表示的整数计数。合法位置若无法表示，转换抛出异常，由既有控制线程故障处理接管。App 的全轴预检可避免第二个轴转换失败时留下部分更新的 PDO。

此前接近整数的浮点结果，例如 `122867.99999999991`，经向零截断会少一个计数。App 下发与 MuJoCo 反馈两处截断叠加，再以实际反馈重建下一次任务原点，会累计位置偏差。最近整数取整处理这类浮点尾差，固定任务原点则让连续会话保持共同的返回目标。

## 测试条件与统计方法

基线来自 2026-11-09 的同机在线测试。本轮按单核方式编译 App、SDK、测试和 MuJoCo，使用唯一一套 App/MuJoCo。Apifox 在线 RCI 契约已刷新并读取。

| 项目 | 两轮共同条件 |
|---|---|
| 模型与硬件 | Talon 7 轴，位置模式，URDF、hardware.yaml 和 MuJoCo talon.xml 的 SHA256 相同 |
| 初始姿态附近 | J2≈45°、J4≈−90°、J6≈90°，通过显式 MoveJ 定位 |
| 伺服限制 | V=3 rad/s，A=10 rad/s²，J=10000 rad/s³ |
| 低通 / 限速 | 开启 / 开启，截止频率 100 Hz |
| 名义周期 | 控制递推、SDK 发送和 MuJoCo 物理步长均为 1 ms |
| 通信保护 | 连续缺失限制 20 周期，最低接收比例 0.9，低质量累计限制 100 周期 |
| 调度 | App RR20、CPU 2–7，客户端 FIFO30、CPU 1，MuJoCo 普通调度并开启可视化 |
| 采集 | UDP 双向抓包、逐周期 trace、每秒 HTTP 状态和进程资源采样 |

使用与基线相同的七轴周期目标

\[
q_{c,i}(t)=q_{0,i}+(-1)^i A\left(\frac{1-\cos(2\pi t/T)}{2}\right)^2,
\qquad i=0,\ldots,6.
\]

600 s 长测采用 A=0.05 rad、T=2 s，共 300 圈，随后保持 0.5 s。60 次启停采用 A=0.03 rad、T=2 s，每个会话运动 4 s、保持 0.5 s，再等待受控停止。另运行往返示例、四段连续运动示例，以及保留旧版逐会话相对原点逻辑的 20 次辅助启停，用于检查保持逐会话相对原点时是否仍出现累计偏移。

本轮显式定位的实际反馈与基线长测首帧最多相差 **3.07×10⁻⁵ rad，约 0.00176°**。本轮离线统计在全部运动完成后执行。基线的部分启停阶段曾与 CPU 12 上的离线统计并行，因此通信极值对比包含这一负载差异。

抓包按会话与 `cycle` 去重，再按 `applied_message_id` 对齐实际应用的原始目标。定义

\[
e_{total}=q-q_c,\qquad e_{processing}=q_d-q_c,\qquad
e_{following}=q-q_{cmd}.
\]

表中 RMS 为最大关节 RMS，P99 为逐帧最大关节绝对误差的 P99。多会话行取各会话统计量的最大值。滞后采用时间移位最小均方误差拟合，搜索分辨率为 0.5 ms。新低通的原始目标重建使用此前已接受的 `q_d`，与本轮实际递推一致。

## 600 秒连续运动对比

本轮回调收到 **550,069** 个新状态，累计采样时间 **600.501 s**，`control()` 墙钟时间 **600.513 s**。抓包获得 **550,052** 个有效去重输出样本，实际应用序号匹配覆盖率为 100%。实际新状态回调频率约为 **916 Hz**。

| 指标 | 修改前 | 修改后 |
|---|---:|---:|
| 总误差最大关节 RMS | 3.7540 mrad | **1.7994 mrad** |
| 总误差 P99 | 6.2540 mrad | **3.0174 mrad** |
| 总误差峰值 | 6.7644 mrad | **3.3071 mrad** |
| 原始目标→参考 RMS | 2.0698 mrad | **0.1103 mrad** |
| 原始目标→参考 P99 | 3.4748 mrad | **0.2667 mrad** |
| 原始目标→参考峰值 | 3.8595 mrad | **0.3678 mrad** |
| 输出→实际 RMS | 1.6896 mrad | 1.6909 mrad |
| 输出→实际 P99 | 2.8058 mrad | 2.8161 mrad |
| 输出→实际峰值 | 2.9470 mrad | 3.0240 mrad |
| 参考→最终下发偏差 | 0 | 0 |
| 一阶低通偏差峰值 | 0.4254 mrad | 0.3128 mrad |
| 限速处理偏差峰值 | 3.5989 mrad | **0.0683 mrad** |
| 限速环节拟合滞后 | 31.5 ms | **0.0 ms** |
| J2 输出→实际拟合滞后 | 27.5 ms | **27.5 ms** |
| 600 s 首尾反馈最大差 | 7.67×10⁻⁶ rad | **0 个反馈计数** |
| 周期峰值波动范围最大值 | 4.60×10⁻⁵ rad | 3.07×10⁻⁵ rad |
| 周期谷值波动范围最大值 | 3.83×10⁻⁵ rad | 3.07×10⁻⁵ rad |
| 最低新指令接收比例 | 0.96 | 0.98 |
| 运动故障数 | 0 | 0 |

本轮总误差峰值发生在 J2，同一时刻总误差为 **−3.3071 mrad**，参考处理误差为 **−0.3078 mrad**，实际跟随误差为 **−2.9993 mrad**。该时刻约 **90.7%** 的误差来自输出到实际反馈。处理链 RMS 降低约 **94.7%**，实际跟随 RMS 两轮接近。

0.0 ms 是当前移位搜索的最佳网格点，网格分辨率为 0.5 ms。限速器仍会在约束介入时修改参考，长测中其最大偏差为 0.0683 mrad。对 J2 的原始目标到处理后参考进一步以 0.1 ms 网格拟合，得到 **1.7 ms**，与 100 Hz 一阶低通及实际控制节拍相符。[本轮连续运动统计](/home/rocos/Documents/GitHub/rocos-app/build/librocos_franka_20261110/07_periodic_600s/analysis.json) · [总参考滞后拟合](/home/rocos/Documents/GitHub/rocos-app/build/librocos_franka_20261110/reference_delay.json)

### 导数限制的实际介入

| 正常参考指标 | 修改前 | 修改后 | 配置上限 |
|---|---:|---:|---:|
| 最大速度 rad/s | 0.11849 | 0.14889 | 3 |
| 最大加速度 rad/s² | 0.70340 | **10.0** | 10 |
| 最大 jerk rad/s³ | 97.0968 | **10000.0** | 10000 |
| 正常运动制动回退帧数 | 0 | 0 | — |

新方案对逐周期目标变化更敏感。本轮 550,817 个正常参考周期中，**49,752 帧，约 9.03%** 至少一轴触及加速度上限，**77,815 帧，约 14.13%** 触及 jerk 上限，**111,183 帧，约 20.18%** 的限速输出与滤波目标相差超过 10⁻¹² rad。按固定 h 做位置差分会放大非均匀更新引起的周期差异，这与新方案中的导数限制介入相符。逐周期参考的导数波动需要与跟踪误差下降共同评估。[导数饱和统计](/home/rocos/Documents/GitHub/rocos-app/build/librocos_franka_20261110/derivative_saturation.json)

全阶段成功 trace，包括停止周期，速度、加速度、jerk 越限数均为 0。正常运动期间可停止性回退次数为 0，原有位置边界、首点、反馈和通信保护继续执行。

## 60 次启停与累计偏移

| 指标 | 修改前 60 次 | 修改后 60 次 |
|---|---:|---:|
| 正常停止会话数 | 60 | 60 |
| 有效输出样本 | 247,469 | 247,308 |
| 最差会话最大关节 RMS | 2.1391 mrad | **1.0263 mrad** |
| 最差会话误差 P99 | 3.8188 mrad | **1.8938 mrad** |
| 全阶段误差峰值 | 3.9508 mrad | **1.9997 mrad** |
| 最低新指令接收比例 | 0.92 | 0.97 |
| J2 累计偏移 | −0.000766990 rad，−120 计数 | **0 计数** |
| J4 累计偏移 | +0.000383495 rad，+60 计数 | **0 计数** |
| J6 累计偏移 | −0.000460194 rad，−60 计数 | **0 计数** |
| 其余关节累计偏移 | 0 | **0 计数** |

修改后 60 个会话的每次末端反馈均回到同一整数计数位置，各轴端点最大偏移也是 0。原始目标的首尾差和跨会话原点变化均为 0。

额外 20 次启停恢复了基线中的“每次从本会话首帧 `q_d` 建立相对原点”逻辑，使用修复后的 App 与 MuJoCo。20 次均正常停止，各轴累计偏移、各次端点偏移和原点变化均为 **0 计数**。该对照说明，本轮即使继续逐会话重建原点，也没有复现此前每次少计数的现象。[启停与对照汇总](/home/rocos/Documents/GitHub/rocos-app/build/librocos_franka_20261110/comparison_summary.json)

## 通信、保护与资源结果

| 阶段 | 会话数 | 有效状态样本 | 最低接收比例 | 发送间隔 P99 / 峰值 ms | 新状态间隔 P99 / 峰值 ms |
|---|---:|---:|---:|---:|---:|
| 往返 example | 1 | 3,846 | 1.00 | 1.028 / 1.064 | 1.223 / 2.107 |
| 连续运动 example | 4 | 15,578 | 0.98 | 1.053 / 1.256 | 1.291 / 4.313 |
| 600 s 连续运动 | 1 | 550,052 | 0.98 | 1.050 / 1.429 | 1.274 / 3.920 |
| 固定原点启停 | 60 | 247,308 | 0.97 | 1.189 / 1.473 | 1.390 / 5.123 |
| 相对原点对照 | 20 | 82,491 | 0.96 | 1.192 / 1.466 | 1.350 / 4.499 |

- 合计 **899,275** 个有效去重输出样本，应用序号全部匹配。86 个会话故障码均为 0，终态均为 `STOPPED`。
- 命令序号断点、回包序号断点、UDP 故障标志帧、tcpdump 内核丢包均为 0。
- 共保存 **900,925** 条 trace，dropped 为 0，io_error 为 false。
- 实际位置、参考位置和实际速度的配置边界越限样本为 0，所有有效输出中 `q_cmd=q_d`。
- 600 s 的参考处理耗时 P99 为 **77.501 μs**，峰值为 **134.155 μs**。这项统计覆盖信号处理函数。
- App RSS 在长测末期为 **48.4 MiB**，启停后升至 **166.9 MiB**，在后续 20 次辅助启停中保持该值。App 线程数为 42～44，MuJoCo 为 18，MuJoCo RSS 约 208.2 MiB。重复启停带来的 RSS 增长仍存在，本轮未分析其分配归属。

`control_command_success_rate` 沿用新指令接收比例语义。测试结束后关闭 RCI、下使能，机器人处于 **IDLE**。App PID **1095298**、MuJoCo PID **1095278** 继续运行。[最终状态](/home/rocos/Documents/GitHub/rocos-app/build/librocos_franka_20261110/final_state.json)

![关节误差、环节滞后和启停累计偏移对比](/home/rocos/Documents/GitHub/rocos-app/build/librocos_franka_20261110/comparison.png)

## 离线回归与适用范围

App、SDK 和 MuJoCo 单核编译成功。七个选定 CTest 目标通过，包含 `librocos_test`、`librocos_robot_test`、`servo_robot_test`、`position_controller_test`、`http_control_rights_test`、`servo_trace_test` 和 `hardware_test`。MuJoCo 的 `mujoco_shm_v2` 通过。

伺服回归中，48 个用例、2,539,530 个断言通过。新增可行周期轨迹测试在 0.5、1、2 ms 周期及 J=10000 下，直接调用 Model 限速入口，目标与输出误差小于 10⁻¹² rad，并通过各步停止可行性检查。

**两项原有低 jerk 阶跃回归失败，当前整套伺服回归尚未全绿。**

| 原有用例 | 已记录的失败 |
|---|---|
| `Servo regression small targets converge without self-excited motion` | J=150 的 0.00297 rad 突变目标出现 0.00312075 rad 输出，超过原有 0.00300001 rad 范围断言 |
| `Servo adaptive tracking settles repeated small targets and reversals` | 小阶跃目标 10⁻⁶ rad 出现约 1.425×10⁻⁶ rad 输出，超过原有 1.1×10⁻⁶ rad 断言 |

这两项仍使用原始断言，并保存失败日志。逐周期导数约束允许系统在位置目标附近仍带有速度和加速度，因而突变目标可以出现超调。指定连续轨迹的在线通过结果只覆盖本报告的轨迹、限制和环境。针对任意阶跃的单调收敛要求仍存在兼容性问题。

初次 MuJoCo 新增转换测试使用了负数主站 ID，触发构造参数检查。改为独立正数 ID 后重新单核编译并通过，该问题发生在测试初始化阶段。

## 复现与原始证据

```bash
cmake --build build --target rocosAppMain move_servo_test servo_robot_test \
  librocos_test librocos_online_test --parallel 1
cmake --build ../rocos-mujoco/build --parallel 1

./build/bin/librocos_online_test 600 2 0.05 1
./build/bin/librocos_online_test 4 2 0.03 60
```

在线命令在已定位、使能且处于 `STOPPED` 的仿真环境中运行，实时调度前缀见各阶段 `result.json`。本轮辅助相对原点程序保存在测试数据目录，并附源码。

- [基线报告](/home/rocos/Documents/GitHub/rocos-app/docs/librocos_joint_servo_online_test_report.md)
- [本轮数据与执行脚本](/home/rocos/Documents/GitHub/rocos-app/build/librocos_franka_20261110)
- [有效配置](/home/rocos/Documents/GitHub/rocos-app/build/librocos_franka_20261110/effective/effective.yaml)
- [编译与回归日志](/home/rocos/Documents/GitHub/rocos-app/build/librocos_franka_20261110/logs)
- [App 与 MuJoCo 文件哈希](/home/rocos/Documents/GitHub/rocos-app/build/librocos_franka_20261110/sha256.json)
