# librocos 关节位置伺服在线测试报告

## 结论

测试于 **2026-11-09 23:20:06 至 23:37:19，Asia/Shanghai** 执行，保持各轴 **J=10000 rad/s³**。使用现有构建的 App 和 MuJoCo，完成五种 librocos examples、600 s 连续周期运动和 60 次重复启停，共 **69 个 RCI 运动会话**。

- 69 个会话均完成受控停止，终态为 `STOPPED`，故障码为 0。最终关闭 RCI、下使能到 `IDLE`。
- 连续运动的最大关节 RMS 误差为 **0.003754 rad，约 0.215°**，最大关节绝对误差的 P99 为 **0.006254 rad**，峰值为 **0.006764 rad，约 0.388°**。
- 连续运动的参考处理存在约 **31.5 ms** 的跟踪律滞后，实际反馈相对下发值存在约 **27.5 ms** 的滞后。一阶低通还贡献约 1.6 个控制周期的低频群延迟。
- 连续运动期间首尾反馈差最大为 **7.67×10⁻⁶ rad**。60 次相对起点重新建立的会话后，J2 累计偏移 **−0.000766990 rad，约 −0.04395°**。这一重复定位偏移需要与会话成功完成的结果同时保留。
- 最低新指令接收比例，连续运动为 **0.96**，重复启停为 **0.92**。本轮控制器通信故障次数为 0。

运动持续性、会话生命周期及当前保护配置下的运行测试完成。动态跟踪精度和反复重建起点的量化偏移仍有明确改进空间。上述数值对应本报告列明的仿真工况和调度条件。

## 环境与执行条件

开测时检测到 App 和 MuJoCo 进程均为空，随后使用原有可执行文件和此前验证过的 1 ms 仿真模型启动一套环境。源码基线为 `f7901131b7d13a436c3259f97f0d7ffeede13015`，SDK 与测试程序来自当前工作区。单核编译完成后开展在线运动，`librocos_test` 的 15 个离线用例通过。

| 项目 | 本轮配置 |
|---|---|
| 机器人 | Talon，7 轴，位置控制 |
| 初始定位 | 显式 MoveJ 到 `[0, π/4, 0, −π/2, 0, π/2, 0]` rad，等待反馈到位 |
| 定位参数 | V=0.2 rad/s，A=0.5 rad/s²，J=2 rad/s³，作为伺服前独立定位运动 |
| 伺服硬件限制 | 各轴 V=3 rad/s，A=10 rad/s²，J=10000 rad/s³ |
| 位置边界 | 使用当前 Talon URDF 与硬件配置 |
| 控制递推周期 / MuJoCo 步长 | 均配置为 1 ms |
| SDK 发送周期 | 1 ms，UDP v2，本机回环地址 |
| 一阶低通 | 开启，截止频率 100 Hz |
| 关节参考限速 | 开启，当前关节位置分支调用 `LimitServoPositionStep` |
| 通信保护 | 开启，max_missing_cycles=20，minimum_success_rate=0.9，low_quality_cycles=100 |
| 首点等待 / SDK 反馈时效 | 1000 ms / 20 ms |
| App 调度 | SCHED_RR，优先级 20，CPU 2–7 |
| 客户端调度 | SCHED_FIFO，优先级 30，CPU 1 |
| MuJoCo | 可视化开启，普通调度，共享内存 ecat_id=0 |
| 采集 | UDP 双向抓包、App 逐周期 trace、每秒 HTTP 状态和进程资源采样 |

部分已完成阶段的离线统计与后续在线运动并行，在 CPU 12、nice=10 下执行。最低接收比例 0.92 出现在重复启停的第 22 个会话，该阶段包含后台统计负载。日志保留了这一运行条件。

Apifox 在线契约于本轮开始时刷新并读取，测试通过 librocos 的 HTTP RCI 接口建立会话，通过 UDP v2 发送关节位置。有效配置、模型与可执行文件哈希随数据保存。[有效配置](/home/rocos/Documents/GitHub/rocos-app/build/librocos_online_20261109/effective/effective.yaml) · [进程与启动参数](/home/rocos/Documents/GitHub/rocos-app/build/librocos_online_20261109/processes.json) · [SHA256](/home/rocos/Documents/GitHub/rocos-app/build/librocos_online_20261109/sha256.json)

## 测试轨迹与流程

五种原有 examples 均实际运行。其中状态读取持续 3 s 后以 SIGINT 结束，`timeout` 的退出码为 124。运动示例使用 J2、0.02 rad 位移和 4 s 运动时间，包含一次反向点到点返回及四次连续独立会话。通信示例保持当前参考 120 s。

为实现连续多周期运动和完整的会话计数，增加手动在线测试程序 `test/librocos_online_test.cc`，复用往返 example 的平滑余弦形状。它通过相同的 `Robot::control(callback)` 与 `MotionFinished` 入口调用 librocos，逐周期目标为

\[
q_{c,i}(t)=q_{0,i}+s_i A\left(\frac{1-\cos(2\pi t/T)}{2}\right)^2,
\qquad s_i=(-1)^i,\quad i=0,\ldots,6.
\]

每个完整周期回到初始目标，端点速度、加速度和 jerk 均为零。

| 阶段 | 参数与结束方式 |
|---|---|
| 长时间运动 | A=0.05 rad，T=2 s，0.5 Hz，七轴交替方向，连续 300 圈，共 600 s，再保持 0.5 s |
| 重复启停 | A=0.03 rad，T=2 s，每个会话运动 4 s 并保持 0.5 s，共 60 个会话 |
| 会话起点 | 每次由首帧 `q_d` 建立相对起点 |
| 正常结束 | 最终目标应用确认后请求停止，等待 `control_active=false` 且 `STOPPED` |
| 测试运动范围 | 实际位置相对本会话起点超过 0.15 rad 时由测试程序结束该会话 |

600 s 阶段回调处理 **548,867** 个新状态样本，回调采样时间累计 **600.501 s**，一次 `control()` 的墙钟耗时 **600.520 s**。因此本轮名义发送频率为 1 kHz，实际新状态回调更新约 **914 Hz**。[在线程序](/home/rocos/Documents/GitHub/rocos-app/test/librocos_online_test.cc) · [逐阶段执行记录](/home/rocos/Documents/GitHub/rocos-app/build/librocos_online_20261109/results.json)

## 数据口径与总体结果

抓包按会话 ID 分组，按 `cycle` 去重，通过 `applied_message_id` 将反馈与实际已应用的原始指令对齐。共获得 **933,232** 个有效、去重且成功对齐的运动状态样本，69 个会话的匹配覆盖率均为 100%。完整 trace 保存 **935,130** 条记录，69 个 trace 的 dropped 均为 0、io_error 均为 false。曲线采用降采样显示，表格使用完整样本。

令 `q_c` 为已应用原始目标，`q_d` 为处理后参考，`q_cmd` 为最终下发值，`q` 为实际反馈，统计采用

\[
e_{total}=q-q_c,\quad e_{processing}=q_d-q_c,\quad
 e_{following}=q-q_{cmd}.
\]

表中的 RMS 为最大关节 RMS，P99 为逐帧最大关节绝对误差的 P99。含多个会话的行取各会话统计量的最大值，单位均为 **mrad**。通信比例从应用序号超过 100 后统计，启动保持帧单独排除。

| 阶段 | 会话数 | 有效状态样本 | 最差关节 RMS mrad | P99 mrad | 峰值 mrad | 最低接收比例 | 结束状态 |
|---|---:|---:|---:|---:|---:|---:|---|
| 关节往返 example | 1 | 3,816 | 0.7365 | 1.2560 | 1.2924 | 0.9700 | 正常停止 |
| 点到点 example | 1 | 3,816 | 0.3856 | 0.6715 | 0.6852 | 0.9900 | 正常停止 |
| 点到点返回 | 1 | 3,813 | 0.3757 | 0.6935 | 0.7134 | 1.0000 | 正常停止 |
| 连续运动 example | 4 | 15,531 | 0.3805 | 0.6793 | 0.6988 | 0.9892 | 正常停止 |
| 保持通信 120 s | 1 | 109,935 | 0.0077 | 0.0077 | 0.0128 | 0.9800 | 正常停止 |
| 七轴周期运动 600 s | 1 | 548,852 | 3.7540 | 6.2540 | 6.7644 | 0.9600 | 正常停止 |
| 重复启停 60 次 | 60 | 247,469 | 2.1391 | 3.8188 | 3.9508 | 0.9200 | 正常停止 |

## 连续运动的误差与原因

### 各轴总误差

| 关节 | RMS mrad | P99 mrad | 峰值 mrad |
|---|---:|---:|---:|
| J1 | 3.7413 | 6.2238 | 6.7324 |
| J2 | 3.7540 | 6.2532 | 6.7644 |
| J3 | 3.7420 | 6.2258 | 6.7388 |
| J4 | 3.7465 | 6.2364 | 6.7516 |
| J5 | 3.7400 | 6.2210 | 6.7260 |
| J6 | 3.7411 | 6.2228 | 6.7209 |
| J7 | 3.7400 | 6.2208 | 6.7286 |

### 控制链误差分解

| 比较对象 | 最大关节 RMS mrad | P99 mrad | 峰值 mrad |
|---|---:|---:|---:|
| 已应用原始目标 → 实际反馈 | 3.7540 | 6.2540 | 6.7644 |
| 已应用原始目标 → 处理后参考 | 2.0698 | 3.4748 | 3.8595 |
| 最终下发 → 实际反馈 | 1.6896 | 2.8058 | 2.9470 |
| 处理后参考 → 最终下发 | 0.0000 | 0.0000 | 0.0000 |

trace 中的一阶低通偏差峰值为 **0.4254 mrad**，滤波后目标到最终参考的偏差峰值为 **3.5989 mrad**。各项峰值发生在不同采样时刻。

在总误差最大的同一时刻，J2 的总误差为 **6.7644 mrad**，其中参考处理贡献 **3.8395 mrad，约 56.8%**，实际跟随贡献 **2.9248 mrad，约 43.2%**。最终下发位置与处理后参考的差值为 0。

当前关节位置分支使用下式生成 jerk 请求

\[
j=w^3(q_{target}-q_d)-3w^2\dot q_d-3w\ddot q_d.
\]

在增益近似恒定且导数投影未触及限制的条件下，其连续时间近似为

\[
H(s)=\frac{w^3}{(s+w)^3}.
\]

该环节具有三阶低通型的跟踪滞后。本轮重建的增益中位数为 **100 s⁻¹**，最低为 **96.433 s⁻¹**。考虑当前先更新加速度、再更新速度和位置的离散积分次序，低频群延迟约为

\[
\tau_{limiter}\approx\left(\frac{3}{wh}-1\right)h_{wall}.
\]

当 `w=100 s⁻¹`、`h=1 ms` 时约为 29 个实际控制周期，与数据移位拟合得到的 **31.5 ms** 相符。本轮关节分支的调用路径及其公式分别见源码。[关节伺服分支](/home/rocos/Documents/GitHub/rocos-app/src/servo_signal_processor.cpp:216) · [位置跟踪律](/home/rocos/Documents/GitHub/rocos-app/src/model_limits.cpp:196)

一阶低通使用 `α=h/(h+1/(2πfc))≈0.38587`，低频群延迟为 `(1−α)/α≈1.5915` 个控制周期。对 J2 最终下发与实际反馈进行最小均方误差移位拟合，得到 **27.5 ms**。MuJoCo CSP 路径通过位置执行器、重力前馈和积分修正跟踪目标，该估计包含仿真执行器响应及控制/反馈时序。[MuJoCo CSP 实现](/home/rocos/Documents/GitHub/rocos-mujoco/src/mujoco_simulator.cpp:753)

本轮最终参考的最大速度、加速度、jerk 分别为 **0.11849 rad/s、0.70340 rad/s²、97.0968 rad/s³**，均低于配置上限。正常周期使用制动轨迹回退的次数为 **0**。跟踪滞后的主要证据指向上述位置跟踪律及实际响应。

连续运动的首尾反馈差最大为 **7.67×10⁻⁶ rad**。对中间 298 个完整周期统计，各轴周期峰值的最大波动范围为 **4.60×10⁻⁵ rad**，谷值最大波动范围为 **3.83×10⁻⁵ rad**，周期幅值保持在这一范围内。[连续运动完整统计](/home/rocos/Documents/GitHub/rocos-app/build/librocos_online_20261109/07_periodic_600s/analysis.json)

## 反复启停的累计偏移

60 次 `control()` 均返回，末次会话也完成正常停止。每次目标按该会话首帧参考重新建立，原始目标的首尾差在全部 60 个会话中均为 **0**。实际反馈的累计变化如下。

| 关节 | 累计变化 rad | 累计变化 ° | 等效编码器计数 |
|---|---:|---:|---:|
| J1 | 0.000000000 | 0.000000 | 0 |
| J2 | -0.000766990 | -0.043945 | -120 |
| J3 | 0.000000000 | 0.000000 | 0 |
| J4 | 0.000383495 | 0.021973 | 60 |
| J5 | 0.000000000 | 0.000000 | 0 |
| J6 | -0.000460194 | -0.026367 | -60 |
| J7 | 0.000000000 | 0.000000 | 0 |

J2 每个会话均变化 **−1.2783173×10⁻⁵ rad，即 −2 个计数**，会话间隙的额外变化为 0。首个会话的原始返回目标对应 **122868** 个计数，最终 `q_cmd` 仅比原始目标低 **5.55×10⁻¹⁶ rad**，换算结果为 **122867.99999999991**。App 的 `static_cast<int32_t>` 将其截断为 **122867**，随后实际反馈为 **122866**。

App 下发位置和 MuJoCo 反馈位置均使用整数截断。伺服启动保持参考取自实际位置，测试程序又将该参考作为下一会话的相对原点。数据中的整数计数变化、浮点残差和这一转换链条共同解释了重复会话中的累计偏移。影响同时涉及控制端位置转换、仿真反馈量化和相对起点选择。[App 转换](/home/rocos/Documents/GitHub/rocos-app/src/hardware.cpp:350) · [MuJoCo 转换](/home/rocos/Documents/GitHub/rocos-mujoco/src/mujoco_simulator.cpp:997) · [启动保持参考](/home/rocos/Documents/GitHub/rocos-app/src/move_servo.cpp:455) · [逐会话计数证据](/home/rocos/Documents/GitHub/rocos-app/build/librocos_online_20261109/restart_drift.json)

## 通信、保护及资源

| 阶段 | 发送间隔 P99 ms | 发送间隔峰值 ms | 新状态采样间隔 P99 ms | 采样间隔峰值 ms |
|---|---:|---:|---:|---:|
| 关节往返 example | 1.051 | 1.293 | 1.271 | 2.931 |
| 点到点 example | 1.051 | 1.175 | 1.296 | 2.184 |
| 点到点返回 | 1.051 | 1.081 | 1.339 | 2.238 |
| 连续运动 example | 1.052 | 1.197 | 1.302 | 4.614 |
| 保持通信 120 s | 1.050 | 1.209 | 1.282 | 3.698 |
| 七轴周期运动 600 s | 1.052 | 2.255 | 1.298 | 4.963 |
| 重复启停 60 次 | 1.169 | 2.528 | 1.767 | 5.339 |

多会话行取各会话 P99 和峰值的最大值。新状态采样间隔来自去重后的 UDP `sample_time_ns`，包含客户端未观察到中间控制周期的情况。`control_command_success_rate` 按在线契约表示新指令接收比例，物理执行通过实际反馈另行统计。

- 各会话抓包中的正命令序号和回包序号断点均为 0，tcpdump 报告的内核丢包均为 0。
- UDP 故障标志帧为 0，末次 HTTP 诊断 fault_code 均为 0。实际位置、参考位置和实际速度越过当前配置边界的样本数均为 0。
- 所有有效样本中 `q_cmd=q_d`，正常 trace 中制动回退次数为 0。
- App 在长测末期 RSS 约 **74.9 MiB**。重复启停阶段前半段增加到 **173.5 MiB**，后约 150 s 保持在该值。采样到的 App 线程数为 **42～44**，MuJoCo RSS 约 **215.0 MiB**。RSS 增长的分配归属尚待单独分析。

通信指标来自启用抓包、trace 和上述后台统计负载的运行。原始监控记录及边界检查均已保留。[边界检查](/home/rocos/Documents/GitHub/rocos-app/build/librocos_online_20261109/bounds_check.json) · [重复启停监控](/home/rocos/Documents/GitHub/rocos-app/build/librocos_online_20261109/08_restart_60/monitor.jsonl)

![连续运动跟踪、通信质量和重复启停偏移](/home/rocos/Documents/GitHub/rocos-app/build/librocos_online_20261109/overview.png)

## 复现入口与数据位置

在线测试要求一套已准备的 App/MuJoCo、使能后的 `STOPPED`、位置工作模式及本报告中的配置与调度条件。源码提供手动在线目标，业务参数如下。

```bash
cmake -S . -B build
cmake --build build --target librocos_online_test librocos_test rocosAppMain --parallel 1
ctest --test-dir build -R '^librocos_test$' --output-on-failure -j1

./build/bin/librocos_generate_joint_position_motion 127.0.0.1 8080 8081 1 0.02 4
./build/bin/librocos_generate_consecutive_motions 127.0.0.1 8080 8081 1 0.02 4
./build/bin/librocos_communication_test 127.0.0.1 8080 8081 1 0 120
./build/bin/librocos_online_test 600 2 0.05 1
./build/bin/librocos_online_test 4 2 0.03 60
```

本轮实际运行的实时调度前缀记录在每个 `result.json` 的 `command` 字段。数据目录包括每阶段的 `udp.pcap`、`commands.bin`、`states.bin`、`analysis.json`、`capture.log`、`client.log`、`monitor.jsonl`，以及统一的逐周期 trace、配置、哈希与最终状态。`setup_failure.txt` 记录运动测试前修复的抓包进程识别问题。

数据根目录为 [librocos_online_20261109](/home/rocos/Documents/GitHub/rocos-app/build/librocos_online_20261109)。最终 App PID 为 **1088818**，MuJoCo PID 为 **1088479**，两者继续运行，机器人处于 `IDLE`、使能关闭、RCI 关闭。[最终状态](/home/rocos/Documents/GitHub/rocos-app/build/librocos_online_20261109/final_state.json)
