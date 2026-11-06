# UDP 伺服四项定向修复与复测报告

日期：2026-11-06。仅修复小目标振荡、启停卡 STOPPING、通信质量误触发及终态/期望状态缺失。保持 IK 单次预算 0.1 ms；没有修改导纳、阻抗、主站/驱动保护、机器人参数或 Apifox。没有恢复伺服 Ruckig。单核构建，未提交 Git。

## 结果

| 项目 | 结果与证据 |
|---|---|
| 小目标振荡 | 原失败用例已转绿；8 种输入各运行 120 s 虚拟时间，固定 0.00297 rad 峰值 0.00297 rad；重复正弦峰值约 0.00997354 rad。在线关节正弦连续 1800.000 s 完成，无故障。 |
| STOPPING 卡住 | 离线 300 次快速启停、在线 300 次快速启停通过；在线同时进行状态查询。线程数/FD：{'tasks': 46, 'fds': 9} → {'tasks': 46, 'fds': 9}。 |
| 通信质量 | 使用同一 1 kHz 客户端。固定休眠接收时中位比例 0.94；到包唤醒长测中位 1.00、最低 0.89，无持续低质量故障。真实断流仍触发 LostHeartbeat。 |
| 故障回包及期望状态 | 首点前非零保持参考与实际位置一致；非法首点单包后不再发包仍收到主动终态；断流收到通信故障终态；last_control 在复位后仍保留首点故障。 |

修复前证据保留在 [原测试报告](/home/rocos/Documents/GitHub/rocos-app/docs/servo_stability_safety_test_report.md)，没有覆盖原始失败数据。

## 实现变化

1. 低通滤波保存独立状态；Model 新增 `LimitServoPositionStep`，使用 `w³(target-q)-3w²v-3wa` 产生期望 jerk，`w=min(0.1/dt,0.5*J/A)`。继续经过原有速度、加速度、jerk、位置包络及完整制动可行性检查，不可行时沿用已验证制动过程。
2. `IsControlActive()` 在同一 FSM 锁中判断所有活动状态。监督线程由 Robot 回收路径终止，不再随控制线程结束；伺服期间检测到控制线程退出会请求停止并进入错误态。
3. 接收成功比例按实际控制调用统计；实际调用数和累计名义周期数分开记录，整数纳秒余数跨周期累计。UDP 接收改为 poll 到包唤醒，空闲等待上限 1 ms。连续缺失、墙钟失联和持续低质量阈值均未降低。
4. 完成控制周期时发布保持参考，故障后拒绝新运动目标，在销毁 socket 前主动发送一次终态。RCI 状态实现新增 `current_control` / `last_control`，保留时间、序号、故障及最终参考/输出。UDP 终态仍是尽力交付，故障也可通过 HTTP 查询。

**跟踪代价：** 阻尼跟踪引入相位滞后和较慢的渐近收敛，不能宣称完全按输入轨迹无延迟运行。本次最大参考/对应已应用输入差为 0.001522000 rad。边界测试保留原精度和导数条件，将 5 s 目标轨迹后的稳定时间从 1 s 改为 7 s。未提供全参数、全状态空间稳定性证明。

## 30 分钟在线关节长测

原有 MuJoCo PID 53021；最终 App PID 94479，配置 `config/app.yaml`，低通、限速及通信保护均开启。七轴输入为各自起点加 0.01 rad、0.05 Hz 正弦，5 s 平滑渐入，各轴相位差 0.1 rad。客户端 SCHED_FIFO 30，1 kHz 发送。

| 指标 | 实测 |
|---|---:|
| 连续运行时间 | 1800.000005 s |
| 发送指令 / 收到回包 | 1800000 / 1799974 |
| 30 分钟后客户端断流收尾的查询状态 | ERROR_STATE（见下文） |
| 修正客户端退出顺序后的 60 s 对照 | SERVOING → STOPPED |
| 收尾断流的终态故障码 | -1008 |
| 各轴最大参考峰峰值 | 0.019940574 rad |
| 最大实际位置/最终输出差 | 0.000056973 rad |
| 参考位置/最终输出差 | 0.000000000 rad |
| 最大参考速度 / 加速度 / 相邻周期 jerk | 0.003788519 / 0.003240432 / 0.005830914 |
| 接收比例最小 / 中位 | 0.89 / 1.00 |
| 相邻控制样本间隔 p50 / p99 / 最大 | 1060.134 / 1196.688 / 4759.979 μs |
| 收尾快照实际控制更新 / 累计名义周期（含断流检测） | 1691747 / 1800020 |
| 同周期快照字段不一致 / applied 大于 received | 0 / 0 |
| 周期号 / 采样时间倒退 | 0 / 0 |
| 客户端原始状态 / 指令日志丢弃 | 0 / 0 |
| 活动期间 App RSS 范围 | 21700 … 22428 kB |

**收尾问题必须区分：** 长测客户端发完 180 万包后，先排序 180 万条统计值，再调用 stop，导致真实断流。原始 UDP 记录的全部故障位为 false，最后一条指令序号 1800000 已应用；故障终态采样在最后一次发送之后 163.262 ms（包含受限减速）。因此 30 分钟连续流稳定性通过，但该次正常退出失败。测试客户端已改为先 stop、后整理统计，另行 60 s 对照返回 STOPPED。未修改机器人失联保护来迁就测试脚本。证据：[shutdown_timing.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_fix_20261106/shutdown_timing.json)、[graceful_exit_60s.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_fix_20261106/graceful_exit_60s.json)。

导数按配置控制周期 1 ms 检查；主站实测节拍偏慢仍存在，未在本轮修改。短暂窗口最低为 0.89，但未持续满足 100 次低质量判据；阈值仍为 0.90。`control_command_success_rate` 表示新指令可用比例，不表示驱动执行成功率。上表周期间隔只统计回包中连续周期，回包遗漏不能当作控制周期未执行。

![长测曲线](/home/rocos/Documents/GitHub/rocos-app/build/servo_fix_20261106/joint_30min.png)

原始记录：[joint_30min.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_fix_20261106/joint_30min.json)、[joint_30min_metrics.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_fix_20261106/joint_30min_metrics.json)、[joint_30min_states.bin](/home/rocos/Documents/GitHub/rocos-app/build/servo_fix_20261106/joint_30min_states.bin)、[joint_30min_commands.bin](/home/rocos/Documents/GitHub/rocos-app/build/servo_fix_20261106/joint_30min_commands.bin)。

## 定向回归与故障验证

- 单核构建成功；move_servo_test 32 项、servo_robot_test 16 项通过；model_reference、position_controller、http_control_rights 测试通过。[final_ctest.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_fix_20261106/final_ctest.log)、[related_tests.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_fix_20261106/related_tests.log)。
- 1,620 组跟踪/停止扫描全部通过，逐周期检查位置、速度包络、加速度和 jerk，并确认停止收敛。[scan.csv](/home/rocos/Documents/GitHub/rocos-app/build/servo_fix_20261106/scan.csv)。
- 固定小目标、平滑/重复/变更新率轨迹共 8 组，每组 120,000 步无失败。[limiter_tracking_scan.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_fix_20261106/limiter_tracking_scan.log)。
- 在线固定 0.00297 rad 小目标运行 5 s 并收敛；300 次快速启停及并发状态查询无异常。[online_regression.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_fix_20261106/online_regression.json)。
- 非法首点拒绝并进入 ERROR_STATE；单包发送到收到故障终态 2.518 ms。复位后仍可查询 `HomePositionError (-2306)`。
- 真实断流后收到故障终态 164.341 ms，`LostHeartbeat (-1008)`。此数值含失联检测、受控减速、线程调度及接收开销，是本次观测值，不是硬实时上界。

![离线修复前后](/home/rocos/Documents/GitHub/rocos-app/build/servo_fix_20261106/tracking_before_after.png)

## IK 与测试边界

笛卡尔保持在首个运动周期返回 `IkCalcFail (-2349)`；客户端含等待退出约 0.103 s。按用户要求保留 0.1 ms IK 预算，转为关节长测。该失败未在本轮修复，不能据关节通过宣称笛卡尔伺服可用。[cartesian_hold.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_fix_20261106/cartesian_hold.json)。

Apifox 在线 ROCOS API 已刷新核对；RCI 与新增诊断字段仍属于当前实现扩展，尚未包含于在线正式契约，本轮没有更新 Apifox。

测试准备阶段，旧可执行文件被重链接标记为 `(deleted)`，重启脚本漏识别，短暂多启动 App PID 92286，随即 SIGINT 退出。脚本已改为兼容该标记并要求恰有一个既存 App；正式长测期间仅运行 PID 94479，MuJoCo 未重启。另一次测试客户端在 HTTP 尚未就绪时退出，未开始运动；就绪后重新开始完整 1800 s 测试。

最终状态：IDLE，RCI enabled=False，原配置保留。上述是本机仿真的有限样本验证，不能替代真实硬件能力核定；导纳、阻抗、IK 和整进程暂停时的主站/驱动行为不在本轮修复范围。

## 补充：IK 无解还是预算不足（2026-11-06）

复用失败现场第一条 UDP 指令和故障快照中的实际关节位置，未修改生产代码、0.1 ms 默认预算或运行中的 App。活动工具变换为单位变换。当前关节均在模型范围内，关节初值与客户端生成目标时的关节值完全相同；用实际 Model 正解检查，位置误差和姿态误差均为 0。因此这个目标明确有解，初值本身就是解，不能把此次 IkCalcFail 解释为不可达。

每组独立重复 3,000 次，离线普通调度运行；“扰动”只把第一关节初值增加 0.001 rad，目标不变。PIN_IK 各组使用显式构造预算，不改生产默认值。

| 路径 | 预算 μs | 初值偏移 rad | 失败数 / 3000 | 中位耗时 μs | P99 μs |
|---|---:|---:|---:|---:|---:|
| Model | 100 | 0 | 8 | 54.879 | 122.888 |
| PIN_IK | 100 | 0 | 43 | 50.153 | 232.860 |
| PIN_IK | 200 | 0 | 6 | 50.236 | 160.820 |
| PIN_IK | 500 | 0 | 1 | 52.019 | 165.626 |
| PIN_IK | 1000 | 0 | 1 | 56.665 | 207.534 |
| PIN_IK | 5000 | 0 | 0 | 58.230 | 226.289 |
| Direct_TL | 100 | 0 | 0 | 0.542 | 0.823 |
| Model | 100 | 0.001 | 26 | 48.733 | 206.045 |
| PIN_IK | 100 | 0.001 | 28 | 53.569 | 181.338 |
| PIN_IK | 200 | 0.001 | 3 | 53.770 | 233.095 |
| PIN_IK | 500 | 0.001 | 2 | 54.984 | 277.192 |
| PIN_IK | 1000 | 0.001 | 1 | 62.748 | 233.379 |
| PIN_IK | 5000 | 0.001 | 0 | 52.506 | 243.567 |
| Direct_TL | 100 | 0.001 | 0 | 4.780 | 37.411 |

Model 是完整应用 IK 路径（含结果验证）；PIN_IK 是双线程包装；Direct_TL 是同一个内部迭代器的同步直接调用（仅诊断对照，未接入应用）。所有成功输出均用 Model 正解复核，达到现有位置和姿态精度要求。

结论：本次目标不是无解，主要问题是 100 μs 总预算与逐次创建双线程的实现不匹配。`pin_ik.cpp:314` 先记录开始时刻，324/325 行创建两个线程；工作线程在 `runSolver` 中计算剩余时间，预算耗尽则可以在真正执行迭代之前退出。等待 join、调度和后续校验又使整个函数耗时可能超过配置预算。Model 最后统一映射到 IkCalcFail，没有单独的超时错误码。

把预算增大到 0.5 或 1 ms 在本次样本中仍有偶发失败，5 ms 两组各 3000 次全部通过；这既不是建议恢复 5 ms，也不是硬实时保证。更合适的优化方向是先检查初值是否已满足目标、使用复用工作区的同步热启动求解路径，减少重复 IK，并分别记录求解超时、未收敛和结果验证失败。Direct_TL 的小扰动测试通过不代表它已验证覆盖全部机器人位姿或可直接替换现有后端。

现场没有逐阶段计时，无法倒推出原始那一次失败究竟卡在哪个线程；上述结论来自已知可行解、重复预算对照和代码路径证据。

证据：[已知解验证](/home/rocos/Documents/GitHub/rocos-app/build/servo_fix_20261106/ik_reachability.json)、[汇总](/home/rocos/Documents/GitHub/rocos-app/build/servo_fix_20261106/ik_diagnosis_summary.json)、[42,000 次原始记录](/home/rocos/Documents/GitHub/rocos-app/build/servo_fix_20261106/ik_diagnosis.csv)、[诊断程序](/home/rocos/Documents/GitHub/rocos-app/build/servo_fix_20261106/ik_diagnosis.cc)。

## 0.3 ms 预算实际效果测试（2026-11-06）

**结论：单次失败率有所降低，但连续笛卡尔伺服仍未通过。** 本轮保持生产预算 0.3 ms，只做测试，没有更改求解代码或保护阈值。

现有 App PID 170887、原有 MuJoCo PID 53021 映射同一 ecm_v2_0 共享内存；活动工具为单位变换，低通、限速、通信保护开启。没有重启或额外启动实例。诊断程序单核编译，离线测试结束后再进行在线测试。

### 离线：每组 10,000 次

复用实际失败现场的精确目标、已知可行初值；另一组只将初值第一关节偏移 0.001 rad。Model 为应用完整路径，PIN_IK 为双线程包装，Direct_TL 为直接同步调用内部迭代器的诊断对照。显式预算只作用于离线实例，没有修改 App。

| 路径 | 预算 μs | 初值偏移 rad | 失败次数 | P50 μs | P99 μs | 最大 μs | 整体调用超预算次数 |
|---|---:|---:|---:|---:|---:|---:|---:|
| Model | 300 | 0 | 6 | 51.65 | 156.45 | 3603.55 | 38 |
| PIN_IK | 100 | 0 | 56 | 56.34 | 223.15 | 3589.10 | 895 |
| PIN_IK | 300 | 0 | 13 | 61.00 | 268.78 | 5210.23 | 79 |
| Direct_TL | 300 | 0 | 0 | 1.25 | 5.73 | 46.11 | 0 |
| Model | 300 | 0.001 | 9 | 53.98 | 272.66 | 3372.93 | 82 |
| PIN_IK | 100 | 0.001 | 65 | 53.15 | 146.96 | 3700.80 | 220 |
| PIN_IK | 300 | 0.001 | 9 | 47.76 | 132.52 | 3426.76 | 38 |
| Direct_TL | 300 | 0.001 | 0 | 3.00 | 3.83 | 17.54 | 0 |

同一双线程后端，精确初值失败率从 0.56% 降至 0.13%，扰动初值从 0.65% 降至 0.09%。但当前 Model 仍分别失败 6 和 9 次；0.3 ms 不是整个函数的硬截止时间，Model 最大耗时约 3.60 ms。

共 80,000 次调用，成功结果均经 Model FK 复核，满足现有位置/姿态精度要求。各组顺序运行，调度环境不完全一致，时间分位数不应解释为严格受控的性能差异。Direct_TL 两组无失败只支持进一步优化调用结构，不能替代全位姿验证。

### 在线：笛卡尔保持及小幅运动

客户端 SCHED_FIFO 30、1 kHz UDP。保持目标由当前关节 FK 生成；小幅运动由七轴 0.01 rad、0.05 Hz 正弦、5 s 渐入，再转换为笛卡尔位姿。每次故障后显式复位、建立新会话。

| 模式 | 次数 | 计划时长 s | 首条发送至故障周期采样 s | 故障码 | 故障时接收比例 |
|---|---:|---:|---:|---:|---:|
| 保持 | 1 | 10 | 0.997860 | -2349 | 0.95 |
| 保持 | 2 | 10 | 2.644316 | -2349 | 1.00 |
| 保持 | 3 | 10 | 0.111205 | -2349 | 0.96 |
| 保持 | 4 | 10 | 1.198993 | -2349 | 1.00 |
| 保持 | 5 | 10 | 0.296191 | -2349 | 1.00 |
| 小幅运动 | 1 | 30 | 0.117728 | -2349 | 1.00 |
| 小幅运动 | 2 | 30 | 0.324877 | -2349 | 1.00 |
| 小幅运动 | 3 | 30 | 0.238645 | -2349 | 1.00 |
| 小幅运动 | 4 | 30 | 0.363167 | -2349 | 1.00 |
| 小幅运动 | 5 | 30 | 1.233022 | -2349 | 1.00 |

10 次均为 IkCalcFail 并进入 ERROR_STATE，没有一次完成计划时长。全部收到故障终态，通信故障位均为 false。上述时间以故障周期单调采样时刻计算，不包含客户端额外等待无回包的约 100 ms；采样在该周期求解前进行，因此不是精确的失败指令执行时刻。小幅运动在渐入阶段已经失败，未覆盖完整正弦行程。

每周期重复 IK 会累积单次失败风险，单次成功率很高仍不足以支撑连续伺服。优先优化方向仍是初值已满足目标时直接返回、同步热启动、复用工作区和减少重复 IK。本轮未接入这些优化，也没有忽略失败继续运动。

在线 Apifox 已刷新；RCI 仍为实现扩展，尚未列入在线正式契约，本轮未更新 Apifox。最终恢复 IDLE、下使能、RCI 关闭，仍保留 0.3 ms 预算及原 MuJoCo。

证据：[离线汇总](/home/rocos/Documents/GitHub/rocos-app/build/ik_budget_03ms_test/offline_summary.json)、[80,000 次原始记录](/home/rocos/Documents/GitHub/rocos-app/build/ik_budget_03ms_test/ik_diagnosis.csv)、[在线汇总](/home/rocos/Documents/GitHub/rocos-app/build/ik_budget_03ms_test/online_summary.json)、[各次会话记录](/home/rocos/Documents/GitHub/rocos-app/build/ik_budget_03ms_test/online_suite.json)、[进程映射](/home/rocos/Documents/GitHub/rocos-app/build/ik_budget_03ms_test/process_maps.txt)、[最终状态](/home/rocos/Documents/GitHub/rocos-app/build/ik_budget_03ms_test/final_cleanup.json)。

## 1 ms 预算内测提交验证（2026-11-06）

本次内测版本为 `3.2.0-rc.6`。Model 在初始化、更新关节限位和重建求解器时均显式传入 1 ms，耦合关节求解也使用同一预算，不依赖 pin_ik 头文件中的默认参数。pin_ik 子模块本地修改和 gitlink 均不纳入提交。

单核构建 App、UDP 示例及本轮 13 个回归测试目标通过。CTest 使用 `--parallel 1`，12 个套件通过：model、model_reference、move_servo、servo_robot、hardware、position_controller、joint_impedance_controller、joint_admittance_controller、cartesian_impedance_controller、http_control_rights、identification_execution、identified_hardware。

`app_config_test` 的 18 个用例中 17 个通过，RCM 配置用例因缺少 `config/robots/rcm/robot.yaml` 抛出异常；207 个已执行断言全部通过。未为通过测试修改外部配置资源。

本轮未重新启动在线 App 或仿真，也未进行 1 ms 预算下的长期笛卡尔运动测试。Model 回归通过不代表连续伺服稳定性已得到验证；单次求解预算仍不是整个函数的硬截止时间。前述 0.3 ms 在线失败记录保留为历史证据。

构建与测试日志分别保存在 `build/rc6_validation/build.log`、`build/rc6_validation/tests.log`，属于本地验证产物。
