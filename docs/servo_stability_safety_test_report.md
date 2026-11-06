# ROCOS-App 伺服稳定性与安全性测试报告

测试时间：2026-11-05 至 2026-11-06（Asia/Shanghai）。结论：**不通过当前稳定性与安全性验收，不建议直接进入带运动的真实机器人验证。**

本次只测试和定位，没有修改产品控制逻辑、IK 预算、限制或断言；没有提交 Git、递增版本或修改 Apifox。按用户追加要求，临时关闭限速器进行了对照，之后恢复原配置。测试结果区分在线仿真实测、离线真实算法/控制器实测、已有回归测试、源码分析及未验证项目。

产物目录：[build/servo_validation_20261105](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105)。完整逐项矩阵包含 66 项：[test_matrix.csv](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/test_matrix.csv)。CSV 中每项均有前置条件、输入、预期、实际、结论和证据路径。不能将“拒绝了输入”“写入了停止请求”“机械系统已停稳”视为同一个结果。

## 1. 优先结论

| 优先级 | 发现 | 直接证据与影响 |
|---|---|---|
| P1 | 限速器会把小幅输入变成持续大幅振荡 | 在线 ±0.01 rad 正弦产生约 ±0.55 rad 实际运动；离线固定 0.00297 rad 目标也产生 0.551756 rad 峰值。速度、加速度、jerk 可同时不越限，仍有意外运动风险。 |
| P1 | 快速启动/停止后控制与监督线程退出，状态卡在 STOPPING | 首轮第 57 次循环未完成；现场线程消失，控制字仍为 15，stop/disable/reset 无法恢复。用户重启 App 后才继续测试。 |
| P1 | 整个 App 停顿时没有测到主站侧独立失联停止 | 暂停 App 约 81.4 ms，MuJoCo 持续更新，控制字一直为 15；恢复后约 82.46 ms 才出现控制字 2。主机内监督线程不能覆盖整个进程失效。 |
| P1 | 最终控制器输出可能绕过导数限制 | 导纳控制器输出 jerk 为 8,000 rad/s³（配置 150）；笛卡尔阻抗首次输出跳变 5 Nm（200 Nm/s、1 ms 对应 0.2 Nm）。均返回成功。 |
| P1 | 0.1 ms IK 与长期运行存在可用性问题 | 单次 IK 10,000 次有 118 次失败；笛卡尔处理步 1,000 次有 28 次失败。四个 450 s 长测工况均提前退出，30 分钟目标未完成。 |
| P2 | 外部闭环状态不完整 | 等待首点时 q_d 可为零而 q_cmd 为非零保持位置；故障用例未收到携带错误的终态 UDP 包。 |
| P2 | ResetFault 的返回值与完成状态不一致 | 多次实际从 ERROR_STATE 恢复到 IDLE，但 API 返回失败并记录“未到 STOPPED”。 |

有效改进也有实测支持：跨发送端与旧会话隔离通过；UDP 绑定失败正确回传 HTTP；超差关节首点被拒绝；单独暂停控制线程、暂停主站均能在约 20 ms 写入停止请求；1,620 组约束/制动扫描未复现“推进到无法停止状态”。这些通过项不能抵消上述失败。

## 2. 环境与代码基线

- HEAD：`89d935589efd35bab134ffc08e617603bd4b5dc6`，存在大量原有未提交修改。报告针对测试开始时工作树，不能当作仅针对该提交。见 [git_status.txt](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/git_status.txt)、[baseline_diff.txt](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/baseline_diff.txt)。
- Release，Unix Makefiles，`/usr/bin/c++`，产品优化 `-O3 -DNDEBUG`。使用 `cmake --build build --parallel 1` 构建相关目标；新增小工具按既有测试链接参数逐个编译，没有并行编译。
- Ubuntu PREEMPT_RT 内核 `6.8.1-1061-realtime`。App 控制线程观察为 SCHED_OTHER；部分发送客户端用 `chrt -f 30`。RT 内核本身不意味着所有应用线程都满足实时截止时间。
- 机器人为 Talon 7 轴，模型为 [robot.urdf](/home/rocos/Documents/GitHub/rocos-app/config/robots/talon/robot.urdf)，模型轴顺序 joint_1…joint_7 对应驱动 id 0…6。声明控制周期 1000 μs。仿真场景来自 `/home/rocos/Documents/GitHub/rocos-mujoco/config/talon/mjcf/scene.xml`。
- 仿真身份依据：MuJoCo 进程 executable、加载 `libmujoco.so.3.4.0`、运行配置、App 与其映射相同 inode 的 `ecm_v2_0`/输入/输出 PDO。没有仅凭进程名称、backend 字段或用户描述判定。证据：[user_simulation_identity.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/user_simulation_identity.json)、[user_shm_identity.txt](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/user_shm_identity.txt)。MJCF 与 URDF 的完整惯性/几何等价性未校验。
- 正式在线仿真主站一直为 PID 53021。主要长测 App 为 PID 53046；第一次卡住后用户重启为 83921。后续用户再启动的 84491 与 83921 曾同时监听 8080/8082，未在该重叠时段执行有效轨迹测试。获得用户授权后结束二者，使用唯一 PID 84673 做关闭限速器对照，最后恢复原配置为 PID 85599。
- 早期隔离启动准备产物 `app.yaml`、`robot.yaml`、`simulation_identity.json` 等不属于正式在线结果；临时实例已结束。正式结果只使用上述已核对的用户 MuJoCo。现有回归夹具与真实硬件运行没有混为一谈。
- 测试末产品源码、原始配置、已有测试文件相对基线 SHA256 无变化：[source_integrity.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/source_integrity.json)。配置指纹见 [configuration_hashes.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/configuration_hashes.json)。

### 实际使用的限制

| 项目 | 数值 | 来源/证据边界 |
|---|---|---|
| q 下限 rad | -2.84, -2.22, -2.84, -2.23, -2.71, -2.11, -2.84 | URDF |
| q 上限 rad | 2.84, 2.22, 2.84, 2.23, 2.71, 2.11, 3.14 | URDF |
| 最大速度 | 各轴 3 rad/s | URDF |
| 最大力矩 | 各轴 30 Nm | URDF |
| 加速度 / jerk | 各轴 10 rad/s² / 150 rad/s³ | hardware.yaml；没有型号实机核定证据 |
| 减速度 | 10 rad/s² | 配置缺省后使用 acceleration；并非独立标定能力 |
| velocity_offset | 0 | 默认值 |
| torque_rate | 200 Nm/s | JointDerivativeLimits 默认值；笛卡尔控制器另有自身设置，不能假定全模式统一 |
| following_error | 0.06 rad | 未显式配置时按 3 × 20 × 0.001 计算 |
| 首点容差 | 远离边界时 0.003 rad；靠近边界按方向速度包络收缩 | Model 限制与周期计算；不是通用固定容差 |
| 首点实际速度条件 | 不超过 acceleration × dt = 0.01 rad/s | 源码检查 |
| 低通 | 默认开启，100 Hz | ServoSettings |
| 限速器 | 默认开启；追加对照仅临时关闭此项 | ServoSettings / 临时 YAML |
| 连续缺失 / 首包等待 | 20 周期 / 1000 ms | ServoSettings |
| 质量窗口与阈值 | 最近至多 100 周期，新指令比例 <0.90 持续 100 周期触发 | ServoCommunication / MoveServo |
| 反馈新鲜度 | 20 × dt = 20 ms | ValidateFeedback / Hardware |
| IK 单次预算 | 0.0001 s | pin_ik.hpp:55、58；配置预算并非硬时限 |

### 基线回归

[build.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/build.log) 构建成功；[baseline_tests.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/baseline_tests.log) 中 11 套有 9 套通过。

通过：model_reference、move_servo、servo_robot、hardware、position_controller、joint_impedance_controller、joint_admittance_controller、cartesian_impedance_controller、http_control_rights。

失败：model_test 的有界关节 IK、Talon 近初值 IK、并发 FK/IK 三例；app_config_test 一例缺少 `config/robots/rcm/robot.yaml`。后者是配置夹具问题，不能归因于在线伺服。既有控制器测试通过与本次新增控制器失败并不矛盾：新增用例覆盖了此前没有断言的最终 jerk 和首次力矩跳变。

## 3. 轨迹稳定性与约束可持续性

### 在线小幅正弦出现大幅实际运动

输入各关节为初始位置加幅度 0.01 rad、频率 0.05 Hz 正弦，前 5 s 平滑渐入，各轴相位差 0.1 rad。客户端 2 kHz 发包，服务端声明 1 kHz 控制。

| 指标 | 测量值 |
|---|---:|
| 发送程序持续时间 | 309.118968 s，计划 450 s |
| 第 1 轴参考范围 | -0.580048 … 0.549307 rad |
| 第 1 轴实际范围 | -0.579723 … 0.548999 rad |
| 所有轴最大参考速度 | 2.891779 rad/s |
| 最大参考加速度 / 相邻周期 jerk | 10 rad/s² / 150 rad/s³ |
| 最大实际与最终下发位置差 | 0.0384534 rad |
| 结束时日志 | 通信质量 0.86，受控停止后 LostHeartbeat |

参考与实际同时放大，而实际与参考之差仍小于默认 0.06 rad 跟随误差阈值。因此单纯跟随误差保护不能发现“参考本身远离用户目标”。日志和 UDP 数据不支持把该现象解释为仅有显示误差。

![在线参考与反馈曲线](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/simulation_tracking.png)

证据：[ranges.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/ranges.json)、[soak_joint_sine.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/soak_joint_sine.json)、同名前缀 `_commands.bin` / `_states.bin`。图为降采样显示，极值由全部原始记录计算。

### 无网络最小复现

`limiter_tracking_scan` 调用当前库的真实 `ServoSignalProcessor`，FakeModel 只提供边界/运动学，周期严格为 1 ms，V=3、A=10、J=150、D=10，初态静止。每组 120,000 步（120 s 虚拟时间）。

| 输入 | 默认限速开启结果 |
|---|---|
| 平滑正弦，每 1 ms 更新 | 峰值 0.01 rad，正常跟踪 |
| 同一正弦，每 2 ms 更新，中间保持上一目标 | 峰值 0.551095 rad，速度峰值 2.87396 rad/s |
| 每 3 ms / 10 ms 更新 | 峰值 0.549336 / 0.564611 rad |
| 从静止到固定 0.00297 rad 目标 | 峰值 0.551756 rad，持续振荡，返回成功 |
| 从静止到固定 0.000001 rad 目标 | 峰值约 1.06793e-6 rad，无上述放大 |

这证明问题不依赖网络、MuJoCo 或 IK。固定 0.00297 rad 首点位于此条件下 0.003 rad 容差内；即使阶跃不满足全部推荐导数条件，系统接受并平滑它之后也不应持续生成约 0.55 rad 的运动。

代码分析：[servo_signal_processor.cpp:120](/home/rocos/Documents/GitHub/rocos-app/src/servo_signal_processor.cpp:120) 用当前位置误差除以 dt 构造期望速度；[model_limits.cpp:180](/home/rocos/Documents/GitHub/rocos-app/src/model_limits.cpp:180) 限制下一步加速度。现有制动可行性检查主要约束关节物理边界，没有证明相对于目标的稳定收敛。该结构可解释现象，但本报告没有提供完整闭环稳定性证明，也没有修改算法。

![离线限速器稳定性复现](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/limiter_stability.png)

### 制动可行性扫描

[scan.csv](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/scan.csv)：**1,620 组，失败 0**。起点 {-0.98,-0.5,0,0.5,0.98}，位置范围 [-1,1]，A={1,3}，J={1,20,150}，D={0.2,1,10}，正负方向、目标幅值 {0.7,0.95,0.999}，停止周期 {50,370,900}。每周期检查 q、当前位置速度包络、A、J，停止阶段最终 AtRest，最多允许 30,000 步。

这是约束及受限停止的有限样本通过，不是跟踪稳定性通过。其初始状态来自 Reset 的零速度/零加速度，动态过程中覆盖了非零导数状态，但没有穷举任意初始速度、加速度或证明连续状态空间安全。

## 4. 关闭限速器对照

用户明确要求后，仅设置 `rate_limit_enabled=false`，低通仍 100 Hz，首点、加速度、jerk、通信与反馈必要检查保留。临时 [app_limiter_off.yaml](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/app_limiter_off.yaml) 已通过 `--check_config`；原 `config/` 文件没有改动。

Franka 文档有版本差异： [libfranka 0.18.0 Robot API](https://frankarobotics.github.io/libfranka/0.18.0/classfranka_1_1Robot.html) 的 control 重载默认 `limit_rate=false`；[overview](https://frankarobotics.github.io/docs/doc/libfranka/docs/overview.html) 仍描述默认开启，并建议设计新控制环的首次测试先关闭这些信号处理功能。不能把“Franka 默认”脱离版本，也不能用该默认值证明 ROCOS 的关闭分支稳定。

| 对照 | 结果 |
|---|---|
| 离线平滑正弦 1 ms 更新 | 120 s 虚拟时间通过，幅度 0.01 rad |
| 离线每 2 / 3 / 10 ms 更新 | 第 416 / 331 / 181 周期 JerkLimit，拒绝继续输出 |
| 离线固定 0.00297 rad | 第 0 周期 AccLimit |
| 离线固定 1e-6 rad | 第 0 周期 JerkLimit |
| 在线关节保持 | 完成 15.000 s，无故障退出 |
| 在线关节正弦 | 客户端约 0.102 s 退出，JerkLimit |
| 在线笛卡尔保持 | 约 0.102 s 退出，IkCalcFail |
| 在线笛卡尔正弦 | 约 0.251 s 退出，JerkLimit |

证据：[limiter_off.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/limiter_off.log)、[off_suite.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/off_suite.json)。客户端退出时间含“无 UDP 回复 100 ms”的观察延迟，不是故障发生时刻。

关闭分支拒绝不符合内部差分检查的样本是必要保护生效，不能仅因未完成运动就认定该分支错误。结果说明关闭限速器没有普遍解决在线可用性，更不能以关闭默认保护来掩盖开启分支的振荡。测试后已恢复原配置。

## 5. 启停、RCI、输入与会话

### 快速启停卡住

第一轮计划 100 次，每次 `rci/start` 后等待 5 ms 再 `rci/stop`，不发送运动点。完整服务器日志在该阶段有 58 次进入 SERVOING、57 次伺服完成：包含 1 次并发接入预试、56 次完成循环，第 57 次循环卡住。最初测试脚本异常前未逐次落盘 HTTP 结果，次数由服务器日志重建；该限制已保留，未伪造缺失响应。

现场：`STOPPING`、RCI active=true；进程只剩基础线程；读 PDO 2 s，主站时间持续增长，所有轴 control word 仍为 15，目标保持旧位置。`rci/stop` 与 `rci/disable` 可返回成功但无法完成；普通 stop 返回 -2307，disable/reset 返回 -2017。见 [rci_timeout_full.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/rci_timeout_full.log)、[stop_timeout_threads.txt](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/stop_timeout_threads.txt)、[stuck_stopping_shm.csv](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/stuck_stopping_shm.csv)、[stuck_api.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/stuck_api.json)。

源码分析：控制线程循环/退出依赖 [robot.cpp:1179](/home/rocos/Documents/GitHub/rocos-app/src/robot.cpp:1179) 的多次状态查询，每次单独加锁；它们不构成同一时刻的状态快照。跨状态切换可能导致组合查询返回 false；[robot.cpp:538](/home/rocos/Documents/GitHub/rocos-app/src/robot.cpp:538) 又在控制线程退出时关闭监督。**这是待进一步确认的竞态解释，不是已经抓到的线程退出栈。** 建议用单次受锁状态快照判定控制活动状态，并使监督存活与控制线程存活相互独立；为异常退出建立明确故障转换与恢复路径。

第二轮在限速关闭配置下、不发运动点，100 次全部结束，100 个不同 session，线程 46→46、FD 9→9：[lifecycle_test.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/lifecycle_test.json)。该轮说明故障不是每次发生，不构成对第一轮失败的修复验证。

### RCI 与 UDP

实测通过：RCI 未开启返回 -1018；普通控制权开启后可释放 HTTP lease，再无凭据接入；两个并发程序只有一个取得会话；端口占用返回 -1016 并进入 ERROR，active=false，未恢复时再启动返回 -2215。HTTP 普通运动、点动、模式、TCP/负载、脚本和抢占入口在 RCI 独占现场返回 -2215。公共 Robot/Lua/debug token 路径还有既有回归证据，但没有把全部路径都记为在线实测。

另一个 UDP 端口使用 >100000 序号和偏离目标不能覆盖当前发送端，攻击端收到回复 0；重启 session 后同一端口发送旧 session 大序号也被拒绝。最大 received 分别为 146、163，均来自合法当前序列；两次结束前仍为 SERVOING：[peer_test.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/peer_test.json)、[peer_summary.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/peer_summary.json)。

### 首点与输入

- 单/多轴 0.00303 rad 超差首点触发 `HomePositionError` / `start_pose_invalid`，先于滤波/限速；源码顺序与已有回归支持“不会让平滑器追错误起点”。在线没有取得故障后完整 PDO 序列，因而不声称已经逐周期证明拒绝之后所有最终输出不变。反馈还有不到约 0.001 rad 的变化，可能包括仿真停机/量化效应，不能解释为错误目标已应用。
- 0.00297 rad 容差内首点被接受，但后续振荡失败，见第 3 节。
- 测试写入的“恰好 0.003 rad”是相对先前 HTTP 采样，不是控制周期瞬时反馈，在线拒绝不能独立认定边界比较有 bug；Fake 边界用例通过。
- NaN / Inf 拒绝为 -2902；错误版本、session、过短/过长包不能维持会话，首点等待超时后进入 ERROR。
- 笛卡尔相同姿态、容差内外测试多数先被 IK 失败遮蔽；不可达目标被拒绝，但本次未完成真实 IK 下笛卡尔首点容差边界、工具转换、多种旋转与非有限矩阵的全部在线验证。

## 6. 通信、独立监督与停止时序

**退出流程澄清：App 已有正常退出自动下使能。** `rocosAppMain.cpp:22` 处理 SIGINT/SIGTERM，主循环退出后在 [rocosAppMain.cpp:78](/home/rocos/Documents/GitHub/rocos-app/src/rocosAppMain.cpp:78) 调用 `robot.setDisabled()`。本报告的整进程故障注入使用 SIGSTOP，进程被暂停而没有退出，不能执行信号清理或下使能代码。因此 W03 只说明“整进程被挂起时的独立保护覆盖”，不说明“正常退出不下使能”。正常退出下使能的实际完成时延未在 W03 中测试；SIGKILL 也未实际注入。

独立只读 PDO 观测约每 0.5 ms 采样。下表时间从注入开始到首次读到控制字 `0x0002`；采样间隔最大约 0.84–0.96 ms，不能把显示的小数位当作微秒级硬保证。测试都在等待首点、静止状态进行，没有机械制动距离/停稳时间结论。

| 注入 | 暂停时长 | 首次停止请求可见 | 暂停期间主站 | 恢复后 |
|---|---:|---:|---|---|
| 原 MuJoCo 主站暂停 | 80.236 ms | 19.912 ms | timestamp 停滞 | ERROR_STATE |
| 仅控制线程 ptrace 暂停 | 80.123 ms | 19.198 ms | timestamp 持续增长 | ERROR_STATE |
| 整个 App SIGSTOP | 81.431 ms | 82.459 ms | timestamp 持续增长；control word 全程 15 | 恢复后 ERROR_STATE |

证据：[watchdog_summary.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/watchdog_summary.json)、[watchdog_suite.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/watchdog_suite.json)、`master_pause_shm.csv`、`control_pause_shm.csv`、`app_pause_shm.csv`。所有暂停均已恢复，没有留下 SIGSTOP 或 ptrace 附加。

这证明主机内独立监督在控制线程还可被独立监督的条件下有效；主站暂停时写 PDO 不等于暂停中的主站已经执行停止。整个 App 被暂停时，所有主机监督也停止运行，当前仿真没有给出独立于 App 的及时停止证据。建议增加主站消费命令的期限/序号监督，并在真实驱动侧核对失联超时与停止配置；不能用本次仿真宣称这些驱动保护已存在。

连续 60 ms 漏发和发送沉默均触发 missing_cycles=20。两次日志从“controlled stop latched”到 ERROR 分别约 285 ms、284 ms，属于故障停止流程时长，**不是从丢包到物理停稳的测量**。单包丢失与确定性 10% 随机丢包各运行 2 s；没有做 netem 固定延迟、突发积压分布及发送进程 SIGKILL 的独立覆盖。

长期保持失败时发送客户端最大间隔仅 918 μs，后台日志丢弃计数 0，仍触发 quality=0.83。因此该次故障不能归因于客户端出现 20 ms 空窗；服务端轮询、主站节拍、调度与质量统计的组合仍需定位。长测正弦客户端最大间隔 6.437 ms、最大工作时间 3.448 ms，不能忽略观测工具自身的延迟。

## 7. 反馈与控制器最终输出

实际位置、速度越界、跟随误差、新鲜度在既有 PositionController 等回归中有拒绝证据，暂停主站提供了真实反馈停更的在线证据；本次没有向在线 PDO 篡写 NaN 或非法实际位置，也没有证明所有允许控制模式都共享完全相同的故障路径。

### 导纳 jerk（新增离线实测失败）

FakeHardware 明确返回 A=10、J=150、D=10；dt=1 ms，M=1、B=10，初始 q/v/外力矩为零。先做一次零输出，然后测得外力矩从 0 变为 8 Nm，用户位置目标仍为 0。当前真实导纳控制器输出 q=8e-6 rad、v=0.008、a=8、jerk=8000，返回 NoError。

[admittance_audit.cc](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/admittance_audit.cc)、[admittance_test.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/admittance_test.log)。问题位置：[joint_admittance_controller.cpp:243](/home/rocos/Documents/GitHub/rocos-app/src/joint_admittance_controller.cpp:243)。建议对最终输出加入适用于该模式的导数/可停约束，或者明确禁止未验证的控制模式用于 RCI。该用例证明控制器层问题，没有在线切换导纳制造运动。

### 笛卡尔阻抗首次力矩（新增离线实测失败）

实际/上次力矩均 0，重力模型返回 5 Nm，设置 200 Nm/s、1 ms。首次输出为 5 Nm，允许变化应为 0.2 Nm；返回 NoError。[cart_imp_test.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/cart_imp_test.log)。

[cartesian_impedance_controller.cpp:296](/home/rocos/Documents/GitHub/rocos-app/src/cartesian_impedance_controller.cpp:296) 只在 `tau_prev_valid_` 为 true 时限速；Reset 也会清除这个标志。首次输出已实测，Reset 后同类风险来自源码分析。建议首次/Reset/模式切换基准统一采用有效的先前输出或实际力矩，并补测停止分支。关节阻抗已有首次/Reset 限速回归通过，不应将笛卡尔控制器失败错误归因于关节阻抗。

## 8. 实时性与长时间测试

### 长测验收未完成

| 默认配置工况 | 计划 | 客户端实际持续 | 结果 |
|---|---:|---:|---|
| 关节保持 | 450 s | 65.479 s | quality 0.83 后故障；位置保持 0 |
| 关节正弦 | 450 s | 309.119 s | 大幅振荡；quality 0.86 后故障 |
| 笛卡尔保持 | 450 s | 0.102 s | IkCalcFail |
| 笛卡尔正弦 | 450 s | 0.101 s | IkCalcFail |

总计划 1800 s，均提前失败。没有关闭通信保护或放宽阈值重试，也没有把恢复等待/离线虚拟时间/多次启动累计冒充 30 分钟连续运行。该项判定失败且验收时长未完成。

### 周期采样间隔（非执行耗时）

从 UDP 状态按 cycle 去重，以 `sample_time_ns` 差分计算。它包含调度和通知等待；回包遗漏会使部分差分跨多个周期，因此不能由此直接断言控制函数超时次数。

| 默认配置 | 唯一采样周期 | P50 μs | P95 | P99 | P99.9 | 最大 |
|---|---:|---:|---:|---:|---:|---:|
| 关节保持 | 61,483 | 1058.790 | 1210.384 | 1296.424 | 1527.644 | 3442.527 |
| 关节正弦 | 290,577 | 1059.504 | 1118.709 | 1200.633 | 1483.282 | 4512.825 |

未测得生产控制函数入口到出口的独立耗时分布、其准确超周期次数及最长连续超周期数；客户端 deadline 统计不能代替它们。不要将此项写为实时性通过。

### IK 0.1 ms 预算复核

真实 Talon Model，10,000 个确定性配置的近初值 IK；另做 1,000 个笛卡尔 ServoSignalProcessor 步。计时为墙钟时间，包括调度和包装开销；不等同于正在运行的生产控制线程。

| 测试 | 失败 | P50 μs | P95 | P99 | P99.9 | 最大 |
|---|---:|---:|---:|---:|---:|---:|
| 单次 IK 总墙钟 | 118/10000，全部 IkCalcFail | 53.194 | 123.281 | 217.943 | 447.504 | 3873.690 |
| 笛卡尔处理步总墙钟 | 28/1000，全部 IkCalcFail | 128.154 | 394.966 | 472.560 | 702.184 | 944.644 |
| 处理步内 IK 累计 | 同上 | 121.283 | 361.083 | 457.551 | 691.976 | 933.406 |

常规处理步可能调用两次 IK；首点容差验证还会额外调用一次 IK。本表的离线处理步不含额外首点检查、Executor/控制器/硬件输出及状态快照 FK，不能当作完整控制周期预算。

单次测试 861 个样本超过 100 μs；笛卡尔测试 198 个处理步至少包含一个超过 100 μs 的 IK 调用。保持了 0.1 ms 配置；建议优化求解器初始化/热启动/超时边界及失败策略，再验证，不以恢复 5 ms 掩盖问题。

![IK计时分布](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/ik_timing.png)

### 资源与观测开销

377 次约 1 Hz 资源采样：App RSS 23,688→24,012 KiB，FD 恒 10，socket 恒 3，线程 44–47 随控制启停变化；平均 CPU 约单核 10.33%。MuJoCo 平均单核约 15.28%。短观察期没有持续 FD/socket 泄漏证据，但不能据此判定 30 分钟无泄漏。

初版发送工具同步写文件导致约 31 ms 停顿，这些短试验仅保留作观测开销证据。关闭原始日志后 15 s 保持成功，随后改用有界 SPSC 队列和后台写盘；正式长测记录丢弃 0。名为 `smoke_joint_hold_2k` 的早期文件实际使用旧 1 kHz 二进制，**排除其作为 2 kHz 证据**。正式 2 kHz 文件带 `send_period_us=500`。详见 [stream.cc](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/stream.cc)、[resource_summary.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/resource_summary.json)。

## 9. 回包与 API 证据边界

正常关节样本中，同周期 q/q_d/q_cmd/时间/应用序号没有观测到字段变化；applied_message_id 未大于 received_message_id；有效处理后的 q_d 与 q_cmd 相同。该样本一致性不能证明跨进程 PDO 原子帧读取，Hardware 的前后时间戳一致也不足以排除生产者尚未更新 timestamp 的中间写入状态。

发现两个 P2 问题：

1. 等待首点、机器人非零姿态时，`output_valid=true`、q_cmd 是当前位置，但 q_d 全零；实测最大差约 0.5555 rad。[move_servo.cpp:484](/home/rocos/Documents/GitHub/rocos-app/src/move_servo.cpp:484) 仅在 processor_ready 时填入期望状态。建议用真实保持参考填充，或提供明确的参考有效标志。
2. NaN、超差首点、IK 等故障中，接收数据的 errors 位均未观察到 true，随后 UDP 沉默；日志/HTTP 才能确认故障。[move_servo.cpp:207](/home/rocos/Documents/GitHub/rocos-app/src/move_servo.cpp:207) 的 fail 先停止接收线程，故障快照发布与发送缺乏可靠终态交付机制。建议保留只读状态通道/最后故障查询，明确运行故障的最终通知契约。

`output_valid` 表示硬件接口写入流程未报错，不是驱动确认；`control_command_success_rate` 是新指令接收周期比例，不是驱动执行成功率。亚周期余量累计没有做可控时钟专项注入，本项未验证，不作通过声明。

按仓库要求已只读刷新 Apifox 在线 OpenAPI 并读取关联资源：[apifox_read_snapshot.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/apifox_read_snapshot.json)。在线项目 ROCOS API 的规范中尚无 RCI 路径；本报告的 RCI 行为来自当前实现与实测，不能冒充线上已发布契约。未更新 Apifox。

ResetFault 多次返回 -2307 而状态已变 IDLE，建议使“清故障并保持下使能”的 API 成功语义与状态机一致：[robot.cpp:863](/home/rocos/Documents/GitHub/rocos-app/src/robot.cpp:863)。此外，短暂两个 App 同时监听同一 HTTP 端口的现场说明启动端口独占并非防多实例保证；httplib 默认启用 SO_REUSEPORT。共享 PDO 多写者防护应单独评估；本次没有以双 App 阶段数据做伺服稳定性判断。

## 10. 修复优先级、未覆盖项与下一阶段

优先修复并回归：目标跟踪振荡；启停竞态与监督生命周期；主站/驱动独立失联停止；所有允许模式的最终输出导数约束；0.1 ms IK 可用性与实际实时线程预算；故障状态交付与首点前参考状态。每一项都应保留本次失败用例，不能通过放宽断言、限制或通信阈值消除失败。

仍需补测：修复后的四工况 30 分钟有效运行；任意初始导数状态的可行性边界；真实运动中的最后输出、停止距离和机械停稳时间；生产线程执行时间；网络固定延迟/积压/抖动矩阵；发送进程退出/终止；所有控制模式的非法反馈注入；在线 Lua 暂停恢复竞争、拖动模式预开启；笛卡尔真实 IK 的首点容差与非identity工具；跨进程原子反馈与驱动执行确认。矩阵中未执行项没有记为通过。

本次仿真不构成真实机器人安全认证。硬件验证的前置阻断条件是上述 P1 项得到修复并以同输入复测，型号 A/J/D/torque_rate 等能力经核定，主站/驱动失联行为及独立停止路径得到确认。随后才适合按单独批准的硬件方案验证。

测试结束状态：唯一 App 使用原 `config/app.yaml`，低通与限速器恢复默认开启；RCI disabled，机器人 IDLE，下使能；MuJoCo 原进程保留，无被暂停进程。见 [final_state.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/final_state.json)、[restart_original.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/restart_original.json)。本次没有执行返回原始零位的定位运动。

## 11. 复现与原始数据说明

在仓库根目录执行。下面离线用例无需启动新主站，不操作机器人：

```bash
python3 build/servo_validation_20261105/compile.py limiter_tracking_scan
./build/servo_validation_20261105/limiter_tracking_scan
python3 build/servo_validation_20261105/compile.py limiter_off
./build/servo_validation_20261105/limiter_off
./build/servo_validation_20261105/scan
./build/servo_validation_20261105/ik_bench
./build/servo_validation_20261105/admittance_audit --test-case='Audit*'
./build/servo_validation_20261105/cart_imp_audit --test-case='Audit*'
```

后两个新增 doctest 应失败，这是缺陷证据，不能改为期望成功来掩盖问题。线上工具 `stream`、`fault_stream`、`watchdog_suite.py`、`restart_app.py` 包含实际运动/暂停/重启操作；本报告保留命令与源码作复现记录，但不应在未重新核对 PID、当前配置、唯一仿真实例和控制权的环境直接重放。脚本中测试 PID 是本次现场值，不是通用部署参数。

正式长测调用形式：

```bash
AUDIT_PERIOD_US=500 chrt -f 30 ./build/servo_validation_20261105/stream 450 joint hold <output-prefix>
AUDIT_PERIOD_US=500 chrt -f 30 ./build/servo_validation_20261105/stream 450 joint sine <output-prefix>
```

`_states.bin` 每记录为 uint64 单调时钟纳秒 + packed RobotState（1096 字节）；`_commands.bin` 每记录为 uint64 纳秒 + packed MotionGeneratorCommand（236 字节）。解析布局见 `analyze.py`，正式聚合见 `metrics.py`；`fault_stream` 的发送记录还包含被故障注入逻辑故意跳过的样本，不能把全部记录数当成实际发送数。该工具无回复截止时间已改为 1.2 s，但 end_reason 文本仍写 100ms，属于工具标签缺陷，故障时刻请以日志/实际时间为准。

CSV、JSON、曲线均保留；`scan.csv` 和 `limiter_tracking_scan.csv` 为虚拟周期时间，不能算入在线运行时长。生产日志 `server_full.log` 和重新启动日志分别对应不同 PID，不应跨进程拼接为连续控制序列。

## 附录：逐项测试结果

| 编号 | 前置条件与输入 | 预期 | 实际结果 | 判定 | 证据 |
|---|---|---|---|---|---|
| B01 | 现有进程与共享内存；核对 executable、MuJoCo 库、配置、inode | 确认仿真且没有连接实物 | 同一 MuJoCo PID53021 / ecm_v2_0；各 App 连接相同 inode | 通过 | [user_simulation_identity.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/user_simulation_identity.json)、[restart_identity.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/restart_identity.json) |
| B02 | 当前源码与配置；记录 HEAD、脏文件、限制、哈希 | 保留用户修改 | 产品源码/配置哈希均未变化；临时关闭配置在产物目录 | 通过 | [git_head.txt](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/git_head.txt)、[baseline_hashes.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/baseline_hashes.json)、[source_integrity.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/source_integrity.json)、[configuration_hashes.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/configuration_hashes.json) |
| B03 | 单核构建；构建并执行11套相关回归 | 全部通过或记录基线失败 | 9套通过；model_test 3例、app_config_test 1例失败 | 失败 | [build.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/build.log)、[baseline_tests.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/baseline_tests.log) |
| R01 | RCI关闭、机器人STOPPED；无凭据 rci/start | 拒绝 | 返回-1018 | 通过 | [lifecycle_test.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/lifecycle_test.json) |
| R02 | 普通HTTP开启RCI；释放普通控制权后无凭据程序接入 | 接受并保持会话 | 所有在线stream均此方式接入 | 通过 | [soak_joint_hold.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/soak_joint_hold.json)、[off_joint_hold.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/off_joint_hold.json)、[stream.cc](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/stream.cc) |
| R03 | RCI开启、无UDP首点；观察保持及快速停止 | 无意外运动且停止完成 | 常规循环完成；第一轮快速循环卡STOPPING | 部分通过 | [server_full.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/server_full.log)、[rci_stop_timeout_state.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/rci_stop_timeout_state.json)、[lifecycle_test.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/lifecycle_test.json) |
| R04 | RCI开启；两个程序并发rci/start | 仅一个成功 | 第二轮明确一个成功一个拒绝 | 通过 | [lifecycle_test.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/lifecycle_test.json) |
| R05 | RCI独占；HTTP MoveJ/L/C、各点动、模式、TCP、负载、旧servo、script/run、takeover | 拒绝绕过 | 各返回-2215；该次现场为STOPPING/RCI独占 | 通过（限定状态） | [stuck_api.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/stuck_api.json) |
| R06 | RCI就绪、Fake/离线夹具；Robot公共入口、Lua运行/暂停/单步、debug token | 拒绝竞争 | 相关既有servo_robot_test通过 | 通过（回归级） | [baseline_tests.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/baseline_tests.log)、[servo_robot_test.cc](/home/rocos/Documents/GitHub/rocos-app/test/servo_robot_test.cc) |
| R07 | 独立RCI会话；HTTP lease释放/过期 | 不撤销独立RCI | 释放在线通过；过期仅既有回归通过 | 部分通过 | [soak_joint_hold.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/soak_joint_hold.json)、[baseline_tests.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/baseline_tests.log) |
| R08 | 会话正常停止；stop后再启 | 保留RCI开关、更换session | 第二轮100次唯一session，全部STOPPED | 通过（本轮） | [lifecycle_test.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/lifecycle_test.json) |
| R09 | 首轮快速启停；每次start后5ms stop，计划100次 | 每轮完成且线程存活 | 循环56轮完成，第57轮卡住；另有1次并发启动预试 | 失败 | [server_full.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/server_full.log)、[rci_timeout_full.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/rci_timeout_full.log)、[stop_timeout_threads.txt](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/stop_timeout_threads.txt) |
| R10 | 卡STOPPING现场；rci/disable、disable、reset | 关闭/恢复流程可结束 | RCI关闭仍pending；disable/reset拒绝 | 失败 | [stuck_api.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/stuck_api.json) |
| R11 | 故障后未恢复；再次rci/start | 拒绝旧会话再启动 | 返回-2215 | 通过 | [lifecycle_test.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/lifecycle_test.json) |
| R12 | 开启RCI前已有拖动/导纳模式；检查EnableRci工作模式限制 | 阻止非RCI运动通道 | 源码无工作模式检查；本次未在线切换拖动 | 未执行（在线） | [robot.cpp](/home/rocos/Documents/GitHub/rocos-app/src/robot.cpp:1984) |
| I01 | 关节伺服、当前位置；首点等于actual | 接受并保持 | 零位保持65s，无漂移，后通信保护 | 通过（首点） | [soak_joint_hold.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/soak_joint_hold.json)、[ranges.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/ranges.json) |
| I02 | 关节伺服、容差0.003rad；偏差0.00297rad | 接受且稳定趋向目标 | 接受，但2s后实际变化0.319rad；离线可复现振荡 | 失败（稳定性） | [fault_joint_first_inside.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/fault_joint_first_inside.json)、[limiter_tracking_scan.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/limiter_tracking_scan.log) |
| I03 | 非零实际姿态；偏差0.003rad边界 | 按实际采样容差判定 | HomePositionError，不能以HTTP旧采样声明恰好边界；Fake边界回归通过 | 部分通过 | [fault_joint_first_boundary.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/fault_joint_first_boundary.json)、[server_full.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/server_full.log)、[baseline_tests.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/baseline_tests.log) |
| I04 | 关节伺服；单轴/多轴偏差0.00303rad | 滤波前拒绝并故障 | HomePositionError、ERROR_STATE；未取得拒绝后完整PDO序列 | 部分通过 | [fault_joint_first_outside.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/fault_joint_first_outside.json)、[fault_joint_first_multi_outside.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/fault_joint_first_multi_outside.json)、[server_full.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/server_full.log) |
| I05 | 关节输入；NaN、Inf | 拒绝非法指令 | ParameterNanOrInf、ERROR_STATE | 通过（拒绝） | [fault_joint_nan.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/fault_joint_nan.json)、[fault_joint_inf.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/fault_joint_inf.json)、[server_full.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/server_full.log) |
| I06 | UDP接收；错误版本、session、短包、长包 | 忽略且不能续命 | 无回包、首点超时LostHeartbeat | 通过 | [fault_summary.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/fault_summary.json)、[server_full.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/server_full.log) |
| I07 | 笛卡尔输入；相同姿态、容差内外、不可能到达的x+10m | 正确区分首点容差与不可达 | 上述在线多数先IkCalcFail，遮蔽容差路径；不可达被拒绝 | 部分通过 | [fault_cartesian_first_inside.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/fault_cartesian_first_inside.json)、[fault_cartesian_first_outside.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/fault_cartesian_first_outside.json)、[fault_cartesian_cart_unreachable.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/fault_cartesian_cart_unreachable.json) |
| I08 | 笛卡尔输入；NaN、Inf、非法旋转、非identity工具 | 拒绝且坐标正确 | 非法旋转/工具在既有Fake回归覆盖；NaN/Inf未在线执行 | 部分通过 | [baseline_tests.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/baseline_tests.log)、[move_servo_test.cc](/home/rocos/Documents/GitHub/rocos-app/test/move_servo_test.cc) |
| I09 | UDP端口已被占用；rci/start绑定失败 | HTTP明确错误并回滚session | 返回-1016，ERROR_STATE，control_active=false | 通过 | [lifecycle_test.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/lifecycle_test.json) |
| I10 | 初始化参数；非法模式、零端口、无效周期/模型 | 拒绝初始化 | 既有回归通过；在线模式/端口试验未持久化响应，不能独立计通过 | 部分通过 | [baseline_tests.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/baseline_tests.log)、[rci_audit.py](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/rci_audit.py) |
| T01 | 静止保持；关节保持450s计划 | 稳定且无故障 | 65.479s退出，quality0.83触发保护；位置始终0 | 失败（持续性） | [soak_joint_hold.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/soak_joint_hold.json)、[ranges.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/ranges.json)、[server_full.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/server_full.log) |
| T02 | 低幅关节正弦；0.01rad、0.05Hz、5s渐入、7轴 | 平稳跟踪小幅输入 | 实际约±0.55rad、参考峰速2.892rad/s，309.119s退出 | 失败 | [ranges.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/ranges.json)、[soak_joint_sine.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/soak_joint_sine.json)、[simulation_tracking.png](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/simulation_tracking.png) |
| T03 | 离线真实信号处理器；同样正弦每1ms更新 | 约束与跟踪稳定 | 120s虚拟时间无失败，峰值0.01rad | 通过（离线） | [limiter_tracking_scan.csv](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/limiter_tracking_scan.csv)、[limiter_tracking_scan.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/limiter_tracking_scan.log) |
| T04 | 离线真实信号处理器；目标每2/3/10ms更新，中间保持 | 重复输入不自激 | 峰值0.549~0.565rad振荡，返回成功 | 失败 | [limiter_tracking_scan.csv](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/limiter_tracking_scan.csv)、[limiter_stability.png](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/limiter_stability.png) |
| T05 | 离线静止初态；固定0.00297rad目标 | 稳定收敛 | 持续振荡峰值0.551756rad，120s未收敛 | 失败 | [limiter_tracking_scan.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/limiter_tracking_scan.log)、[limiter_stability.png](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/limiter_stability.png) |
| T06 | 边界与停止扫描；1620组位置、A、J、D、方向、目标、停止时刻 | 遵守约束且可停 | 0失败；每例最终AtRest | 通过（有限样本） | [scan.cc](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/scan.cc)、[scan.csv](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/scan.csv)、[scan.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/scan.log) |
| T07 | 任意非零初始速度/加速度；不依赖Reset静止态的全状态扫描 | 区分初始不可行与算法导致不可行 | 扫描中动态状态覆盖，但未穷举任意初始化状态 | 未执行（全状态证明） | [scan.cc](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/scan.cc) |
| T08 | 笛卡尔保持/正弦450s计划；真实Talon、0.1ms IK | 稳定运行 | 各约0.1s客户端退出，日志IkCalcFail | 失败 | [soak_cartesian_hold.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/soak_cartesian_hold.json)、[soak_cartesian_sine.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/soak_cartesian_sine.json)、[server_full.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/server_full.log) |
| C01 | 2kHz发送、控制1kHz；单次约1ms漏发、10%确定性随机丢包 | 不误接受坏包、保护行为合理 | 2s完成；微小初始偏差引起放大，运动稳定性不通过 | 部分通过 | [fault_joint_loss_single.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/fault_joint_loss_single.json)、[fault_joint_loss_random.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/fault_joint_loss_random.json) |
| C02 | 已接入；60ms连续漏发/永久沉默 | 触发保护且不自动恢复 | missing_cycles20触发；随后ERROR_STATE | 通过（检测与故障转换） | [fault_joint_loss_burst.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/fault_joint_loss_burst.json)、[fault_joint_silent.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/fault_joint_silent.json)、[server_full.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/server_full.log) |
| C03 | 重复包；始终id=1 | 不续命 | LostHeartbeat | 通过 | [fault_joint_duplicate.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/fault_joint_duplicate.json)、[server_full.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/server_full.log) |
| C04 | 跨批次乱序；交替发送回退序号 | 不覆盖新指令 | 拒绝旧序号的既有回归通过；在线最终低质量停止 | 部分通过 | [fault_joint_reorder.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/fault_joint_reorder.json)、[baseline_tests.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/baseline_tests.log) |
| C05 | 已绑定UDP端口；另一端口发id>100000、偏离目标 | 不能抢占，不能错误回包 | foreign回复0，received最大146，仍SERVOING | 通过 | [peer_test.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/peer_test.json)、[peer_summary.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/peer_summary.json)、[other_endpoint_states.bin](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/other_endpoint_states.bin) |
| C06 | 重启会话；同一端口发旧session大序号 | 旧包隔离 | received最大163，未接受大序号 | 通过 | [peer_test.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/peer_test.json)、[peer_summary.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/peer_summary.json)、[old_session_states.bin](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/old_session_states.bin) |
| C07 | 发送频率；1kHz与2kHz | 评估质量和稳定性 | 1kHz保持约0.308s故障；2kHz也长期退出 | 失败（可持续性） | [smoke_joint_hold.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/smoke_joint_hold.json)、[soak_joint_hold.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/soak_joint_hold.json) |
| C08 | 队列积压/固定延迟/分布抖动；定量网络注入 | 给出分布与恢复行为 | 未做tc/netem；线程500us轮询和有界排空仅源码检查 | 未执行 | [move_servo.cpp](/home/rocos/Documents/GitHub/rocos-app/src/move_servo.cpp:518) |
| C09 | 发送程序异常；SIGSTOP/退出/SIGKILL分别测试 | 均触发保护 | 已用同等断流测试；未分别杀死发送进程 | 未执行（进程级） | [fault_joint_silent.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/fault_joint_silent.json) |
| W01 | 原MuJoCo实例、等待首点；主站SIGSTOP80ms后恢复 | 有界等待/独立停止 | 19.912ms观测到控制字2，恢复后ERROR | 通过（停止请求） | [watchdog_suite.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/watchdog_suite.json)、[watchdog_summary.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/watchdog_summary.json)、[master_pause_shm.csv](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/master_pause_shm.csv) |
| W02 | 仅控制线程冻结；ptrace暂停80ms | 独立监督有效 | 19.198ms写控制字2，主站持续更新，恢复后ERROR | 通过（停止请求） | [watchdog_suite.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/watchdog_suite.json)、[control_pause_shm.csv](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/control_pause_shm.csv) |
| W03 | 整个App冻结；SIGSTOP约81.4ms | 主站/驱动侧独立超时保护 | 暂停期间控制字仍15；恢复后82.459ms才见2 | 失败（端到端覆盖） | [watchdog_summary.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/watchdog_summary.json)、[app_pause_shm.csv](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/app_pause_shm.csv) |
| W04 | 运动中保护停止；PDO停止请求/最后目标/实际停稳分别计时 | 完整停止时序 | W01~03为静止等待；运动中只有日志/UDP，机械停稳时间未量出 | 未执行（完整时序） | [watchdog_suite.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/watchdog_suite.json)、[fault_joint_silent.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/fault_joint_silent.json) |
| F01 | 实际反馈非法；位置/速度越界、跟随误差、过期 | 合法目标不能掩盖坏反馈 | position_controller等既有回归通过 | 通过（回归级） | [baseline_tests.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/baseline_tests.log)、[position_controller_test.cc](/home/rocos/Documents/GitHub/rocos-app/test/position_controller_test.cc) |
| F02 | 共享内存实际反馈；统一注入NaN/Inf、时间戳停滞、跳变 | 所有模式一致故障 | 主站暂停覆盖停更；未在线篡改PDO，未穷举所有模式 | 部分通过 | [master_pause_shm.csv](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/master_pause_shm.csv)、[baseline_tests.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/baseline_tests.log) |
| F03 | 关节阻抗；首次/Reset力矩变化率 | 限制生效 | 现有测试通过；切换/机械停止跳变未实测 | 部分通过 | [baseline_tests.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/baseline_tests.log)、[joint_impedance_controller_test.cc](/home/rocos/Documents/GitHub/rocos-app/test/joint_impedance_controller_test.cc) |
| F04 | 导纳控制器+显式A10/J150配置；外力矩0到8Nm、M1/B10 | 最终输出遵守jerk | 输出jerk8000，函数返回成功 | 失败 | [admittance_audit.cc](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/admittance_audit.cc)、[admittance_output.csv](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/admittance_output.csv)、[admittance_test.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/admittance_test.log) |
| F05 | 笛卡尔阻抗首次输出；当前力矩0、重力5、200Nm/s、1ms | delta<=0.2Nm | delta5Nm，返回成功 | 失败 | [cart_imp_audit.cc](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/cart_imp_audit.cc)、[cart_imp_first.csv](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/cart_imp_first.csv)、[cart_imp_test.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/cart_imp_test.log) |
| L01 | 四种450s计划；累计目标1800s | 至少30min稳定运行 | 4工况全部提前退出，最长309s；不能用重启时间凑30min | 失败/未完成 | [soak_progress.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/soak_progress.json) |
| L02 | 真实Model、IK预算0.1ms；10000单次IK+1000双IK伺服步 | 收敛且时间可控 | 118与28次IkCalcFail；单次最大3874us | 失败 | [ik_bench.csv](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/ik_bench.csv)、[ik_summary.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/ik_summary.json)、[ik_timing.png](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/ik_timing.png) |
| L03 | UDP状态采样；去重周期号、统计sample时间间隔 | 给出分位与边界 | 已统计；不是控制函数执行耗时，缺失周期影响最大值 | 部分通过 | [metrics.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/metrics.json) |
| L04 | 服务端实际计算耗时；逐周期入口出口计时 | 工作时间分位/超周期连续次数 | 未做生产线程入口出口埋点；只有离线IK与客户端统计 | 未执行 | [ik_summary.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/ik_summary.json)、[soak_joint_sine.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/soak_joint_sine.json) |
| L05 | 377秒资源采样；RSS/CPU/FD/socket/线程 | 无持续泄漏 | App RSS+324KiB、FD/socket不增长，观察期不足30min | 部分通过 | [resource_summary.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/resource_summary.json)、[resources.jsonl](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/resources.jsonl) |
| L06 | 100次启动停止；第二轮限速关闭、不发运动点 | 状态/资源一致 | 100次完成、100个唯一session、FD9/9、线程46/46 | 通过（该轮） | [lifecycle_test.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/lifecycle_test.json) |
| S01 | 正常已应用关节命令；同周期回包比较 | 一致周期快照 | 采样内字段无同周期变化，applied<=received，qd=qcmd | 通过（观测样本） | [metrics.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/metrics.json) |
| S02 | 等待首点、非零实际姿态；检查q_d/q_cmd | 期望保持参考有效 | output_valid=true但q_d为0、q_cmd非零 | 失败 | [soak_cartesian_hold_states.bin](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/soak_cartesian_hold_states.bin)、[metrics.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/metrics.json) |
| S03 | 首点非法或运行故障；UDP错误终包 | 外部能可靠得知错误 | 这些在线用例收到的包errors均false，随后无回包；日志有故障 | 失败（终态可见性） | [fault_summary.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/fault_summary.json)、[server_full.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/server_full.log) |
| S04 | 通信质量；区分接收/硬件执行 | 语义明确 | 字段为新指令比例；没有驱动执行ACK | 通过（语义确认） | [servo_type.hpp](/home/rocos/Documents/GitHub/rocos-app/src/servo_type.hpp:70) |
| S05 | 短周期累计；交替亚周期延迟专项注入 | 累计正确 | 未直接注入可控时钟；不能仅凭源码认定通过 | 未执行 | [move_servo.cpp](/home/rocos/Documents/GitHub/rocos-app/src/move_servo.cpp:299) |
| S06 | 跨进程PDO快照；验证原子读一致性 | 完整反馈帧一致 | 时间戳前后校验不能证明PDO写入原子帧 | 未验证 | [hardware.cpp](/home/rocos/Documents/GitHub/rocos-app/src/hardware.cpp:1216) |
| O01 | 按用户要求关闭rate limiter；8组离线同输入、低通仍100Hz | 对照定位且必要检查保留 | 平滑1ms通过；固定偏差/重复目标被AccLimit/JerkLimit拒绝 | 通过（对照） | [limiter_off.cc](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/limiter_off.cc)、[limiter_off.csv](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/limiter_off.csv)、[limiter_off.log](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/limiter_off.log) |
| O02 | 同一App临时配置关闭limiter；关节保持/正弦、笛卡尔保持/正弦各15s计划 | 稳定且不降低保护 | 保持15s完成；其他3项jerk/IK退出 | 部分通过 | [off_suite.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/off_suite.json)、[off_joint_hold.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/off_joint_hold.json)、[off_joint_sine.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/off_joint_sine.json) |
| O03 | 关闭对照结束；重启原配置并清理 | 原配置、单App、RCI关闭、IDLE | 已恢复，PID85599，保留原MuJoCo | 通过 | [restart_original.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/restart_original.json)、[final_cleanup.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/final_cleanup.json)、[final_state.json](/home/rocos/Documents/GitHub/rocos-app/build/servo_validation_20261105/final_state.json) |
