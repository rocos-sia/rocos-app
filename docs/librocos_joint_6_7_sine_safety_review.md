# 关节零位正弦示例与真机安全检查

## 远程 IPv4 接入

示例增加 `--ip <控制器IPv4>`，默认地址为 `127.0.0.1`。App 的 RCI start 支持远程 IPv4，UDP 监听 `0.0.0.0`。启用 RCI 仍走原有控制权流程，启动仍要求 STOPPED、入口已开启与独占会话。会话关联发起 HTTP 请求的源 IPv4，首个有效正序号运动包进一步绑定 UDP 源端口。查询、旧会话、重复包、乱序包与其他来源继续受现有检查约束。

控制器启动时应配置可从客户端访问的 HTTP 监听地址，例如 `0.0.0.0`。通信端口为配置的 HTTP TCP 端口及指定的 UDP 端口，默认分别为 8080、8081。HTTP 与 UDP 使用同一客户端源 IPv4。当前没有纯 IPv6 UDP 支持。

在另一台电脑准备同版本 SDK、示例依赖及本地有效配置。导出配置中的 `robot.urdf` 需指向客户端本地可读的相同 URDF，程序再通过远程 API 验证模型原文和关节标定。IP 替换为控制器实际可达地址，输出目录位于运行示例的客户端。

```bash
sudo ./build/librocos-standalone/bin/librocos_joint_6_7_sine \
  build/j67-effective/effective.yaml build/remote-real-001 \
  --ip 192.168.1.10 --all-joints --realtime --duration 3600
```

两台电脑无需对齐单调时钟起点。SDK 保留服务端采样周期与时间戳递增检查，用客户端本地时间计算反馈接收超时和回调耗时，并通过已应用指令序号查找本地发送时间、检查回执时效。回调周期仍由服务端相邻采样时间差计算。首点、模型边界、跟随误差、限速、停止和通信阈值继续由原有控制链执行。此前真机记录发现的 EtherCAT PDO 有效性问题需单独修复和验证。

远程接入修改已单核编译验证。SDK 19 个离线用例通过，涵盖非默认目标 IPv4、服务端时钟提前 30 天或落后 1 小时、应用回执停滞、断流、异常清理和示例数据采集。App 离线控制器共 20 个用例通过，新增 HTTP 来源 IPv4 关联、UDP 错误源 IP/端口拒绝、纯 IPv6 拒绝检查。sudo 下远程 SDK 与示例 2 个用例通过。使用本机非回环地址 `170.170.170.100` 的完整 HTTP/UDP 离线主站测试通过，完成 3 次会话、11624 次回调，最大参考跟随误差约 4.08×10^-6 rad，最低接收比例 0.97。该测试使用独立共享内存主站；两台实体电脑之间的链路及真机运动待现场验证。

在线 Apifox 已更新 `/api/robot/rci/start` 的 IPv4、来源绑定与时间戳语义，并刷新 MCP 验证。前后导出核对仅此路由改变，模型、鉴权定义和其他路由内容一致。导入过程曾增加鉴权目录层级，已恢复定义原有目录；导入新建的空目录可能仍存在。验证日志位于 `build/j67_validation/remote-*.log`。

## 七轴工况与 1 ms 轨迹时间

新增 `--all-joints` 选择本轮七轴工况。J1/J2/J3/J5 为零中心 ±10°，J4/J6/J7 为零中心 ±50°。各轴同相，频率 0.2 Hz，运动 180 s，含首尾各 5 s 过渡，再保持零位 1 s。幅值按两组固定，`--all-joints` 与 `--amplitude-deg` 同时使用会报参数错误。沿用 `librocos_joint_6_7_sine` 程序名；未传入 `--all-joints` 时使用下文记录的双轴工况。

轨迹时间使用 `(state.cycle - start_cycle) * 0.001`，控制周期每增加一次，轨迹时间增加 1 ms。反馈周期跳号时按实际周期差推进。软件采样时间继续用于反馈时效和回调耗时检查。CSV 的 `time_s` 为轨迹时间，`sample_ns` 和 `dt_s` 保留实际软件采样时刻与间隔。分析脚本从 `sample_ns` 计算采样抖动，并按 metadata 的 `joint_amplitudes_rad` 分析所有运动轴。

启用七轴工况前，须通过独立定位运动把七轴全部定位至零位并等待 STOPPED。示例逐轴检查起点、模型/硬件范围、V/A/J 和最终反馈。J=10000。七轴联合运动的机械环境、碰撞与型号能力需现场确认。新增参与轴及幅值会改变机械工况，后续实际跟踪误差对比需同时考虑这些变化。

先进行离线预检

```bash
./build/librocos-standalone/bin/librocos_joint_6_7_sine \
  build/j67-effective/effective.yaml build/joint-real-003 --all-joints --check
```

运行并采集数据，核对启动打印的七轴幅值后按 Enter

```bash
sudo ./build/librocos-standalone/bin/librocos_joint_6_7_sine \
  build/j67-effective/effective.yaml build/joint-real-003 --all-joints --realtime
```

采集后生成七轴误差、幅相、加速度/jerk 触限比例和通信报告

```bash
sudo env PYTHONNOUSERSITE=1 python3 librocos/examples/analyze_joint_6_7_sine.py \
  build/joint-real-003
```

本轮示例单核编译通过，SDK 离线测试 18 个用例通过。独立对端逐行验证 CSV 轨迹时间等于控制周期差乘以 0.001，在非等间隔软件时刻下验证七轴幅值、完整结束与零位起点拒绝；sudo 下该集成用例也通过。七轴限值预检通过。分析脚本已用旧双轴真机数据及明确标注的七轴合成数据验证，采样抖动统计仍使用实际时刻。验证产物位于 `build/j67_validation/cycle-time-*`，真机本轮数据待现场采集。

## 交付参数

示例 `librocos_joint_6_7_sine` 通过 librocos 的 `Robot::control()` 发送关节位置。J6、J7 对应数组索引 5、6，两轴以控制器绝对关节零位为中心，同相运动。J1–J5 的命令取会话首帧参考并保持该值。

| 参数 | 设置 |
|---|---|
| J6、J7 范围 | 各自 −60°～+60° |
| 正弦频率 | 0.2 Hz，周期 5 s |
| 运动时间 | 180 s，含首尾各 5 s 幅值过渡 |
| 零位保持 | 运动结束后 1 s，总计划会话约 181 s |
| 发送周期 | 1 ms |
| 启动零位容差 | 0.0001 rad，约 0.00573° |
| 启动与结束静止检查 | 各相关实际速度不超过 0.001 rad/s |
| J6、J7 配置 jerk | 10000 rad/s³ |
| 数据 | 逐回调 CSV、运行参数与终态 JSON、离线统计和图形报告 |

同相、5 s 幅值过渡和 1 s 结束保持是本轮的实现选择。实际零位超出容差时，程序明确提示先执行独立 MoveJ 定位。位置零位采用控制器标定坐标。

## 安全检查结论

**已有软件保护覆盖了多个主要故障路径，真机保护的完备性仍需主站、驱动和实物验证。** 本轮完成源码核查、轨迹数学验证、独立协议对端测试及分析脚本验证。真机运动结果由现场运行采集。

| 检查项 | 源码中的现有实现 | 本轮结论 |
|---|---|---|
| 首点容差 | MoveServo 对首个原始目标与实际配置比较，超限报 `HomePositionError`，进入故障停止 | 示例再增加 J6/J7 零位检查，HTTP 预检与首个 UDP 回调均检查 |
| 目标与实际位置边界 | `ValidateJointCommand`、`ValidateFeedback` 与 Model 位置限制 | 示例取生效硬件配置与 URDF 边界交集，并检查全轴反馈及最终位置输出 |
| 速度、加速度、jerk | Model 逐周期限速、候选停止轨迹检查、信号处理末端再验证 | J=10000，轨迹生成前验证包含启动包络的导数上界 |
| 实际跟随 | 位置误差及速度误差检查，故障并入停止流程 | 当前默认位置容差为 V×dt×20，V=3、dt=1 ms 时为 0.06 rad，约 3.44°。型号容差需核定 |
| 反馈新鲜度 | 硬件源时间检查，默认 20 个硬件周期 | 需结合真实主站时间戳发布方式与时钟行为验证 |
| UDP 通信 | 会话 ID、发送端、递增序号、缺失和接收质量保护 | SDK 另外检测陈旧反馈、回调超时和最终目标提交确认 |
| 控制线程阻塞 | 有界主站等待与独立监督线程，超时写入安全停止请求 | 配置阈值对应触发条件，物理停稳时间取决于调度、总线和驱动 |
| 正常退出 App | 主循环收到 SIGINT/SIGTERM 后执行 `robot.setDisabled()` | 正常退出有下使能路径 |
| 整个 App 挂起或强制终止 | 进程内监督线程同时失去执行机会 | 独立主站心跳、驱动失联动作及其时限尚需真机验证 |
| Quick Stop | `Hardware::RequestSafetyStop()` 向控制字写入 `0x0002`，并锁存停止标志 | 驱动具体减速方式、响应、状态字确认与抱闸配合尚需核实 |
| 碰撞与机械环境 | 当前关节位置链检查关节与导数约束 | 工具碰撞、自碰撞、外力异常及工作空间安全需要实物场景评估 |
| 已知阶跃问题 | 前轮记录两项 J=150 阶跃超调回归失败 | 该问题继续列为已知限制，本示例使用平滑连续目标 |

主要源码入口包括 [MoveServo](/home/rocos/Documents/GitHub/rocos-app/src/move_servo.cpp)、[反馈校验](/home/rocos/Documents/GitHub/rocos-app/src/feedback_validation.hpp)、[伺服信号处理](/home/rocos/Documents/GitHub/rocos-app/src/servo_signal_processor.cpp)、[硬件等待与停止](/home/rocos/Documents/GitHub/rocos-app/src/hardware.cpp)、[控制线程监督](/home/rocos/Documents/GitHub/rocos-app/src/robot.cpp) 和 [App 退出流程](/home/rocos/Documents/GitHub/rocos-app/src/rocosAppMain.cpp)。

### 配置与型号能力

Apifox 在线规范于 2026-11-10 刷新并读取。`/api/robot/info` 提供现有关节名称、ID、编码器比例和零点信息，模型身份与主站周期的部分字段为占位。当前契约没有运行中的完整位置、加速度、jerk 和伺服设置查询。

因此示例使用 **实际启动配置导出的 `effective.yaml`**，读取硬件限制与对应 URDF。运行前再读取控制器的 `/api/robot/urdf` 和 `/api/robot/info`，核对模型原文、关节名称、ID、单位、编码器比例与零点。运行中的导数限制与导出文件一致性仍由配置管理确认。预检拒绝 1 ms 以外的配置周期、缺失限值、异常编码器比例、型号/关节链不符、关闭通信保护或关闭限速器的配置。

当前 Talon J6 位置配置为 [−2.11, 2.11] rad，J7 为 [−2.84, 3.14] rad。零中心的 ±60° 位于这些范围内。J=10000、A=10、V=3 是本次软件配置，机械型号能力、驱动参数与负载下的可用范围需要独立核定。

### 轨迹与导数

两轴共同采用

\[
q(t)=A\,r(t)\sin(\omega t),\qquad A=\pi/3,\quad \omega=2\pi\cdot0.2.
\]

幅值包络使用七次多项式

\[
s(u)=35u^4-84u^5+70u^6-20u^7.
\]

前 5 s 取 \(r(t)=s(t/5)\)，中段取 1，后 5 s 取 \(r(t)=s((180-t)/5)\)。连接点的前三阶导数连续，开始和结束的位置、速度、加速度、jerk 均为零。

\[
\dot q=A(\dot r\sin\omega t+r\omega\cos\omega t),
\]

\[
\ddot q=A[(\ddot r-r\omega^2)\sin\omega t+2\dot r\omega\cos\omega t],
\]

\[
q^{(3)}=A[(r^{(3)}-3\dot r\omega^2)\sin\omega t+
(3\ddot r\omega-r\omega^3)\cos\omega t].
\]

示例采用 \(|\dot r|\le2.1875/T_r\)、\(|\ddot r|\le8/T_r^2\)、\(|r^{(3)}|\le53/T_r^3\) 的保守包络上界进行预检。

| 导数 | 稳态正弦峰值 | 含启停包络的保守上界 | 当前配置 |
|---|---:|---:|---:|
| 速度 rad/s | 1.31595 | 1.77410 | 3 |
| 加速度 rad/s² | 1.65367 | 3.14023 | 10 |
| jerk rad/s³ | 2.07806 | 5.95582 | 10000 |

上述界对应连续时间的目标函数。在线采样与调度会改变逐周期差分，App 的限速和保护继续处理这些扰动。原先 ±60°、10 Hz 的目标要求峰值速度约 65.8 rad/s、加速度 4134 rad/s²、jerk 259758 rad/s³，示例会在启动之前拒绝该组合。

## 构建与运行

示例增加 yaml-cpp、TinyXML2 依赖，用于读取有效配置和 URDF。数据分析使用 NumPy、Matplotlib。

```bash
cmake -S librocos -B build/librocos-standalone -DCMAKE_BUILD_TYPE=Release
cmake --build build/librocos-standalone --parallel 1
ctest --test-dir build/librocos-standalone --output-on-failure -j1
```

使用真机 App 实际采用的启动配置导出有效配置，以下命令中的 `config/app.yaml` 应对应该启动配置。

```bash
./build/bin/rocosAppMain --config=config/app.yaml --check_config \
  --export_config=build/j67-effective

./build/librocos-standalone/bin/librocos_joint_6_7_sine \
  build/j67-effective/effective.yaml build/j67-real-001 --check
```

`--check` 完成离线配置和轨迹预检后返回。运行准备包括驱动/主站与工具环境检查、位置模式、使能后的 `STOPPED`，以及通过独立 MoveJ 将 J6、J7 定位至零位。J1–J5 的姿态由现场确定。

```bash
./build/librocos-standalone/bin/librocos_joint_6_7_sine \
  build/j67-effective/effective.yaml build/j67-real-001
```

程序先读取配置、模型和当前状态，输出轨迹与限制，等待 Enter。确认后再次核对模型和零位，开启 RCI 并运行。输出目录应使用新名称，例如下一轮使用 `build/j67-real-002`。

支持 `--amplitude-deg`、`--frequency`、`--duration` 和 `--udp-port`。默认参数即本报告的 60、0.2、180、8081。通信使用本机回环地址，HTTP 端口来自有效配置。

### 实时调度对照

2026-11-10 首轮真机数据保存在 `build/j67-real-001`，约 14.06 s 后因窗口接收质量持续低于 0.9 进入故障停止。实时调度对照使用新的目录 `build/j67-real-002`。

App 控制线程和伺服 UDP 接收线程在启动时请求 `SCHED_RR/20`，独立监督线程请求 `SCHED_RR/21`。已有更高的 FIFO/RR 优先级会保留。每个线程打印请求值、实际策略与优先级、设置错误码。权限不足时 App 明确警告并沿用实际调度，现场对照应检查这些日志均显示申请成功。

示例的 `--realtime` 请求当前发送/回调线程使用 `SCHED_RR/30`。失败时保存错误并在开启运动会话前退出。CSV 写盘线程使用 `SCHED_OTHER`。这些优先级作为首轮对照参数，具体周期截止时间和共享锁等待仍需实测。App 对控制线程的设置也作用于其他运动模式。

编译后在上位机下使能并正常退出旧 App，保持主站运行，再启动新 App。重新定位、使能并确认 STOPPED 后运行示例。

```bash
sudo ./build/bin/rocosAppMain --config=config/app.yaml
```

另一个终端执行

```bash
sudo ./build/librocos-standalone/bin/librocos_joint_6_7_sine \
  build/j67-effective/effective.yaml build/j67-real-002 --realtime
```

输入 Enter 后应看到 `Client scheduling` 的实际策略为 `SCHED_RR/30`，App 对应输出 `Control scheduling`、`Watchdog scheduling` 和 `UDP scheduling`。可通过线程名核验

```bash
ps -eLo pid,tid,cls,rtprio,psr,comm | rg 'rocos_control|rocos_watchdog|rocos_servo_udp|librocos_servo'
```

运动参数继续使用零中心、±60°、0.2 Hz、180 s、J=10000。结束后仍使用同一分析脚本处理新目录。sudo 运行会使新目录属于 root，默认权限下普通用户可读取数据；分析脚本写入该目录时采用

```bash
sudo env PYTHONNOUSERSITE=1 python3 librocos/examples/analyze_joint_6_7_sine.py \
  build/j67-real-002
```

本轮实时调度修改验证结果如下。

- App、示例与测试目标编译通过。构建命令均使用 `--parallel 1`，过程中两条构建任务曾短暂重叠，后续按串行完成。
- 普通用户 SDK 测试通过，共 18 个用例，覆盖实时权限不足时退出并保持零运动会话。
- sudo 下调度与独立协议对端测试 2 个用例通过，实际获得 `SCHED_RR/30`，覆盖中断及数据保存。
- sudo 下独立共享内存主站的控制器测试 19 个用例通过，日志确认控制与 UDP 为 RR20、监督线程为 RR21，包含反复启停及主站断流保护。
- `move_servo_test` 为 48/50 通过，原有两项低 jerk 阶跃目标收敛回归继续失败，分别为 `Servo regression small targets converge without self-excited motion` 和 `Servo adaptive tracking settles repeated small targets and reversals`。
- 测试日志位于 `build/j67_validation/realtime-*.log`。真机对照结果由现场下一轮采集。

Ctrl-C/SIGTERM 请求服务端受控停止，退出记录标为 interrupted。正常结束保持零位 1 s，并检查实际位置和速度，再确认会话终止。正常终态为 `STOPPED`，RCI 为开启状态；下使能可通过上位机执行。故障或停止确认失败会返回非零退出码，终态信息写入 `outcome.json`。

## 数据采集与分析

回调将固定大小样本写入预分配的 8192 条单生产者/单消费者队列。普通调度的后台线程负责 CSV 写盘。队列溢出或写盘失败会触发会话停止并标记数据错误。运行日志文件包括

- `metadata.json`，轨迹参数、读取的控制器信息、配置路径和使用的限值。
- `samples.csv`，采样时间、实际回调间隔、控制周期、收到/应用序号、接收质量、原始生成目标、实际位置/速度/力矩、处理后参考及最终下发位置。
- `outcome.json`，完成或中断标志、异常信息、写盘状态、已写样本数、最后故障码及最后周期位置/速度。

结束或故障后执行

```bash
PYTHONNOUSERSITE=1 python3 librocos/examples/analyze_joint_6_7_sine.py \
  build/j67-real-001
```

脚本生成 `report.md`、`analysis.json`、`analysis.png`，包含 J6/J7 的误差 RMS/P99/峰值、稳态幅值比、单频等效相位滞后、结束偏移、限值与跟随误差统计、通信接收比例及采样间隔。J1–J5 的实际位移也会统计。

`requested` 是当前回调生成并供下一次发送的目标，与当前反馈比较的误差包含客户端时序。`q/q_d/q_cmd` 来自同一服务端快照。CSV 保存应用序号，逐发包原始目标映射需要额外抓包。参考 jerk 仅在连续控制周期样本间统计，完整峰值可结合 App trace。记录的实际力矩依赖驱动标定与硬件配置。

## 本轮验证记录

- 单核编译通过，独立 librocos CTest 通过，其中包含 17 个 doctest 用例。
- 默认 180 s、0.2 Hz、±60° 配置的离线预检通过。
- 轨迹测试逐毫秒扫描 180 s，在 0.2 Hz 和 0.23 Hz 下验证位置及前三阶导数上界、包络连接连续性，并验证超限组合被拒绝。
- 独立协议对端运行 10 s 运动加 1 s 保持，检查 CSV 完整行数、J1–J5 命令恒定、J6/J7 同相及幅值边界。零位不符和 10 Hz 超限输入在建立运动会话前被拒绝。
- Ctrl-C 和 UDP 反馈断流注入后，客户端完成停止清理，保留数据与异常终态。
- 分析脚本使用显式标注的合成数据验证，注入幅值比 0.98、总滞后 25 ms、参考处理 2 ms、实际跟随 23 ms，恢复结果误差小于 10⁻⁷。跳过中间控制周期时，jerk 统计标记可用范围。

验证产物位于 [j67_validation](/home/rocos/Documents/GitHub/rocos-app/build/j67_validation)。本轮验证数据覆盖数学函数与软件路径，现场真机报告将在实际采集后生成。
