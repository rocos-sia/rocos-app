# UDP 伺服实现说明

HTTP 契约以 Apifox 项目“ROCOS API”在线 OpenAPI 为准。2026-11-05 已刷新并核对 servo/start、servo/mode、servo/stop 及响应定义。以下记录实现行为；调节参数放入机器人 YAML。UDP v2 及启动响应的会话字段是本轮实现扩展，在线契约尚待同步。

## 模型限速函数

`Model` 继承的 `GetPositionVelocityLimits(q, deceleration, velocity_offset, lower, upper)` 实现在 `src/model_limits.cpp`。位置上下限和额定速度通过模型 getter 获取，使用 URDF 导入的轴序和限制；显式修改模型位置限位后，计算同步生效。函数无状态、不读取硬件、不修改输入，任何失败保留输出。

采用 [Franka 位置相关速度包络公式](https://frankarobotics.github.io/docs/robot_specifications.html#position-based-velocity-limits)：

```text
upper[i] = min(vmax[i], max(0, sqrt(2 * dec[i] * (qmax[i] - q[i])) - offset[i]))
lower[i] = -min(vmax[i], max(0, sqrt(2 * dec[i] * (q[i] - qmin[i])) - offset[i]))
```

有限关节越界直接报错，不通过开方钳位掩盖。连续关节的无限位置范围保留，速度仍限于 URDF 的额定速度。NaN、非法范围、非正制动参数及维度错误均拒绝。旋转关节单位为 rad，移动关节单位为 m。

当前 URDF 解析器只导入位置、速度和 effort，没有导入加速度、jerk、制动减速度及速度偏置。初始化轴绑定按关节名称建立对应关系，避免把 drive id 当作列表索引。额外参数由 `Hardware::GetJointDerivativeLimits()` 按 model index → drive id 绑定读取机器人硬件 YAML 的 `limit.acc/jerk/deceleration/velocity_offset`。`deceleration: 0` 表示沿用 `acc`，`velocity_offset` 默认 0；不复制 FR3/FER 的标定数值。现有 acc/jerk 缺省值仍沿用硬件配置规则，设备部署前应填入该机器人的核定值。该速度包络本身不是 jerk 受限制动距离证明。

## 信号处理和必要条件

处理顺序：原始报文/首点校验 → 一阶低通 → 关节轨迹速率限制 → 输出必要条件校验 → 控制器。

- 首点与实际反馈比较；每轴容差使用模型额定速度及当前位置方向速度包络中更严格者乘以控制周期。超限返回现有 `HomePositionError`（-2306），记录 `start_pose_invalid`，锁存故障并进入 ERROR_STATE。滤波器不能把错误首点修正成合法首点。需要接近起点时，先显式执行定位运动。
- 首次启动还检查反馈速度相对 `acc * dt` 的静止容差。位置越界、NaN/Inf、非法齐次矩阵、IK 失败均拒绝。
- 一阶低通默认 100 Hz，`alpha = dt / (dt + 1/(2*pi*fc))`，保存独立的滤波状态，参考上一周期滤波后的指令（不反馈限速后的输出）。笛卡尔平移线性滤波、旋转沿 SO(3) 最短旋转插值，避免逐元素滤波破坏旋转矩阵。
- 笛卡尔原始目标先做 IK 和位置检查，滤波后再次求 IK，随后统一进入关节速率限制。输入及反馈 TCP 使用启动时锁定的活动工具变换；输出向控制器传关节位置，避免再次求 IK 产生不同分支。
- 伺服采用 Franka 风格的逐周期差分、约束与积分，不使用 Ruckig 重规划目标点。Model 中的 `LimitServoPositionStep` 使用三阶阻尼反馈 `jerk = w³(target-q) - 3w²v - 3wa`，`w=min(0.1/dt, 0.5*jerk_limit/acc_limit)`，由当前位置、速度和加速度共同确定减速；该跟踪律有相位滞后，不承诺逐点无延迟。`LimitServoJointStep` 约束加速度和 jerk；`MakeServoBrake` 检查从候选状态出发的完整离散制动过程及位置相关速度包络。候选不满足约束时，继续上次已验证的制动过程。关闭整形仍检查必要条件。
- 每个输出再次检查位置、相邻位置步进、位置相关速度、加速度和 jerk。包络突变、IK 或联动约束导致不可行时，报错并停止，不钳位输出位置隐藏不连续性。模型/硬件联动若改变已验证输出则拒绝。
- 运动中切换 joint/cartesian 返回现有 `ConflictTaskRunning`；静止切换清空旧目标、重新校验首点，序号继续递增，故障不被清除。

## 通信质量与停止

`control_command_success_rate` 已从“send_to 成功比例”改为“最近至多 100 个控制周期收到新运动指令的比例”。启动无运动指令时为 0，暖机按已采样周期数计算。查询、重复、乱序、长度错误报文不贡献成功周期；突发多个新包最多贡献当前一个周期。每次实际执行 GenerateRef 只统计一个接收样本；控制节拍延迟单独通过 `control_cycles` 与 `elapsed_periods` 诊断，不把较慢的实际主站节拍直接折算成丢包。名义周期余数使用整数纳秒跨次累计；突发包仍不能补记未来周期成功。

默认保护：连续缺失 20 个周期（1 kHz 时约 20 ms）；最后一个新包的单调时钟年龄及控制线程长间隔也检查相同期限；完整 100 周期窗口的成功率低于 90%，持续 100 次控制更新时停止；启动 1000 ms 未收到运动指令也故障。这是 ROCOS 的可配置策略，不宣称等同 Franka 固件阈值。

通信故障锁存 `LostHeartbeat`（-1008），记录成功率和连续缺失数，进入内部停止阶段；恢复来包不恢复运动。停止阶段继续按 acc/jerk 限制生成减速指令，先下发零速度/加速度样本，再返回故障让 Robot 进入 ERROR_STATE。普通 Stop 同样受限减速，完成后通过 STOPPING 回到 STOPPED。硬必要条件违反直接拒绝本次输出；不可行的减速也立即故障。`Abort()` 用于错误态/销毁释放线程和 socket，和普通 Stop 分开。

UDP 丢包减速在控制线程执行；独立监督线程负责控制周期停滞保护，二者不能替代驱动器独立 watchdog 或证明实体制动效果。UDP 回包也不代表指令已在硬件执行。

## 配置

可在机器人 `robot.yaml` 中添加 `servo`，或在 `user.yaml` 的 `robot.servo` 覆盖：

```yaml
robot:
  servo:
    lowpass_enabled: true
    rate_limit_enabled: true
    cutoff_hz: 100
    communication_guard_enabled: true
    max_missing_cycles: 20
    minimum_success_rate: 0.90
    low_quality_cycles: 100
    first_command_timeout_ms: 1000
```

驱动 `limit` 可添加非负 `deceleration` 和 `velocity_offset`；原有 `acc`、`jerk` 在伺服启动时必须严格为正。非法配置不能启动。有效配置快照包含完整 servo 缺省值。

## 启动、UDP 和诊断

`POST /api/robot/servo/start` 接受可选整数端口（默认 8081，范围 1..65535），需要控制权凭据。UDP 绑定 IPv4 loopback。绑定与非阻塞设置在线程启动前同步检查，失败清理资源并将具体 Result 传回 HTTP，绑定失败为 `ServoBindFail`（-1016）。内部负数 Result 的 HTTP 映射沿用现有实现，在线 StandardResponse 没有逐项定义这些内部码。

- `message_id == 0` 仅查询；成功 Reset 建立新序号域，运动序号必须跨批次严格递增。重复和旧包丢弃且不回包。
- 使用 poll 到包唤醒（无包最多等待 1 ms），消除固定休眠造成的接收延迟；每批最多排空 256 包，回包地址与所选新运动包绑定。旧包、查询、畸形包不覆盖它。拒绝长度错误，包括超长包合法前缀。
- 首条运动指令独立保存，先检查它，再处理最新目标。
- `RobotState.message_id` 是回包计数而非时间戳；矩阵为 row-major；最多 10 轴，使用下述 v2 布局。
- 原预留 `errors`：0=任意伺服故障，1=首点无效，2=通信故障，3=位置/速度/加速度/jerk/轨迹不可行；其余保留。故障后拒绝新目标，发布最后周期快照后，关闭 UDP 前向已绑定发送端主动发送一次终态；UDP 仍是尽力交付。RCI 状态的 `last_control` 保留会话终态，断开后仍可查询。
- C++ `MoveServo::GetDiagnostics()` 提供周期号、收到的序号、成功率、缺失周期、实际控制周期数、累计名义周期数、停止状态、故障、处理后的 q/dq/ddq。这些是生成的参考状态，不标称硬件确认执行状态。

## 与 Franka 的边界

当前 UDP 接口仅有关节位置/笛卡尔位置输入。本次提供两种输入的滤波和关节导数约束，不新增外部速度/力矩接口。关节阻抗力矩率、应用层独立监督和周期快照已接入；仍不宣称等同 Franka 固件的碰撞反射、驱动侧 watchdog 或执行确认机制。笛卡尔位置经 IK 后遵守 URDF 关节限制；单独的 TCP 平移/转动速度、加速度、jerk 数值限制无法由标准关节 URDF 唯一确定，本次没有复制 Franka 的常数。关节限制后的实际笛卡尔路径可能偏离原始目标路径。

## 验证

```bash
cmake --build build --target rocosAppMain move_servo_test model_test servo_robot_test http_control_rights_test app_config_test position_controller_test hardware_test --parallel 1
ctest --test-dir build -R '^(move_servo_test|model_test|servo_robot_test|http_control_rights_test|app_config_test|position_controller_test|hardware_test)$' --output-on-failure
```

测试覆盖 URDF 模型包络、边界/连续关节、无效输入、配置与导出、不同周期下导数限制和停止、通信统计/超时/持续丢包、TCP 偏置、原有启动/序号/首点保护。`servo_robot_test` 使用独立共享内存模拟主站和测试 URDF/PDO，不连接真实 EtherCAT。实体机器人制动、周期抖动和吞吐尚需在目标设备验证。


## 本轮定向修复：协议 v2 与保护边界

在线 Apifox 已于 2026-11-05 刷新。现有 `/api/robot/servo/start` 的在线契约只定义启动和通用 MotionResponse，尚未描述下面的会话字段和 UDP v2 布局；以下是本轮实现扩展，需同步到在线契约，不应视为在线规范已更新。

- 启动成功额外返回 `data.protocol_version=2` 和 `data.session_id`（32 位十六进制字符串）。UDP 两种报文前置 `uint32_t protocol_version`、16 字节 `session_id`；仍采用 `servo_type.hpp` 的紧凑本机布局和 row-major 位姿。旧长度/旧版本报文拒绝，客户端必须同步升级。
- 只有携带当前会话的报文有效；第一条有效运动指令绑定发送端 IP/端口。查询不能绑定发送端。跨会话、其他端口、乱序和重复运动包拒绝。HTTP 释放/抢占、租约过期、服务关闭会撤销会话；UDP 流量不代替 HTTP 心跳。伺服启动不接受调试 token 旁路。
- 状态由 Executor 完成控制周期时发布，UDP 线程只读取快照。首点前 `q_d` 填实际保持位置；`output_valid` 判断本周期输出是否有效。新增 `cycle`、单调采样时刻 `sample_time_ns`、主站时间 `source_time_us`、`received_message_id`、`applied_message_id`、`q_d/dq_d/ddq_d`、`q_cmd/tau_cmd`、`output_mode`、`output_valid`。applied 表示本周期通过校验并成功调用硬件输出接口，不是驱动执行确认；当前共享内存没有驱动逐指令确认。
- `control_command_success_rate` 仍表示最近 100 个控制周期收到新指令的比例，不表示物理执行率；时间余数跨周期累计。硬件采样检查读取前后主站时间是否一致，且要求反馈未过期。当前共享内存没有输入帧 seqlock，不能声称提供跨进程原子 PDO 帧。
- 实际位置/速度、跟随误差和反馈新鲜度故障进入现有错误流程并请求 Quick Stop。`hardware[].limit.following_error` 可显式配置；0 时用模型额定速度乘 20 个控制周期。`torque_rate` 为关节阻抗输出变化率上限，默认 200 Nm/s（移动关节对应 N/s），必须按机器人核定。
- 控制周期等待使用单调时钟和有界超时；独立监督线程在伺服控制停滞时锁存 CiA402 Quick Stop 请求。复位并确认关闭使能后才能解除锁存。主站完全失效时，应用无法确保 PDO 到达驱动，必须配置并验证主站/驱动的通信看门狗。
- 常规与联动 IK 单次求解预算均为 1 ms（2026-11-06 从 0.3 ms 调整）。App 在 Model 初始化、限位修改和求解器重建时显式传入预算，不依赖 pin_ik 子模块默认值；这不是整个笛卡尔周期已经满足 1 ms 的证明。

## RCI 开关与无额外凭据接入

以下为新增实现，**尚未同步到在线 Apifox ROCOS API**。在线规范仍为唯一正式契约；本节不是已发布 API 的替代定义。当前 MCP 仅能读取规范，不能发布此更新。

RCI（ROCOS Control Interface）开关独立于运动状态：开启后仍为 `STOPPED`，保持位置，禁止普通运动入口；程序成功接入后才进入 `SERVOING`。RCI 首个接入程序独占控制，不验证该程序身份。现阶段 UDP 仅监听 `127.0.0.1`，因此 RCI 接入 HTTP 请求也限定 loopback；上位机可以远程开启和监控，本机控制程序随后接入。

| 新增入口 | 授权和行为 |
| --- | --- |
| `POST /api/robot/rci/enable` | 必须有有效 HTTP `ControlToken + ClientId`，禁止调试旁路。仅机器人 `STOPPED` 且无运行/暂停脚本时开启；重复开启幂等。 |
| `POST /api/robot/rci/start` | 无需 HTTP token 或手工凭据。仅 RCI 已开启、未关闭中、无现存会话且机器人 `STOPPED` 时成功。请求可含 `port`（默认 8081）及 `mode`（`joint` / `cartesian`，默认 `joint`）；返回自动生成的 `session_id`、`protocol_version`、`port`。 |
| `POST /api/robot/rci/stop` | 无需 token，只能停止 RCI。请求受控停止，完成后撤销 UDP 会话并保留 RCI 开关；重复停止幂等。 |
| `POST /api/robot/rci/disable` | 必须有有效 HTTP 控制权，禁止调试旁路。先停止、再撤销会话和关闭 RCI；`closing=true` 期间继续禁止其他运动。返回成功表示请求已接受，完成状态应查询 status。 |
| `GET /api/robot/rci/status` | 公开只读，返回 `enabled`、`closing`、`control_active`、`robot_state`；2026-11-06 实现新增活动会话 `current_control` 和保留终态 `last_control`（首次结束前为 null），含周期时间、指令序号、故障、实际/期望/最终输出及统计计数，不泄露会话 ID。`/api/robot/state` 同时增加 `rci` 字段。 |

所有响应沿用现有 JSON 包装 `success/code/message/data`。未开启时 start/stop 返回 `ServoNotInitialized`；重复接入、非停止状态、脚本占用及普通运动冲突返回 `ConflictTaskRunning`；非法 HTTP port/mode 返回 `1001`；启动资源失败保留原始 Result。状态示例：`enabled=true, closing=false, control_active=false, robot_state=STOPPED` 表示等待接入。

Robot 公共运动入口（MoveJ/L/C、点动、旧 MoveServoing、模式/负载/TCP 修改以及旧单轴输出函数）执行独占检查，不仅依赖网页按钮或 HTTP 检查。Lua Run/Step 与 RCI 开启共用命令门闩，已有运行或暂停脚本必须先结束。直接持有公开 `hardware`/`controller` 指针的进程内 C++ 代码仍属于可信实现，RCI 不是进程内安全沙箱。

RCI 期间 `/api/control/takeover` 返回冲突，而在线规范目前仍写“无条件覆盖”；这是待同步的明确契约变更。常规 HTTP 写入口拒绝修改运动/配置，保留监控、停止和故障恢复。旧 `/servo/start` 在 RCI 关闭时保留原先 HTTP 租约语义，在 RCI 开启时拒绝。`/rci/start` 创建的会话与开启者 HTTP 心跳独立，释放或过期不终止 RCI 程序。程序断连由原有首包超时/通信看门狗检测，故障停止后撤销会话，RCI 保持开启；必须恢复故障、回到 `STOPPED` 才能重新接入，绝不自动恢复旧指令。

会话 ID 由客户端自动保存并放入每个 UDP 包，仅用于旧包隔离；首个合法运动包绑定发送端，序号必须持续递增。它不是用户需要管理的接入凭据。开启 RCI 意味着允许任一本机程序竞争首次接入；同时只允许一个控制会话。

使用路径：

- 上位机取得普通 HTTP 控制权并调用 `rci/enable`；程序直接调用 `rci/start`，自动处理会话 ID 并发送 UDP。
- 程序自行调用 `control/acquire → rci/enable → control/release → rci/start`，随后通过 UDP 控制；普通控制权已被占用时应由上位机开启，不自动抢占。
- `examples/udp_example.cpp` 已实现两种路径，不再要求手工设置 token 或 session ID；程序退出调用 `rci/stop`，保留 RCI 开关。示例会发送运动指令，只应在已准备好的环境中运行。

## 2026-11-06 启停线程修复

`IsControlActive()` 在一次 FSM 锁范围内判断所有有效状态，避免多次读取跨状态转换得到错误的 false。控制线程退出不再关闭独立监督线程；伺服监督已武装而控制线程意外结束时，请求安全停止并进入错误态。监督线程由 Robot 的线程回收路径结束。正常应用退出下使能路径未修改。
