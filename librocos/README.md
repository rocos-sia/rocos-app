# librocos

C++17 关节位置伺服客户端，采用 libfranka 风格的 `Robot::control(callback)`、`RobotState`、`Duration`、`JointPositions` 和 `MotionFinished`。客户端命名空间为 `librocos`，与控制器的 `rocos::Robot` 区分。

## 构建和接入

在 ROCOS-App 仓库根目录执行单核构建。独立构建只依赖编译器、Threads 和仓库中的 HTTP/JSON/UDP 协议头。

```bash
cmake -S librocos -B build/librocos-standalone -DCMAKE_BUILD_TYPE=Release
cmake --build build/librocos-standalone --parallel 1
ctest --test-dir build/librocos-standalone --output-on-failure
cmake --install build/librocos-standalone --prefix /tmp/librocos-install
```

外部 CMake 项目使用安装目录作为 `CMAKE_PREFIX_PATH`。

```cmake
find_package(librocos CONFIG REQUIRED)
target_link_libraries(my_program PRIVATE librocos::librocos)
```

```cpp
#include <librocos/robot.h>

librocos::Robot robot("127.0.0.1");
robot.enableRci();
double elapsed = 0;
librocos::JointArray initial{};
robot.control([&](const librocos::RobotState& state, librocos::Duration period) {
    if (period.toNSec() == 0) initial = state.q_d;
    elapsed += period.toSec();
    librocos::JointPositions command(initial, state.dof);
    if (elapsed >= 2.0) return librocos::MotionFinished(command);
    return command;
});
robot.disableRci();
```

运行运动程序前，准备好控制器及硬件或仿真环境，将机器人使能至 `STOPPED`，工作模式设为 `position`。`enableRci()` 在入口已开启时直接返回，支持上位机先开启 RCI。程序开启入口时，库自动申请 HTTP 控制权、开启 RCI 并释放 HTTP 租约，业务程序无需管理令牌。其他持有者占用 HTTP 控制权时，错误会传回调用者。

`control()` 每次建立一个独占 RCI 会话，校验协议版本、会话 ID、UDP 端点及报文长度。现有 RCI UDP 绑定 `127.0.0.1`，运动程序需与 App 在同一主机运行。HTTP 状态读取可使用可访问的控制器地址。

## 周期与状态语义

- `readOnce()` 返回一次 HTTP 实际状态，`read(callback)` 默认每 20 ms 轮询。`reference_valid=false`，`q/dq/tau` 取自同一次 HTTP 状态响应。关节数量使用 `hw_state.joint_num` 并与 `joint_states` 核对。
- `control(callback)` 的状态来自 UDP 周期快照，`reference_valid=true`。`q/dq/tau` 为实际反馈，`q_d/dq_d/ddq_d` 为处理后的参考，`q_cmd/tau_cmd` 为最终控制输出字段。
- UDP v2 的 `enabled/robot_state/work_mode` 依据有效位置伺服会话填充。完整机器人状态通过 `readOnce()` 查询；`control_cycles/elapsed_periods` 和数值故障码通过 `rciStatus()` 获取，UDP 回调中的这些字段保持零。
- 回调首帧 `period=0`，随后使用相邻新采样的 `sample_time_ns` 差值。重复周期只保持上一命令，回调仅处理新采样。
- `Options::control_period` 是发送周期，默认 1 ms，调用者应匹配硬件配置。当前 `/robot/info` 的周期字段可能为 null。发送采用绝对时钟调度，超期后跳过补发。
- `control_command_success_rate` 表示新运动指令的接收比例。`output_valid` 和 `applied_message_id` 表示硬件接口提交结果，实际运动应结合反馈判断。
- 低通、限速、首点容差、关节边界、跟随误差和可停止性由现有 App 执行。客户端校验命令维度和有限值，周期内复用固定容量数组，最多支持 10 个关节。

## 结束、故障与并发

`MotionFinished(command)` 先发送最终命令并等待其应用序号得到确认，然后请求受控停止，等待会话结束后返回。轨迹末端应平滑降速并预留保持时间，最终命令的提交确认仅说明该目标已进入控制链。示例在轨迹结束后保持 250 ms。

回调抛异常、反馈超时、陈旧反馈、协议错误和控制器故障均触发会话清理。控制器故障通过 `ControlException::code()`、`status` 保留，清理失败时 `cause` 保留原异常。停止未确认时，该对象拒绝再次启动会话，可调用 `stop()` 重试。失败的会话接入请求不会取得会话清理责任。

故障诊断中的关节数组可能包含由非有限反馈序列化得到的 JSON null，库将其保留为 NaN，同时提供终态和故障码。

`stop()` 可从其他线程请求结束当前 `read/control`。回调应及时返回，阻塞的回调由 App 通信看门狗保护。`control()` 与 RCI 开关、持续 `read()` 在同一个对象上互斥，`readOnce()` 和 `rciStatus()` 可独立查询。对象销毁前应先结束调用它的线程。

回调内避免 HTTP 请求、打印、文件写入和阻塞操作。默认反馈新鲜度上限为 20 ms，库会检测回调超过该上限的情况。实际实时性取决于调用线程、操作系统调度和控制器配置。

## 示例

可执行文件位于 `build/librocos-standalone/bin`，运动示例启动前显示参数并等待 Enter。

| 程序 | 用途 |
| --- | --- |
| `librocos_echo_robot_state` | HTTP 状态打印，Ctrl-C 结束 |
| `librocos_generate_joint_position_motion` | 一个关节的平滑余弦往返，端点速度、加速度和 jerk 为零 |
| `librocos_joint_point_to_point_motion` | 指定关节相对当前位置的七次时间标定定位 |
| `librocos_generate_consecutive_motions` | 四次独立 RCI 会话的往返运动 |
| `librocos_communication_test` | 保持当前参考，统计回调周期和指令接收比例 |

```text
program [host [http_port [udp_port [joint_index [amplitude_rad [duration_s]]]]]]
```

默认地址 `127.0.0.1`，HTTP 8080，UDP 8081，关节索引 0，幅度 0.01 rad，轨迹持续 4 s。幅度与时间必须符合当前机器人限制。运动结束后 RCI 入口保持开启，可以继续启动新会话，关闭入口使用 `disableRci()`。

## 测试

`librocos_test` 使用独立的 HTTP/UDP 协议对端，覆盖会话、回调、报文、新鲜度、异常及停止流程。主仓库构建中的 `librocos_robot_test` 使用真实 `RobotHttpServer + MoveServo` 和独立共享内存离线主站，覆盖往返位置轨迹、反复会话及非法首点，测试 jerk 为 10000。

```bash
cmake -S . -B build
cmake --build build --target servo_robot_test --parallel 1
ctest --test-dir build -R '^librocos_robot_test$' --output-on-failure
```
