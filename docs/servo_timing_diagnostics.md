# UDP 伺服时序诊断

本诊断仅记录时序，不改变控制算法、滤波/限速、J=10000、通信保护、会话鉴权或现有线程优先级，也不改变 UDP/HTTP 协议。默认关闭，不随普通启动自动采集。

## 启动和采集

远端编译目录为 `/root/rocos-app/build/release`。请由操作者正常关闭旧 App 后启动新二进制；不需要为诊断重启主站。

```bash
sudo env ROCOS_SERVO_TIMING_TRACE=1 \
  /root/rocos-app/build/release/bin/rocosAppMain \
  --config=/root/rocos-app/config/app.yaml
```

环境变量必须在启动 App 时设置。每次启动伺服会话时才创建该会话的记录文件，日志打印一次：

```text
Servo timing capture: /dev/shm/rocos-servo-timing/servo_<pid>_<session_id>.bin (65536 records per stream)
```

只测试通信时，仍使用 `librocos_communication_test` 保持初始收到的关节参考，不发送定位或正弦轨迹。启动 App 不等于同意执行机器人运动，测试前仍需确认现场静止、模式正确、已准备使能及控制权状态。

每个会话约占用 **9 MiB** tmpfs 和锁定内存，两条环形缓冲区各 65536 条。在 1 kHz 且每周期一个包的情况下分别保留最近约 65.5 秒，超出时覆盖旧记录，不阻塞控制线程。无效会话、来源不匹配、长度/协议错误和 id=0 查询包不进入本采集；有效会话内的重复、乱序正序号包有记录。

预分配、预触页和 mlock 在线程启动前完成。如果 tmpfs 容量不足或 mlock 权限/额度不足，记录一次 `Servo timing capture disabled: ...`，该次诊断关闭，原有运动启动与安全检查按原逻辑执行。此时应先处理诊断失败再做时序实验；不要认为已有有效记录。

采集期间只读取时钟、复制固定长度记录、更新单生产者索引。不创建写文件线程，不逐周期分配或格式化字符串，不在实时线程调用 CSV 输出、文件 write、flush 或 msync。环形数据写入锁页的共享内存，停止或进程退出后文件仍留在 tmpfs；重启操作系统会丢失。

## 导出

**等伺服会话结束后导出**，不必关闭 App。使用日志中该会话的完整文件名：

```bash
sudo ls -lt /dev/shm/rocos-servo-timing/

sudo python3 /root/rocos-app/examples/servo_timing_dump.py \
  /dev/shm/rocos-servo-timing/servo_<pid>_<session_id>.bin \
  /root/rocos-app/build/servo-timing-export-001
```

输出目录必须尚不存在。导出器生成 `udp.csv`、`control.csv`、`metadata.json`，记录被覆盖的数量。它检查导出过程中写计数是否变化，但这不能替代操作者确认会话已停止。活动会话在写入环形槽位时不保证跨线程快照一致，不用于在线闭环。

文件为 Linux 本机小端 v1 诊断格式，权限 0600。它独立于 UDP 协议，不向客户端增加状态字段。测试文件会留存，确认导出后可以删除对应已结束会话的文件释放空间；不要删除仍在写入的会话。

## 字段与时点

全部 `*_ns` 使用**对方 App 所在主机的 CLOCK_MONOTONIC，单位 ns**。零表示该阶段没有执行。不能直接与另一台电脑的 MONOTONIC 比较。`metadata.json` 的创建时 MONOTONIC/REALTIME 是近似配对，仅用于粗关联；精确关联两端抓包还应同步采集同机时钟映射。

`udp.csv`：

| 字段 | 定义 |
| --- | --- |
| message_id | 接收的正序号指令 |
| receive_ns | 非阻塞 recv_from 返回后立即取时，先于协议检查及指令锁等待；不是网卡硬件接收时间 |
| locked_ns | UDP 线程取得指令互斥锁后 |
| ready_ns | 锁内更新 cmd、标志和接收时间完成后，解锁前的发布就绪时刻 |
| published_ns | 锁已释放后立即取时，标记发布完成 |
| disposition | 1 已发布；2 停止/故障中拒绝更新；3 乱序或重复拒绝 |

`ready_ns` 与 `published_ns` 界定解锁边界。控制线程可能在解锁后、UDP 线程记录 `published_ns` 之前已取用指令，因此少量 `consumed_ns < published_ns` 并不证明记录乱序，应同时检查 `ready_ns`。

`control.csv`：

| 字段 | 定义 |
| --- | --- |
| cycle | 原有伺服周期序号 |
| entry_ns | GenerateRef 进入、等待互斥锁前 |
| locked_ns | 控制线程取得互斥锁后 |
| consumed_ns | 实际读取最新 cmd 序号、统计本周期是否有新指令之前 |
| consumed_id | 上述统计使用的 cmd 序号 |
| reference_id | 本周期传入参考生成的指令序号；首次使用保留的 first_cmd，可能早于 consumed_id；保持/停止等未生成新候选时为 0 |
| applied_id | CompleteCycle 按原逻辑保存的实际下发参考序号；失败时可能保持旧值，不等于驱动执行确认 |
| completed_ns | 本周期状态发布完成后、提交诊断记录前 |
| result | 原有 CompleteCycle 的 Result；负值表示失败 |

控制记录在现有 CompleteCycle 中提交，不额外插入控制调用。异常导致未进入 CompleteCycle 的周期没有完整记录，不能把缺失行当成 UDP 丢包。

## 定位方法

对同一会话同一 message_id 关联 UDP 与控制记录：

- `locked_ns - receive_ns`：UDP 用户态接收返回后到获取指令锁，包含校验与锁等待，不单独等同内核到用户态调度延迟。
- `ready_ns - locked_ns`：锁内发布处理。
- `published_ns - ready_ns`：解锁及随后调度/取时区间。
- 控制 `locked_ns - entry_ns`：控制线程获取指令锁的等待。
- `consumed_ns - ready_ns`：发布就绪到被控制线程取用的间隔；按 consumed_id 关联，重复取用应保留每个周期。
- `completed_ns - consumed_ns`：取用到状态发布，包含原有参考处理及执行器过程，不是 IK 单独耗时。
- `consumed_id` 重复、下一周期跳号：结合上一条和下一条的 ready/published 时刻，区分跨周期与未发布；某条报文已发布但被更新指令覆盖时可能从未被取用。

若增加对方入站抓包，使用同机 REALTIME/MONOTONIC 映射后，才能估计内核抓包到 `receive_ns` 的延迟。单靠本采集不能定位 IRQ/NAPI 之前或 USB 网卡内部的延迟。

诊断必然增加少量时钟读取和内存写入，可能轻微影响边界竞争；后续需要与关闭采集的同工况比较，不能宣称采集绝对无扰动。
