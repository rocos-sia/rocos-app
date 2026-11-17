# 独立双臂 App

`rocosDualArmMain` 在一个进程中管理两个独立 `Robot`，链接现有 `rocos::robot`。
`rocosAppMain`、单臂配置解析器和单臂 HTTP 服务保持原样。默认不构建双臂目标。

## 构建与配置检查

从 rocos-app 根目录执行：

```bash
cmake -S . -B build/dualarm -DCMAKE_BUILD_TYPE=Release -DROCOS_BUILD_DUALARM=ON
cmake --build build/dualarm --target rocosDualArmMain dualarm_unit_test --parallel 2
./build/dualarm/bin/rocosDualArmMain --config dualarm/config/app.yaml --check_config
ctest --test-dir build/dualarm -R '^dualarm_' --output-on-failure
```

依赖搜索路径沿用本机已有 CMake 设置；例如本地安装的 Pinocchio 可以通过
`-DCMAKE_PREFIX_PATH="$PWD/build/deps/prefix"` 指定。依赖仍由原有核心库管理。

入口配置独立采用 `schema_version: 1`，只接受以下字段：

- `arms.left.config` / `arms.right.config`：现有 schema_version=1 单臂 App 配置文件。
- `http_host` / `http_port`：默认 `127.0.0.1:8080`。
- `http_control_ttl_seconds`：每臂控制权租约，默认 300 秒，必须为正数。
- `workcell.urdf` / `workcell.asset_root`：可选完整双臂模型及资源根目录。
  当前要求两臂均引用这份完整 URDF，通过各自 base/tip 选择运动链；不配置时仍能独立控制，但模型下载不可用。
- `diagnostics.host` / `diagnostics.port`：默认 `127.0.0.1:8082`。
- `logging.directory` / `logging.file_level` / `logging.console_level`。

路径相对于声明该路径的配置文件解析。子配置只用于 Robot 和设备资源，实际 HTTP、诊断和日志服务由顶层启动一次。
`--check_config` 校验模型、关节绑定及配置隔离，不连接主站、不创建 Robot、不写设备快照。
启动时先检查两个主站及周期，再构造两个 Robot；双方必须进入下使能的 IDLE 后才开放 HTTP。

示例要求同级目录中存在 `rocos-mujoco`，引用其双臂 URDF、硬件和主站配置。
左右设备文件位于独立目录，工具库位于 `build/dualarm/tools/{left,right}`，设备 ID 为
`dualarm_app_left` / `dualarm_app_right`。不要与其他控制程序同时控制主站 0/1。

## 运行

先在 MuJoCo 仓库启动双臂仿真，再在本仓库运行：

```bash
./build/dualarm/bin/rocosDualArmMain --config dualarm/config/app.yaml
```

HTTP 接口：

| 操作 | 路径 |
| --- | --- |
| 枚举两臂 | `GET /api/arms` |
| 状态、信息、运动状态 | `GET /api/arms/{arm}/robot/{state,info,move_status}` |
| 获取、心跳、释放控制权 | `POST /api/arms/{arm}/control/{acquire,heartbeat,release}` |
| 控制权状态 | `GET /api/arms/{arm}/control/status` |
| 使能、下使能、复位 | `POST /api/arms/{arm}/robot/{enable,disable,reset}` |
| MoveJ、暂停、恢复、停止 | `POST /api/arms/{arm}/robot/{movej,pause,resume,stop}` |
| 精确运动 | `POST /api/arms/{arm}/robot/{movej_ik,movel,movel_fk}` |
| 关节、笛卡尔、零空间、SVD 点动 | `POST /api/arms/{arm}/robot/jog/{joint,cartesian,nullspace,svd}` |
| 停止点动 | `POST /api/arms/{arm}/robot/jog/stop` |
| TCP、使能查询 | `GET /api/arms/{arm}/robot/{tcp,enabled}` |
| 坐标系列表 | `GET /api/arms/{arm}/robot/{tool_frames,object_frames}` |
| 坐标系读取、设置、删除 | `GET/POST/DELETE /api/arms/{arm}/robot/{tool_frame,object_frame}` |
| 激活坐标系 | `POST /api/arms/{arm}/robot/{active_tool_frame,active_object_frame}` |
| 坐标系持久化 | `POST /api/arms/{arm}/robot/frames/{load,save}` |
| 负载参数 | `GET/POST /api/arms/{arm}/robot/load_params`、`POST …/load_params/save` |
| 六点工具标定 | `POST /api/arms/{arm}/calibration/{pose,run,save}`、`GET …/calibration/result?session_id=…` |
| 工作站模型清单 | `GET /api/workcell/model` |
| 完整模型与资源 | `GET /api/workcell/urdf`、`GET /api/workcell/urdf/mesh?path=…` |

`GET /api/arms` 增加 capabilities、model_url 和运动链信息。只声明本适配层已开放的能力。
按臂 `robot/urdf`、`robot/urdf/mesh` 是共用模型下载的兼容入口，返回同一完整模型，
不能将其各加载一次再叠放。优先使用 workcell 入口下载一次，按清单 joint_names 更新两组关节。
root_link 定义显示模型根；每臂 base_link/tip_link 及它们在 URDF 中的变换定义运动坐标，
相机方向不改变左右命名。模型版本为内容校验和，URDF/资源支持 ETag / If-None-Match。
模型及资源在启动时读取为不可变快照；更新磁盘模型后需重启控制器。资源仅允许下载清单中的原始引用，
兼容 `../meshes` 与 `package://包名/路径`，解析后的文件（包括符号链接目标）必须位于 asset_root。
不按文件名递归搜索，避免同名资源串用；单资源最多 64 MiB，总包最多 256 MiB。

点动沿用单臂方向向量、speed、timeout、dir_threshold 语义；笛卡尔 frame 支持
BASE/FLANGE/TOOL/OBJECT，参考所选臂，WORLD 不能直接作为运动 frame。
7 轴 SVD 点动 dim_speeds 长度为 1。持续按住时串行续发，松开后停止续发并调用本臂 jog/stop；
丢失续发由 Robot 的 timeout 结束运动，不依赖控制权 TTL。客户端仍须处理失焦、切臂和迟到请求；
本版未引入点动会话 ID，不能任意并发乱序发送同一臂的点动与停止。

坐标系/负载写操作要求本臂 IDLE 或 STOPPED，否则 code=4005。工具支持 persist，工件坐标系
显式 frames/save 后持久化。所有保存/加载仅接受本臂默认配置路径（或与其相同的显式路径），
其他路径 code=1001，避免跨臂读写。不同于单臂在线协议的自由路径与工具重命名能力，
本适配层暂不支持 previous_name 重命名、工具包导入/通用编辑；tool_frames 返回 package_support=false，
表示工具包管理未开放。已配置工具包可六点标定；tool_frame 返回其 package 安装与标定信息。
负载保存对非法 JSON/path 严格返回 1001，不采用单臂旧接口的静默默认路径回退。
位姿要求 position 和 orientation 对象以及完整 xyz/xyzw 数字，拒绝零四元数并归一化非零四元数。

`arm` 必须为 `left` 或 `right`。请求体、单位、业务结果沿用在线 ROCOS API；
例如 MoveJ 的 `joints` 为本臂 7 个关节角度，单位 rad。成功表示受理，完成需查询状态及反馈。
每臂分别 acquire，取得 token/client_id 后使用 `X-Rocos-Control-Token` 和
`X-Rocos-Client-Id` 请求头。左右 token 不可混用；没有 debug-token 旁路。
同一臂的鉴权、租约更新与写命令在同一个臂级锁内执行，另一个臂可独立接收请求。
GET 不续期；release 不等价于运动停止。超时的 MoveJ POST 不应盲目重试。

MuJoCo 下使能期间关节可能漂移。重新使能后须等状态进入 `STOPPED`、反馈速度持续稳定再发
MoveJ；使能成功响应不表示物理反馈已稳定。联调曾在立即重发时触发核心的 `SpeedLimit`
保护，当前双臂入口保留单臂核心的使能和轨迹衔接语义。

按臂请求记录到 `DualArm.left` / `DualArm.right`；现有核心库日志仍使用原模块名称，
没有为双臂改动其日志格式。设备有效配置快照继续按设备 ID 保存到
`~/.local/state/rocos/config_snapshots/`。

SIGINT/SIGTERM 停止接单后分别停止两臂运动、下使能并回收线程；遇到停止超时请求
硬件安全停止并返回非零退出码。必须让仿真主站保持运行直至 App 退出。
单臂命令失败不操作另一臂；同进程仍共享进程故障边界。

本版本提供独立运动，不承诺两臂同步启动、同步完成或同周期快照，也不提供跨臂避碰规划。
未开放脚本、工具包导入/导出等管理、RCI/UDP、DDS 和双臂批量动作；状态保留单臂既有字段。

## 六点工具标定

每臂独立维护最多 16 个会话，复用核心 `CaptureCalibrationPose` 与 `solveSixPointTool`。
支持普通工具和已配置工具包的法兰到 TCP 标定。
`GET /api/arms` 中 `capabilities.six_point_calibration=true`、`tool_package_calibration=true` 表示这两项能力。

1. 获取目标臂控制权，将本臂停止并等待实际关节速度稳定。写操作仅允许 IDLE/STOPPED；
   采集额外由核心检查实际关节速度不超过 0.001 rad/s。另一臂可以独立运动。
2. `POST …/calibration/pose`，请求 `{"name":"probe","index":1}` 创建会话；
   后续点及重采点携带返回的 `session_id`、相同名称和 1–6 的 index。
   P1–P4 保持同一 TCP 固定点、改变法兰姿态；保持 P4 方向，沿期望工具 +X/+Z
   移动后分别采集 P5/P6。接口读取真实反馈和 FK，不接受客户端 pose/flange/joints。
3. `POST …/calibration/run`，请求 `{"session_id":"…","max_error":0.001}`。
   max_error 单位 m，范围 (0, 0.05]；返回法兰到 TCP 的 pose、RMS/最大残差、条件数和轴夹角。
   求解复用单臂算法：退化姿态、超限残差、方向位移不足 1 mm、方向夹角过小或 P5/P6
   姿态偏差超过 1° 均失败。计算不保存、不激活工具，也不执行运动。
4. `GET …/calibration/result?session_id=…` 可无控制权回读六个样本槽位及结果，不续期租约。
5. `POST …/calibration/save` 只接受 `{"session_id":"…"}`，使用服务端结果原子保存到本臂
   默认 frames 文件。需要激活时再显式调用本臂 `robot/active_tool_frame`。

会话同时受臂和创建者 token 隔离。更换控制权后旧会话仍可只读查询，但无法继续采集、计算或保存；
须重新从 P1 开始。重启、超过 16 个会话淘汰最早会话、或本臂成功 frames/load 后失效。
每次成功重采立即清除旧结果，求解失败不保留旧解，保存失败不改变活动工具。
4001 表示缺样本/缺结果，4002 表示求解失败，4004 表示会话不存在或不属于当前控制者，
4005 表示状态/静止反馈不允许；鉴权沿用 3006/3007。
会话名称匹配已配置工具包时，P1 固定 `package_digest` 和 `package_revision`；
后续采点、计算和保存均校验安装上下文。包更新、移除或安装 revision 改变返回 4004 并清除旧解，
必须重新采集。普通工具保持空摘要和 revision=0；普通工具变为包工具时旧会话也失效。

工具包保存按 `local_tcp = mount.Inverse() * result.pose` 换算，保留 mount、包摘要、版本与原始资源。
只更新本臂 frames 文件中的本机局部 TCP、安装 revision 和标定记录（时间、robot_id、包摘要、
mount、法兰到 TCP、残差、条件数、轴夹角）；不改原包，不自动激活。
文件原子提交成功后才发布新注册表快照；写文件失败保留原 TCP/revision 和可重试的计算结果。
成功保存后会话跟随新 revision，可明确重复保存；其他会话仍绑定旧 revision，不能覆盖新结果。
可用 `GET …/robot/tool_frame?name=…` 回读 package.tcp、mount、revision、calibration_yaml，
并用显式 `frames/load` 恢复已保存安装。启动仍沿用现有设备配置选择规则，不新增启动覆盖优先级。
SDK 仅增加只读 `Robot::GetToolPackage`、带状态门闩的 `Robot::SaveToolCalibration`，以及注册表原子保存方法；
原单臂 HTTP 路由和行为保持原样。
与上游单臂接口相比，本地额外拒绝客户端样本字段/保存扩展字段，并在 frames/load 后清空会话。

## 实际 MuJoCo 联调

停止已有主站 0/1、8080/8082 服务，再从本仓库执行：

```bash
python3 dualarm/tests/integration_mujoco.py
```

完整工具包标定联调（额外依赖 Python NumPy，仿真最长 120 秒）：

```bash
python3 dualarm/tests/integration_mujoco.py --tool-packages \
  --output build/dualarm/integration-package-calibration
```

此模式在测试输出目录创建左右独立工具包和设备配置，移动两臂分别取得六个有效标定位姿，
验证 TCP 换算、保存记录、重复保存、重新加载和跨臂隔离，再运行原有双臂运动检查。
仅修改测试输出目录，不覆盖源工具包或设备配置。

脚本自行启动一个有限时长的 MuJoCo 和一个双臂 App，测试两个 Robot、左右单独运动、
并发运动、左臂停止而右臂继续、独立下使能、错误 token/臂名/参数隔离及运动中退出。
结果保存在 `build/dualarm/integration/report.json`，日志在同目录。
此测试会使仿真运动，需显式执行，不加入默认 CTest。
配置和租约离线测试无需仿真，也不依赖同级 MuJoCo 仓库。

## 基础入口验证记录（扩展前，2026-11-17）

- 默认关闭双臂选项时不生成双臂目标；单臂和双臂入口均构建成功。
- 离线配置 17 个用例、租约 2 个用例 / 23 条断言通过；缺少主站时启动快速失败。
- MuJoCo 联调通过，最大到位误差约 0.00042 rad；运动中退出后 14 个驱动均下使能，IPC 清理完成。
- 单臂回归中 hardware、position_controller、move_joint、servo_robot、http_control_rights、tooling_http 六个套件通过。
- app_config 的 19/20 个用例通过，RCM 用例缺少 `config/robots/rcm/robot.yaml`；
  robot_fsm 和 lua_script_engine 缺少 `hardware_talon_config.yaml`。
  三个套件的失败均在原有 `build/bin/` 产物复现，未为本次双臂工作修改原测试或单臂核心。

完整结果见 `build/dualarm/verification/summary.json`，仿真报告见
`build/dualarm/integration/report.json`。初次立即重发 MoveJ 触发速度保护的记录保存在
`build/dualarm/integration-enable-transient/`，最终联调采用上文的反馈稳定条件。
独立协议文件位于同级 MuJoCo 仓库根目录 `dualarm-openapi.yaml`，已完成 OpenAPI 校验，尚未发布到在线 Apifox。

## 控制器扩展验证（2026-11-17）

- 独立双臂目标构建成功；4 个离线用例 / 39 条断言、19 个配置用例通过。
- 下载完整 URDF 和 15 个 mesh，逐文件比对源资源；ETag、未引用路径和越界资源拒绝均通过。
- 左右分别验证关节、笛卡尔、零空间、SVD 点动，观察真实仿真关节变化及停发超时停止。
- 左右分别验证 MoveJ_IK、MoveL、MoveL_FK 的不同目标运动；另一臂保持原状态。
- 两臂同时独立点动，停止左臂后右臂仍运行；错臂 token 不能停止另一臂。
- TCP、坐标系增删改查/激活/保存/加载、负载读写及跨臂文件拒绝通过，全部写入测试目录。
- 原有双臂 MoveJ、暂停/恢复、独立使能/释放与退出下使能测试通过；主站 0/1 IPC 清理完成。
- hardware、position_controller、move_joint、servo_robot、http_control_rights、tooling_http、
  frame_registry、tool_package 八个单臂回归套件通过。servo_robot 初次与编译并行时超时，
  单独重跑 7.49 秒通过；两个未构建的工具测试目标构建后通过。
- 协议为 `0.2.0-local`，40 条路径 / 45 个操作，通过 OpenAPI 校验；未发布线上。

最终仿真报告：`build/dualarm/integration-extended-final/report.json`。
验证汇总：`build/dualarm/verification/controller-extension.json`。
过程中修正了集成脚本异常变量遮蔽 `error()` 导致的测试退出；该失败发生在测试逻辑，
随后完整重跑通过。上述验证针对 MuJoCo，不代表真机验收。

## 六点标定验证（2026-11-17）

- 新增 `dualarm_calibration_test`：3 个用例 / 42 条断言通过。使用已知非平凡 TCP 的六点数据，
  验证求解精度、真实注册表文件保存/重新加载、不自动激活、另一臂工具不变；验证重采/失败求解
  清空结果、旧 owner 拒绝、16 会话淘汰及保存失败传播。
- 原双臂单元与配置测试通过；单臂 `tool_calibration_test`、`frame_registry_test`、
  `tooling_http_test` 回归通过。`src/` 单臂核心与服务未修改。
- MuJoCo 完整联调 28 项检查通过，包括左右并发真实反馈采样、跨臂会话和令牌拒绝、
  更换控制权后旧会话不可写、重复姿态求解失败且不能保存、frames/load 后会话失效，
  以及运动中 pose/run/save 返回 4005。原有独立运动/点动/退出检查继续通过，IPC 清理完成。
  成功六点求解和落盘使用离线几何测试；此次 MuJoCo 标定测试覆盖真实采样和失败/隔离路径，
  未完成仿真中移动六个有效标定位姿的完整成功链路。
- 本地独立 OpenAPI 为 `0.3.0-local`，44 条路径 / 49 个操作，规范与引用校验通过；未发布线上。

仿真报告：`build/dualarm/integration-calibration/report.json`。

## 工具包标定补齐验证（2026-11-17）

- 支持已配置工具包的六点标定上下文、局部 TCP 换算、原子保存与标定记录回读。
  新增 SDK 接口为小范围增量，原单臂 HTTP 服务未修改；单臂/双臂入口均构建成功。
- 标定离线测试 6 个用例 / 91 条断言通过，包括普通工具、非平凡安装变换、包摘要/revision
  冲突、移除工具、重复保存、保存后重新加载、原包/另一臂不变，以及文件失败回滚与重试。
- 双臂单元/配置测试，以及单臂 tool_calibration、frame_registry、tool_package、tooling_http 回归通过。
- MuJoCo 完整联调 30 项检查通过。两臂各移动到六个有效姿态，从实际反馈采样后求解、保存、
  回读本机局部 TCP 与标定记录，再显式重新加载；完成了上节尚未验证的完整成功链路。
  本次仿真 TCP 平移误差分别为左约 0.025 mm、右约 0.036 mm，不代表真机标定精度。
  另一臂和原始工具包不变，退出时 14 个驱动下使能，IPC 清理无错误。
- 首次扩展仿真已通过全部功能检查，但测试脚本仍按旧 70 秒等待清理，提前杀掉延长时长的
  仿真。已修正为基于仿真启动时刻和配置时长的截止时间，清理该次测试自有 IPC 并完整重跑通过。
- 独立 OpenAPI `0.4.0-local`：44 条路径 / 49 个操作，规范及 193 个本地引用校验通过；未发布线上。

最终报告：`build/dualarm/integration-package-calibration-final/report.json`。

### 双臂日志反馈（本地协议）

`rocosDualArmMain` 启动时启用独立 INFO+ 日志缓存，沿用 `diagnostics.host/port`
（默认 8082）。`GET /api/dualarm/logs?limit=100` 返回近期事件与 `next_id`，随后
`GET /api/dualarm/logs/stream?after=...` 订阅一个覆盖两臂及进程的 SSE 流。
不需要控制凭据，不延长控制权。`Last-Event-ID` 优先于 `after`；客户端按 `id`
去重，显示 `gap`，断线只恢复读取，不重发运动请求。

- `arm=left/right/process`，`ecat_id`、`device_id` 来自入口配置。HTTP、Robot
  控制/看门狗线程、Hardware 状态线程传递上下文；RT 记录仍为固定 64 字节以内，
  不在实时入口分配字符串。公共启动/退出和非法臂名归属 process。
- 写操作及失败请求统一记录 `operation.name/request_id/outcome/http_status/duration_ms`，
  包括控制权、标定、坐标/负载及早返回错误。成功查询与成功心跳不产生操作事件；
  点动续发仍逐次记录。不会把 token、client_id、请求体或请求头写入操作审计。
- `accepted` 仅代表请求受理，不能当成运动完成。Robot 状态与运动结束/失败消息
  分别进入日志，不提供 request_id 与后台任务完成的关联；判断运动到位仍应读状态。
- 新缓存独立限制为 4096 帧或约 16 MiB，订阅配额与旧流共享。队列丢失、缓存淘汰、
  服务重启会报告 gap。此通道是有界诊断日志，不是不可丢失的审计存储。
- 原 `/api/logs` 与 `/api/logs/stream` 仍只有 warn/error/critical 六字段事件，
  单臂入口不开放双臂日志路径。`/api/logs/file/latest` 继续返回当前文件快照；
  双臂文件行增加 arm/ecat_id 和操作 request_id，文件级别/轮转由 logging 配置决定，
  下载不合并已轮转历史。

本地 Apifox 契约：同级 `rocos-mujoco/dualarm-openapi.yaml`。未上传在线项目。
页面：同级 `rocos-viz/design/dualarm/v2`，日志连接真实诊断端口，运动/标定仍为交互预案。
