# ROCOS-App 配置

实现依据：[已评审的配置文件规范](https://rocos-sia.feishu.cn/wiki/N1QKwpKtEicos3kpt55csqSrnrg)。本目录包含运行配置、Talon 模型资源和工具包；测试专用夹具仍位于 `test/`。本次不更改 EtherCAT 协议、控制算法或 HTTP API。

## 目录与入口

```text
config/
  app.yaml                           # 软件参数、机器人/设备/用户配置选择器
  user.yaml                          # 唯一用户差量，初始为空映射 {}
  robots/talon/
    robot.yaml                       # 型号、版本、模型链、硬件和主站选择
    hardware.yaml                    # 驱动器/PDO/换算/限位及 joint_name 绑定
    ecm.yaml                         # master 下的 18 个 ECM 参数
    eni7.xml                         # ENI 原件
    robot.urdf
    meshes/
  devices/talon_001/
    device.yaml                      # 设备身份、工具安装、辨识结果引用
    identification/                  # 仅放本设备实际产生的辨识结果
  tools/demo_gripper/
    tool.yaml
    gripper.urdf
    meshes/
  scripts/                           # 源码示例脚本目录
```

默认示例为 **Talon、ECM、无安装工具**。原有驱动器参数、主站运行参数、URDF、mesh、ENI 保留；不为示例虚构辨识结果。原 `payload.yaml` 的零附加载荷用无工具状态表达，原 `joint_binding.yaml` 的映射已由硬件 `joint_name` 完整承接。`demo_gripper` 是可选用的示例工具包，默认不安装。

主程序不再内嵌源码目录路径。未传 `--config` 时只读取 `/opt/rocos/config/app.yaml`；源码运行必须显式指定入口，没有自动目录搜索或回退。

```bash
cmake --build --preset release --parallel 2
# 不连接硬件、不启动 HTTP；检查配置、模型和绑定
./build/release/bin/rocosAppMain --config=config/app.yaml --check_config
# 校验并导出后退出，不启动主站或控制器
./build/release/bin/rocosAppMain --config=config/app.yaml --export_config=/tmp/talon-config
# 准备好主站或仿真器及快照目录权限后运行
./build/release/bin/rocosAppMain --config=config/app.yaml
```

`--config` 本身相对启动工作目录；此后所有路径相对**声明该路径的 YAML 文件**。例如 `app.yaml` 的 `robot_config` 相对 `app.yaml`，`robot.yaml` 的 `hardware_config` 相对 `robot.yaml`；`user.yaml` 中替换的路径相对 `user.yaml`。绝对路径也可用。工具资源仍遵守现有工具包完整性约束：模型、mesh 必须在包内，不能用绝对资源路径或 `..` 绕过包校验。

## 覆盖及校验

加载顺序：内置默认值 → 选中的基础文件 → 一份 `user.yaml` → 已有显式 HTTP 命令行参数。选择器先应用，再读取被选文件，不读取已被替换的旧文件。不自动加载同目录其他 YAML，也不将 `effective.yaml` 当作输入。

`user.yaml` 只允许四个命名空间：

- `software`：软件参数，以及 `robot_config`、`device_config` 选择器。
- `robot`：模型路径/链、硬件与主站选择器、完整 `hardware` 列表及 `master` 参数。
- `tool`：名义 `tcp` 位姿。
- `device`：工具选择、`tool_install`、辨识文件选择，以及已有 YAML 辨识结果参数的差量。

对象递归合并，标量替换，数组整体替换。`0`、`false` 和允许的空串均生效。`null` 只用于明确允许为空的字段，不能用于删除任意参数；恢复继承请删除用户键。拒绝重复 YAML 键、未知字段、非法枚举/类型、设备与型号不匹配、重复硬件绑定和不完整模型关节覆盖。型号名称/版本、设备身份、辨识 `context` 不可由用户差量改写。

```yaml
# config/user.yaml：只改变 HTTP 和现场 CPU/网卡参数
software:
  http_port: 8081
robot:
  master:
    cpuidx: 2
    instance: 1
```

替换硬件列表时必须提供完整列表；仅修改某个驱动器也不能依靠数组下标深合并。较大的现场硬件/PDO 差异可以用 `robot.hardware_config` 或 `robot.master_config` 选择另一份完整文件。

设备零点结果通过 `calibration_files.joint_zero` 引用，格式如下。`joints` 被适配到 `device.calibration.joint_zero`；实测零点替代型号名义 `offset_pos_cnt`，用户差量再替代实测值，不做累加。

```yaml
schema_version: 1
context:
  device_id: talon_001
  robot: {name: talon, package_version: 1.0.0}
joints:
  joint_1: {offset_pos_cnt: 1234}
```

安装示例工具可在 `user.yaml` 写：

```yaml
device:
  tool_config: tools/demo_gripper/tool.yaml
  tool_install:
    parent_link: link_7
    position: [0, 0, 0]
    orientation: [0, 0, 0, 1]  # xyzw
```

当前机器人消费者仅支持在 `robot.tip_link` 安装，其他 `parent_link` 会报错。最终工具变换为 `tool_install × 局部 TCP`，只组合一次。`tool_tcp.yaml` 的实测局部 TCP 优先于名义 `tool.tcp`。TCP/负载结果必须携带匹配的设备、型号、工具名称/版本及安装上下文；旧的只有 `mass/com` 的文件不能直接作为新版辨识结果。

无工具时 `tool_config`、`tool_install`、`calibration_files.payload/tool_tcp` 必须为 `null`。有工具但未提供负载辨识结果时保持“未知负载”，不会将其当成已辨识的零负载。运动学/动力学 NPZ 目前没有可接入的消费者，非空引用会明确报错；不读取其数组并假装生效。

## 主站和快照

App 组合并校验配置；**独立主站负责实际消费主站参数**。本次不修改相邻 `rocos-ecm`、`rocos-igh` 仓库，也不增加主站启动器或配置热重载。

- ECM：源文件只允许根级 `master`。导出 `ecm.yaml` 保持新结构；`ecm-legacy.yaml` 仅用于旧 ECM 解析器，把具名枚举转为原数字并展开顶层。路径已绝对化，`auxclk/flash` 使用后端内部默认值。
- IgH：支持规范版本 `version: 2` 的结构读取、覆盖、校验和导出，根级 `slaves` 在组合时进入 `robot.master.slaves`。不将旧版 `version: 1` 或新参数静默降级。当前独立 IgH 程序尚需实现新版参数与运行状态控制，不能直接把导出文件交给旧版程序。
- 运行主站必须使用同一份导出参数，不能再通过未记录的主站 CLI 覆盖周期/ID。App 校验 `robot.ecat_id == robot.master.id`，连接后核对共享内存周期；PDO 名称、类型/位宽及实际拓扑仍须与主站配置和设备一致。`--check_config` 不进行总线发现或设备读回。
- `master.state` 接受 `init/preop/safeop/op`；App 控制进程仅允许 `op`。运行中改配置切换主站状态需要独立主站和运动互锁协同实现，本次不会触发状态切换。

### 共享内存 v2

App 仅连接 ABI 2：`/ecm_v2_{id}`、`/pd_input_v2_{id}`、`/pd_output_v2_{id}`、
`/sync_v2_{id}_{slot}`。ID 仍取 `robot.ecat_id`，无需在 `app.yaml` 配置内存文件名或布局版本。
公共库验证 magic、精确 ABI、结构尺寸、ready、PDO 长度及描述符边界后才开放访问；
连接失败会抛出可报告的异常，不创建空对象，也不扫描或兼容旧 Boost/v1 布局。

容量固定为 50 个从站、每从站每方向 25 个 PDO。IgH 的 `bit_length` 必须为 8 的倍数，
不支持位级 PDO；多字节 PDO 不要求自然对齐，App 缓存索引后通过公共库的 memcpy 值接口读写。
动态总线字段使用公共库原子访问方法；周期信号仅用于通知，不保证整帧 PDO 同周期一致性。

ECM 的 `master.command_timeout_ms` 默认 1000，允许 1–60000 ms，随主站配置导出，
由主站限制非实时邮箱请求的绝对截止时间。此参数不改变控制周期，也不改变客户端本地等待时间。
同一总线的 App 线程共用 `EcatConfig::getInstance(id)` 返回的对象及邮箱 mutex。
SDO 和统计复位只能在非实时线程调用；`resetCycleTime()` 已由公共库改为提交 `CMD_RESET_STATS`。
本地超时后必须用 `pollCommand()` 消费最终结果，或停机重连，不能清空邮箱或自动重试写入。
这些是 C++ 共享内存能力，本次未新增 HTTP 路由。

部署必须先停止主站、App 和监控进程，使用相同固定公共库版本重编译，再先启动主站、后启动 App。
每个总线只允许一个可写 App，由启动管理器保证；其他监控工具使用 `connect(true)`，
不得消费周期信号或写邮箱。无法保证单实例时不得启用。任一端异常退出后先停止另一端并关闭旧映射，
再重启两端，不支持热接管。仍使用旧共享内存的模拟器也必须迁移至 v2 才能连接。
实际设备身份、SDO 状态限制、故障超时和周期抖动仍需分别完成 ECM/IgH 实机验收。

普通启动在硬件初始化前写 `~/.local/state/rocos/config_snapshots/<device_id>/effective.yaml`；写失败则不初始化机器人。它是完成校验的**启动配置快照**，包含默认值、用户覆盖、最终显式 HTTP 参数、选中来源列表、用户覆盖字段和辨识上下文。它不代表硬件启动成功或设备读回结果，`snapshot.master_parameters_applied: false` 明确表示 App 未向独立主站下发参数。运行期 API 改动不会自动改写这份启动快照。

URDF、mesh、NPZ、ENI 在合并层保持不透明，只保留引用，不展开或嵌入快照。快照为原子替换、文件权限 `0444`，授权字段脱敏；主站派生配置含运行所需授权，权限 `0600`。导出拒绝覆盖本次读取的源配置文件。

新启动流程从 `device.yaml` 初始化工具，不叠加旧 `frames.yaml`。现有帧/工具导入导出接口仍保留其文件语义，默认帧输出位于设备目录的 `frames.yaml`；显式导入仍可使用。该输出不会成为下一次启动的隐含覆盖层；需要随启动持久采用的安装/TCP 参数应写入设备配置及其辨识引用。负载保存到已有新版结果文件时保留上下文；存在用户负载差量时拒绝默认回写，避免将人工修正伪装成原始测量。未引用的负载导出不会自动修改 `device.yaml` 的引用。

## 部署权限与升级

仓库不会自动 `chmod/chown` 开发目录。部署时按以下边界授权，目录本身也须限制写权限：

| 路径 | 普通用户权限 | 维护者 |
| --- | --- | --- |
| `config/app.yaml` | 只读 | 部署管理员；升级保留 |
| `config/robots/`、`config/tools/` | 只读 | 发行包维护者 |
| `config/user.yaml` | 授权维护用户可写 | 现场维护；升级保留 |
| `config/devices/` 与现场资源 | 按设备授权写入 | 现场维护；升级保留 |
| `~/.local/state/rocos/config_snapshots/<id>/` | 服务账户可原子替换输出 | 服务账户 |

快照目录根据运行账户的 `HOME` 自动创建，无需预先创建 `/var/lib/rocos` 或使用 sudo。Talon 示例保存到 `$HOME/.local/state/rocos/config_snapshots/talon_001/effective.yaml`。服务运行时应提供有效且可写的用户主目录；`HOME` 缺失或不是绝对路径时明确报错。启动失败记录为 Error 级别日志；日志系统尚未初始化时，标准错误输出使用 `[ERROR]` 标识。

`robots/tools` 目录建议管理员所有、目录 `0755`/文件 `0644`；含授权的主站源文件可用 `0640` 并给服务账户读取组。`app.yaml` 管理员所有；`user.yaml` 用维护组 `0660`，父目录保持管理员所有，防止普通用户替换只读入口。设备辨识目录由允许保存结果的账户维护。服务账户需读取全部所选资源，并写入日志、工具缓存、脚本及快照目录。生产部署可在入口中将日志、工具库、脚本路径分别设为 `/var/log/rocos`、`/var/lib/rocos/tools`、`/var/lib/rocos/scripts`。

升级不得直接覆盖整个 `config/`。保留已有 `app.yaml`、`user.yaml`、`devices/` 和现场自定义文件，仅更新归属明确的发行型号/工具包。本仓库现有 CMake 未启用完整安装器，本次不改变该打包边界。

## 代码入口

- `src/app_config.cpp`：文件选择、来源路径、合并、校验、结果适配及快照/主站导出。
- `src/rocosAppMain.cpp`：加载入口、显式 CLI 优先级、离线检查和启动快照。
- `src/hardware.cpp`：复用 `LoadConfigFromNode` 将合并结果转成已有 `HardwareConfig`；构造硬件时使用该结果。
- `src/robot.cpp`：使用硬件 `joint_name` 绑定模型链，应用零点换算后的硬件配置、负载与工具。
- `src/frame_registry.cpp`：将已选择的工具包及设备安装/TCP 初始化为运行时不可变快照。

旧单文件配置仍可通过显式 `--config=<旧文件>` 使用原解析路径，不能与新版覆盖混用；不支持由旧格式生成完整组合快照。仓库不再保留重复的旧默认配置。
