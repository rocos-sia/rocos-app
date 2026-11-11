# config/ 下 YAML 配置文件使用情况分析

核查日期：2026-10-24。代码基线：`58eaead`，并包含核查时工作区已有的 `config/frames.yaml`（当时为未跟踪文件）。

初次分析覆盖清理前 `config/` 及其子目录中的全部 YAML 文件，共 **15 个 `.yaml`，无 `.yml`**。依据为当前源代码、CMake 构建清单、测试和配置之间的引用关系；“未使用”指未发现当前仓库的有效加载调用，不排除外部程序或用户自行传入文件路径。旧文档中的用途描述不作为已接入代码的证据。

清理记录（2026-10-24）：删除未接入代码的 `hardware_config.yaml`、`robotDH.yaml`、`joint_impedance_control.yaml`、`calibration.yaml`；将接口描述文件迁至 [docs/rocos-API.yaml](rocos-API.yaml)，保留其内容和文档用途。**清理后 `config/` 保留 10 份 YAML：5 份默认启动配置、5 份测试样例，另保留 `scripts/` 下的 Lua 脚本。** 本文保留已删除文件的核查结论，供追溯清理依据。

在线文档同步（2026-10-24）：迁移后的 `docs/rocos-API.yaml` 已按 Apifox MCP「ROCOS API」刷新，在线版本为权威来源。现为 OpenAPI 3.1.0、文档版本 3.0.0，包含 76 个路径、82 个接口操作、48 个 schema；MCP 的分拆引用已合并成独立 YAML。这是接口文档版本，不是 ROCOS-App 软件版本变更。

## 1. 使用情况总表

当前默认启动链涉及 **5 个文件**；另外 **5 个文件有离线测试读取**。表中最后五项为已删除或迁移的文件。

| 文件 | 当前使用状态 | 功能或内容 | 加载入口 / 关键证据 |
| --- | --- | --- | --- |
| [config.yaml](../config/config.yaml) | **主程序默认读取**，也有测试读取 | HTTP、日志、诊断服务、机器人资源路径、脚本和 Web 目录 | `main()` → `AppConfig::load()`；[rocosAppMain.cpp:28](../src/rocosAppMain.cpp#L28)、[app_config.hpp:40](../src/app_config.hpp#L40) |
| [hardware_talon_config.yaml](../config/hardware_talon_config.yaml) | **默认硬件配置**；硬件集成程序也引用 | Talon 七轴驱动器，ID 0～6；PDO 映射、单位换算、力矩来源等 | `config.yaml` 的 `robot.hardware_config` → `Robot` → `Hardware`；[robot.cpp:618](../src/robot.cpp#L618) |
| [joint_binding.yaml](../config/joint_binding.yaml) | **默认启动时尝试读取** | `joint_1`～`joint_7` 显式绑定到驱动器 ID 0～6 | `Robot::jointBinding()` → `JointBinding::LoadFromYaml()`；[robot.cpp:522](../src/robot.cpp#L522) |
| [payload.yaml](../config/payload.yaml) | **默认启动时尝试读取**，可运行时保存 | 零力拖动负载质量和质心；当前质量为 0 | `Robot::LoadLoadParameters()`；[robot.cpp:628](../src/robot.cpp#L628)、[robot.cpp:984](../src/robot.cpp#L984) |
| [frames.yaml](../config/frames.yaml) | **默认启动时存在则读取**，可运行时读写 | 工具 / 工件坐标系、活动坐标系，以及可选的工具包安装覆盖数据 | `Robot` → `FrameRegistry::load()`；[robot.cpp:622](../src/robot.cpp#L622)、[frame_registry.cpp:189](../src/frame_registry.cpp#L189) |
| [hardware_driver_config.yaml](../config/hardware_driver_config.yaml) | **测试读取**；默认主程序不选用 | 六个驱动器，ID 1～6 | [hardware_test.cc:50](../test/hardware_test.cc#L50) → `Hardware::LoadConfigFromYAML()` |
| [hardware_ft_config.yaml](../config/hardware_ft_config.yaml) | **测试读取**；默认主程序不选用 | 一个六维力传感器，ID 10，含非零偏置 | [hardware_test.cc:100](../test/hardware_test.cc#L100) → 同一硬件解析器 |
| [hardware_io_config.yaml](../config/hardware_io_config.yaml) | **测试读取**；默认主程序不选用 | 一个 IO 模块，ID 20，16 DI / 16 DO / 8 AI / 4 AO | [hardware_test.cc:138](../test/hardware_test.cc#L138) → 同一硬件解析器 |
| [hardware_ft_and_driver_config.yaml](../config/hardware_ft_and_driver_config.yaml) | **测试读取**；默认主程序不选用 | 两个驱动器 ID 1、2，以及六维力传感器 ID 7 | [hardware_test.cc:170](../test/hardware_test.cc#L170) → 同一硬件解析器 |
| [hardware_all_config.yaml](../config/hardware_all_config.yaml) | **测试读取**；默认主程序不选用 | 驱动器 ID 1、2，六维力传感器 ID 10，IO ID 20 | [hardware_test.cc:210](../test/hardware_test.cc#L210)、[hardware_test.cc:346](../test/hardware_test.cc#L346) |
| `hardware_config.yaml`（已删除） | **未发现加载调用；通用示例** | 驱动器 ID 4、六维力传感器 ID 5、IO ID 6 | 格式可由现有 `Hardware` 解析器读取，但默认配置和测试均未选用这个文件 |
| `robotDH.yaml`（已删除） | **有解析器，但未接入当前构建和调用链** | 六组改进 DH 参数，机器人名 `Robot_pan` | [DHParamsLoader::loadFromYAML()](../src/dh_params_loader.cpp#L22) 存在；其 `.cpp` 未列入当前 CMake 目标，未找到调用方 |
| `joint_impedance_control.yaml`（已删除） | **未发现加载入口或字段解析代码** | 七轴阻抗 / 导纳相关参数及传感器系数等 | 当前 `JointImpedanceController` 使用代码默认值或 C++ setter，见第 5 节 |
| `calibration.yaml`（已删除） | **未发现加载入口或字段解析代码** | `T_tool_`、`T_object_` 两组六元素数据 | 当前坐标系由 `FrameRegistry` 管理，标定保存也写入该注册表，见第 5 节 |
| [rocos-API.yaml](rocos-API.yaml)（已迁至 docs/ 并同步 Apifox） | **接口描述文档，非运行时配置** | OpenAPI 3.1 文档：接口操作、数据 schema、鉴权定义等 | HTTP 路由在 C++ 中注册；未发现读取此文件来创建服务或生成路由的当前构建步骤 |

“测试读取”表示有实际读取文件的测试代码，不表示主程序会自动加载这些文件。所有 `hardware*.yaml` 共用同一个按路径读取的解析器；修改 `robot.hardware_config` 或向 `Hardware` 显式传入路径，可以选用其他文件，但仍需同时满足模型、关节绑定及 EtherCAT 从站布局要求。

## 2. 默认启动调用链及路径规则

```text
CMakeLists.txt:223
  ROCOS_DEFAULT_CONFIG_FILE = <源码根目录>/config/config.yaml
    ↓ gflags --config 可覆盖
rocosAppMain.cpp:28  AppConfig::load(FLAGS_config)
    ├─ YAML::LoadFile → 解析字段 → 解析相对路径 → validate()
    ├─ logging → LogManager::configure()
    ├─ diagnostics → DiagnosticsHttpServer → start()
    └─ Robot(config.robot)                         robot.cpp:612
         ├─ 构造 FrameRegistry（此时只记录路径，不读取 YAML）
         ├─ Hardware(hardware_config, ecat_id)
         │    └─ loadFromYAML → parseDrive / parseFTSensor / parseIO
         ├─ Model(urdf, base_link, tip_link)        读取 URDF，不读取 robotDH.yaml
         ├─ jointBinding → LoadFromYaml            joint_binding.yaml
         ├─ frames_.load()                        frames.yaml 存在时
         ├─ frames_.reloadPackages(false)         扫描工具库
         └─ LoadLoadParameters()                  payload.yaml
    ↓
LuaScriptEngine(robot, scripts)
RobotHttpServer(robot, ..., web_root) → runAsync(http_host, http_port)
```

路径处理集中在 [AppConfig::load()](../src/app_config.hpp#L40)：先把入口 YAML 路径转为绝对路径，再把 `robot.hardware_config`、`robot.urdf`、`robot.joint_binding`、`robot.payload`、`robot.frames`、`robot.tool_library`、`scripts`、`web_root`、`logging.directory` 相对入口文件所在目录解析（[app_config.hpp:96](../src/app_config.hpp#L96)）。绝对资源路径保持绝对路径语义。

以当前 `config/config.yaml` 为入口，硬件、绑定、负载、坐标系均位于 `config/`；URDF 位于 `models/talon/robot.urdf`；工具库、日志、Web 分别位于仓库根目录下的 `tools/`、`logs/`、`web/`，脚本目录为 `config/scripts/`。

需要区分三种调用方式：

1. **主程序启动**：`--config` 指定入口 YAML；若 `--config` 自身是相对路径，其基准仍为进程当前工作目录。入口文件内部的资源路径随后按入口文件目录解析。
2. **直接使用 C++ 库**：`Robot()` 使用 [Robot::Config](../src/robot.hpp#L109) 的默认字符串；`Robot(config)` 和 `Hardware(path, ...)` 不会自动经过 `AppConfig::load()`。直接传入的相对路径仍受进程工作目录影响。
3. **运行时 load/save 显式指定 path**：负载和坐标系读写函数直接使用该路径，不再套用 `AppConfig` 的目录解析规则；空路径才回落到启动时记录的默认文件路径。加载或另存为其他路径不会自动改写默认路径。

构建配置明确说明运行配置不复制到 `bin/`（[CMakeLists.txt:247](../CMakeLists.txt#L247)）。源码目录的 YAML 修改后，在相应启动 / 重新加载时生效，无需为 YAML 内容重新编译；但当前没有监视文件变化并自动重载的机制。

## 3. 默认启动涉及的配置详解

### 3.1 config.yaml：统一启动配置

加载调用：[rocosAppMain.cpp:28](../src/rocosAppMain.cpp#L28)。实际文件读取、字段转换与校验：[app_config.hpp:40](../src/app_config.hpp#L40)。

| 字段 | 功能和消费位置 |
| --- | --- |
| `http_host`、`http_port` | 控制 HTTP 服务监听地址；[rocosAppMain.cpp:59](../src/rocosAppMain.cpp#L59) |
| `http_control_ttl_seconds` | HTTP 控制权有效期；传给 `RobotHttpServer` 构造函数，[rocosAppMain.cpp:58](../src/rocosAppMain.cpp#L58) |
| `robot.hardware_config`、`robot.ecat_id` | 硬件 YAML 路径和 EtherCAT 实例 ID；[robot.cpp:618](../src/robot.cpp#L618) |
| `robot.urdf`、`robot.base_link`、`robot.tip_link` | 运动学 / 动力学模型及链首尾；[robot.cpp:619](../src/robot.cpp#L619)、[model.cpp:73](../src/model.cpp#L73) |
| `robot.joint_binding`、`robot.payload`、`robot.frames` | 指定以下三类运行时文件；不在入口解析器内递归读取内容 |
| `robot.id`、`robot.tool_library` | 工具安装记录所属机器人标识、工具包库目录；[robot.cpp:612](../src/robot.cpp#L612)、[frame_registry.cpp:22](../src/frame_registry.cpp#L22) |
| `scripts` | Lua 脚本根目录；[rocosAppMain.cpp:57](../src/rocosAppMain.cpp#L57) |
| `web_root` | HTTP 静态资源根目录；[rocosAppMain.cpp:58](../src/rocosAppMain.cpp#L58) |
| `logging.directory/file_level/console_level/max_file_size_mib/max_files/flush_interval_ms` | 日志目录、输出级别、滚动大小及数量、刷新周期；[app_config.hpp:71](../src/app_config.hpp#L71) 解析，[rocosAppMain.cpp:36](../src/rocosAppMain.cpp#L36) 应用 |
| `diagnostics.host/port/max_sse_clients/max_downloads` | 独立诊断 HTTP 服务、SSE 和下载并发限制；[app_config.hpp:84](../src/app_config.hpp#L84) 解析，[rocosAppMain.cpp:38](../src/rocosAppMain.cpp#L38) 应用 |

缺失字段使用结构体默认值，已提供但类型不正确的字段会报错；根节点和 `robot` / `logging` / `diagnostics` 必须符合代码要求。`validate()` 还检查端口、控制权 TTL、机器人名称等。代码只读取列明的键，未实施通用的未知字段拒绝检查。

显式传入的 `--http_host`、`--http_port`、`--http_control_ttl_seconds` 覆盖 YAML 值（[rocosAppMain.cpp:29](../src/rocosAppMain.cpp#L29)）。启动在构造 `Robot` 前检查硬件文件、URDF 文件和脚本目录；初始化异常由 `main()` 捕获并返回失败。

### 3.2 hardware_talon_config.yaml：硬件布局、PDO 和单位转换

完整调用链为：`Robot::Robot(config)` → `Hardware(config.hardware_config, config.ecat_id)` → `Hardware::loadFromYAML()` → 按 `hardware[].type` 分派解析。

| 解析位置 | 读取字段 | 运行时用途 |
| --- | --- | --- |
| [hardware.cpp:336](../src/hardware.cpp#L336) `loadFromYAML()` | 顶层 `hardware` 列表、各项 `type` | 分离驱动器、六维力传感器和 IO 模块 |
| [hardware.cpp:67](../src/hardware.cpp#L67) `parseDrive()` | `id`、`joint_name`、`torque_source`、`limit`、`transform`、`inputs`、`outputs` | 驱动器配置；具体字段生效边界见下文 |
| [hardware.cpp:128](../src/hardware.cpp#L128) `parseFTSensor()` | `id`、六维 `inputs` 和 `offset` | 六个力 / 力矩 PDO 输入通道及偏置 |
| [hardware.cpp:159](../src/hardware.cpp#L159) `parseIO()` | `id`、四种通道数量、`inputs`、`outputs` | 数字 / 模拟 IO 的边界检查和 PDO 映射 |

解析后，`Hardware` 构造函数建立 ID 查找表、连接 EtherCAT 共享内存并调用 [buildPDOCache()](../src/hardware.cpp#L1025)，按从站 ID 和 YAML 中的变量名查找 PDO 指针。运行时使用缓存指针，不会每周期重新解析 YAML。

当前 Talon 文件包含七个 `driver`，不包含 `ft_sensor` 或 `io`。七轴都选 `torque_source: load_torque`，因此 [GetJointLoadTorque()](../src/hardware.cpp#L532) 读取 `inputs.load_torque_value` 指向的通道，按 `load_torque_per_unit` 换算；虽然也配置了 `secondary_position_value`，当前选择下它不作为此函数的返回来源。选 `secondary_position` 时才转而读取该通道，同样按 `load_torque_per_unit` 换算，不扣位置零点或除传动比。

位置换算位于 [hardware.cpp:398](../src/hardware.cpp#L398)，采用 `(raw_cnt - offset_pos_cnt) / pos_per_unit / ratio`；力矩换算位于 [hardware.cpp:410](../src/hardware.cpp#L410)。`joint_name`、单位名称和转换参数也被 [Robot::GetJointInfo()](../src/robot.cpp#L1117) 用于关节信息输出。当前第 7 轴存在非零 `offset_pos_cnt`，属于实际参与换算的数据。

**文件被加载并不表示每个字段都进入控制算法：**

- `limit.lower/upper/vel/acc/jerk/effort` 已解析存入 `Drive::limit`，但当前生产代码未发现使用该结构进行限位检查的调用。以关节阻抗控制为例，[ValidateTorqueCommand()](../src/joint_impedance_controller.cpp#L216) 从 `model_` 获取位置、速度、力矩限制；这些模型限制来自 URDF 的 `<hardware><limit>` 解析及默认值（[model.cpp:30](../src/model.cpp#L30)、[model.cpp:223](../src/model.cpp#L223)）。不能把修改硬件 YAML 的 `limit` 等同于修改模型限位。
- `hardware_config.yaml`、`hardware_driver_config.yaml`、`hardware_ft_and_driver_config.yaml`、`hardware_all_config.yaml` 含 `inputs.secondary_velocity_value`，但 `parseDrive()`、`Drive` 数据结构及 PDO 缓存没有相应读取项；该字段当前被忽略。Talon 文件没有这个字段。
- `torque_source` 仅精确匹配 `secondary_position`；其他字符串均走 `load_torque` 分支，未对拼写错误单独报错（[hardware.cpp:73](../src/hardware.cpp#L73)）。
- 六维力传感器可解析多个，但 [GetWrench()](../src/hardware.cpp#L870) 当前使用第一个传感器，计算为原始值加 `offset`。模拟 IO 的 `channel` 会用于范围检查，但 [GetAnalogInput() / SetAnalogOutput()](../src/hardware.cpp#L963) 当前读写同一个标量 PDO 指针，没有按通道索引数组；配置多个通道数量并不等于已实现多通道寻址。

错误处理：缺失或非列表的 `hardware` 会抛异常；YAML 解析错误向外抛出；缺少 `type` 或未知 `type` 的条目记录警告后跳过（[hardware.cpp:344](../src/hardware.cpp#L344)）。

### 3.3 joint_binding.yaml：模型关节到硬件驱动器的映射

[Robot::jointBinding()](../src/robot.cpp#L522) 先从模型获取关节名，从硬件获取驱动器 ID，再调用 `JointBinding::Configure()`。文件可读时调用 [LoadFromYaml()](../src/joint_binding.cpp#L102)，读取：

- `binding_policy`：默认 `explicit`，当前只支持该值。
- `joint_bindings[]`：每项包含 `joint_name`、`drive_id`。

[Validate()](../src/joint_binding.cpp#L231) 要求映射覆盖所有模型关节，引用已存在的驱动器 ID，且不重复使用 ID。随后 [robot.cpp:590](../src/robot.cpp#L590) 按模型关节顺序生成驱动器 ID 数组，调用 [Hardware::SetJointBinding()](../src/hardware.cpp#L808)，使批量位置、速度、力矩操作使用正确的模型顺序。

文件打不开时，只有模型关节数与驱动器数相同才按各自列表顺序绑定；否则启动失败。文件可读但内容无效也会导致启动失败。`JointBinding::SaveToYaml()` 虽已实现（[joint_binding.cpp:166](../src/joint_binding.cpp#L166)），当前未发现生产调用，因此不能认为启动时自动生成 / 回写该文件。

### 3.4 payload.yaml：零力拖动负载参数及持久化

[Robot::LoadLoadParameters()](../src/robot.cpp#L984) 使用启动配置记录的负载路径；`mass` 必填，`com` 可省略并默认为零。`com` 存在时要求为至少三个元素的序列，使用前三项。质量及质心还经 [SetLoadParameters()](../src/robot.cpp#L966) 检查有限性，质量不能为负。

加载成功后设置 `load_mass_`、`load_com_` 和 `load_params_valid_`。切换到 `zero_force_drag` 时，[robot.cpp:834](../src/robot.cpp#L834) 将有效参数交给 `ZeroForceDragController`，跳过负载辨识。**当前 `mass: 0.0` 是有效的“无末端负载”配置，不表示参数未设置。**

启动时文件缺失或加载返回错误不会在该调用点抛出启动失败；初次加载未成功时参数保持未就绪，零力拖动可进入辨识流程。辨识完成的回调会更新 Robot 参数并调用 `SaveLoadParameters()` 保存到默认文件（[robot.cpp:842](../src/robot.cpp#L842)）。

其他调用入口：

| 入口 | 行为 | 代码 |
| --- | --- | --- |
| `POST /api/robot/load_params` | 修改内存中的质量 / 质心，不自动保存文件 | [robot_http_server.cpp:1146](../src/robot_http_server.cpp#L1146) |
| `POST /api/robot/load_params/save` | 调用 `SaveLoadParameters(path)`，写出 `mass`、`com` | [robot_http_server.cpp:1203](../src/robot_http_server.cpp#L1203)、[robot.cpp:1028](../src/robot.cpp#L1028) |
| Lua `LoadLoadParameters` / `SaveLoadParameters` | 显式读取 / 保存，可选路径 | [lua_script_engine.cpp:695](../src/lua_script_engine.cpp#L695) |

没有发现对应的 HTTP `load_params/load` 路由；HTTP 设置负载参数不能当作读取 YAML 文件。

### 3.5 frames.yaml：工具 / 工件坐标系及工具包安装状态

`FrameRegistry` 构造时只保存路径；[robot.cpp:622](../src/robot.cpp#L622) 在文件存在时显式调用 [FrameRegistry::load()](../src/frame_registry.cpp#L189)。当前文件为空注册表：`tools: {}`、`objects: {}`，活动名称均为空，属于有效配置。

解析字段和含义：

| 字段 | 含义 |
| --- | --- |
| `tools.<name>.position/orientation` | 工具相对法兰的变换；位置三个元素，姿态四元数 `[x,y,z,w]` |
| `objects.<name>.position/orientation` | 工件坐标系变换；相同位置 / 四元数格式 |
| `active_tool`、`active_object` | 活动坐标系名称；非空时必须在相应集合存在 |
| `robot_id`、`package_tools` | 可选工具包安装数据；有包记录时校验机器人 ID、包摘要、版本，读取安装 `mount`、局部 `tcp`、`revision`、可选 `calibration` 元数据 |

解析器检查名称、向量维数、有限性、旋转有效性，并归一化四元数。包工具的有效工具变换为 `mount * tcp`。普通工具变换用于 [resolveTcp()](../src/frame_registry.cpp#L52) 计算 TCP；有名称的工具也用于 [Robot::MoveL 的目标转换](../src/robot.cpp#L1262)，活动工件坐标系用于点动参考坐标转换（[robot.cpp:1627](../src/robot.cpp#L1627)）。

文件不存在时允许以空注册表启动；文件存在但加载失败则由 `Robot` 抛异常终止初始化。之后调用 `reloadPackages(false)` 扫描工具库，但该启动扫描不自动持久化。

运行时读写路径：

- `POST /api/robot/frames/load` → [handleLoadFrames()](../src/robot_http_server.cpp#L2373) → `FrameRegistry::load(path)`。
- `POST /api/robot/frames/save` → [handleSaveFrames()](../src/robot_http_server.cpp#L2388) → `FrameRegistry::save(path)`。
- C++ `Robot::LoadFrames()` / `SaveFrames()` 包装注册表调用（[robot.cpp:1418](../src/robot.cpp#L1418)）；Lua 也暴露对应方法（[lua_script_engine.cpp:779](../src/lua_script_engine.cpp#L779)）。
- 普通坐标系修改默认只更新内存；HTTP 工具设置、删除、激活可带 `persist`，默认 `false`（[robot_http_server.cpp:2267](../src/robot_http_server.cpp#L2267)）。工具包导入、安装覆盖更新和显式标定保存会走持久化路径。
- `POST /api/calibration/save` 保存普通工具或包工具标定到注册表（[tool_package_http.cpp:61](../src/tool_package_http.cpp#L61)），**不是写入 `config/calibration.yaml`**。

底层写出由 [FrameRegistry::write()](../src/frame_registry.cpp#L131) 完成：先写临时文件，刷新并同步，再原子替换目标文件。文件属于可变运行数据，后续内容可能超出当前四个基础键。

## 4. 仅在测试中选用的硬件 YAML

五份测试配置的加载统一为：

```text
test/hardware_test.cc 中的测试用例
  → cfg("hardware_...yaml")
  → Hardware::LoadConfigFromYAML(path)       hardware.cpp:390
  → Hardware::loadFromYAML(path)            hardware.cpp:336
  → 解析数据结构并断言
```

这条静态入口只做配置解析，**不构造 `Hardware`，不初始化 EtherCAT**（[hardware.hpp:249](../src/hardware.hpp#L249)）。测试覆盖三种设备类型、字段、默认值、错误输入及配置查找。

路径定位函数 [hardware_test.cc:26](../test/hardware_test.cc#L26) 实际先尝试 `../../config/<name>`，失败后使用 `config/<name>`，不能只依据其旧注释判断运行目录。当前 CMake 最终把 `hardware_test` 等离线测试的工作目录设为 `<build>/test-work`，并为 `config`、`models` 创建指向源码的符号链接（[CMakeLists.txt:578](../CMakeLists.txt#L578)）。

此外，Talon 文件被这些集成程序引用：

| 程序 | 引用与用途 | 构建 / 执行状态 |
| --- | --- | --- |
| `hardware_enable_test` | [hardware_enable_test.cc:95](../test/hardware_enable_test.cc#L95) 默认选择 Talon，可通过第二个位置参数覆盖配置路径；实际构造 `Hardware` 并执行上使能 / 运动 | CMake 有可执行目标，未注册为 CTest |
| `mode_switch_test` | [mode_switch_test.cc:42](../test/mode_switch_test.cc#L42) 默认选择 Talon，可覆盖路径；实际构造 `Hardware` 并测试位置 / 力矩模式 | CMake 有可执行目标，未注册为 CTest |
| `test_hardware_dt.cpp` | [test_hardware_dt.cpp:45](../test/test_hardware_dt.cpp#L45) 默认使用 Talon 文件名，第一个位置参数可指定 YAML；构造 `Hardware` 后读控制周期 | 源码存在，当前 CMake 未声明对应目标 |

前两个程序使用各自的 `resolveConfig()`，没有通过 `AppConfig` 加载启动配置；`test_hardware_dt.cpp` 直接使用传入路径。该文件中“在 build/bin 使用复制过来的 YAML”的注释与当前 CMake 不复制配置的行为不一致。

## 5. 已删除或迁移文件的核查依据

以下记录清理前的文件内容及调用关系；四份旧配置已删除，解析器源码和运行逻辑未修改，OpenAPI 文件已迁至 `docs/`。

### 5.1 hardware_config.yaml：可解析的通用模板，当前未被选用

内容为三类硬件的示例，各一个。当前 `config.yaml` 指向 `hardware_talon_config.yaml`，`Robot::Config` 的默认硬件文件名也是 Talon；`hardware_test` 选择了其他五份测试配置。未找到读取 `hardware_config.yaml` 的当前调用。

它不是格式失效文件：可以被现有硬件解析器读取。只是直接拿来替换默认配置时，其一个驱动器与 Talon 七关节模型 / 当前显式绑定并不匹配；“可解析”不等于“可直接作为当前七轴启动配置”。

### 5.2 robotDH.yaml：解析能力保留在源码，当前模型从 URDF 构建

[DHParamsLoader::loadFromYAML()](../src/dh_params_loader.cpp#L22) 能解析 `robot_name`、`base_link`、`tip_link`、`dh_convention`，以及 `dh_parameters[]` 中的 `joint_name/alpha/a/d/theta/type/offset`，然后调用 `buildChainFromDH()` 构造 KDL 链。

但当前有三项反证说明该文件未进入运行链：

1. [CMakeLists.txt:151](../CMakeLists.txt#L151) 的 `robot` 库源文件列表未包含 `src/dh_params_loader.cpp`，其他当前目标也未加入它。
2. [robot.hpp:37](../src/robot.hpp#L37) 的 `dh_params_loader.hpp` include 已注释；未找到 `DHParamsLoader` 的外部实例化或 `loadFromYAML()` 调用。
3. 实际构造调用是 [robot.cpp:619](../src/robot.cpp#L619) 的 `Model(config.urdf, ...)`；[model.cpp:73](../src/model.cpp#L73) 读取 URDF 并 `ParseUrdf()`。

因此修改 `robotDH.yaml` 不会改变当前主程序的运动学模型。仅有解析器源码，不能算该配置已被应用加载。

### 5.3 joint_impedance_control.yaml：当前控制器没有读取该文件

文件包含 `kesai`、`collection_force`、`sensitivity`、`M`、`K`、`F_joint_stop`、`a_sensor`、`b_sensor`、`joint_K`、`ZeroOffset`。未找到文件名引用或对应 YAML 解析入口；特征键如 `kesai`、`a_sensor`、`F_joint_stop` 也未在生产代码中形成读取链。`M`、`K` 这类普通变量名在算法中出现，不构成读取该 YAML 键的证据。

实际 `jnt_imp` 模式在 [Robot::SetWorkMode()](../src/robot.cpp#L828) 创建 `JointImpedanceController`。其 [UpdateCmd() 的增益初始化](../src/joint_impedance_controller.cpp#L152) 在未设置参数时按轴填充代码默认值：刚度 `500.0`、阻尼 `20.0`（[joint_impedance_controller.hpp:78](../src/joint_impedance_controller.hpp#L78)）。C++ 提供 `SetStiffness()`、`SetDamping()` 等方法（[joint_impedance_controller.cpp:268](../src/joint_impedance_controller.cpp#L268)），测试通过这些方法注入参数；主程序没有把本文件接到这些 setter。

所以，修改该文件目前不会改变关节阻抗控制参数。若以后接入，需要明确新配置字段与实际控制器参数的映射，不能假定旧字段已经生效。

### 5.4 calibration.yaml：旧格式数据未接入当前坐标系功能

文件只包含 `T_tool_`、`T_object_` 两组六元素值。未找到这两个 YAML 键的解析或此文件的加载。当前实现使用 `frames.yaml` 的具名坐标系、三元素位置和四元素四元数；工具包安装记录还区分 `mount` 与 `tcp`。

因此修改 `calibration.yaml` 不会改变当前工具 / 工件坐标系。虽然 `test/app_config_test.cc` 中出现 `tools/calibration.yaml` 字符串，它只是验证路径解析的测试数据，不是读取 `config/calibration.yaml` 的证据。

### 5.5 rocos-API.yaml：OpenAPI 文档

该文件描述 REST API，并非控制器参数。HTTP 路由由 [robot_http_server.cpp:285](../src/robot_http_server.cpp#L285) 等位置直接注册，处理函数直接实现行为；未发现把此 YAML 作为路由生成或运行时接口定义来源的当前代码 / 构建步骤。

修改该文件只能修改接口说明，不会自动增加路由或改变返回码。本次只确认它的使用性质，没有逐接口审计文档与实现是否一致。

## 6. 维护时应关注的差异

| 现象 | 当前事实与维护建议 |
| --- | --- |
| 配置、测试样例、旧参数和 API 文档混放 | 已删除四份未接入的配置，API 文档迁至 `docs/`；五份有测试读取的硬件样例继续保留 |
| `CLAUDE.md` 曾将 DH、阻抗及标定 YAML 描述成运行时数据 | 已同步修订配置目录说明、启动命令及资源路径说明 |
| 硬件 YAML 中的 `limit` 看起来像控制限位来源 | 已解析但未发现被生产控制路径消费；需要分别核对硬件配置与模型 URDF 的限制 |
| `secondary_velocity_value` 出现在若干样例中 | 当前解析器忽略该键；后续应补实现或标注为未支持 |
| `frames.yaml` 核查时为未跟踪文件 | 本次已纳入分析；其他检出环境不一定带有该文件，启动允许其缺失，但已有坐标系数据需自行管理 |
| 负载与坐标系 YAML 可由 API / Lua / 标定结果写回 | 它们兼具启动配置与持久化状态角色；默认保存目标为启动配置解析后的路径 |
| 部分集成程序仍按工作目录寻找配置 | 不适用主入口的自动路径解析；运行时应显式传入适合该程序的配置路径 |

清理仅删除未接入的配置、迁移 API 文档并修订相关说明；保留的配置内容、控制器与构建逻辑未修改。

## 7. 核查方法与验证范围

本次完成以下核查：

- 初次分析枚举清理前全部 15 份 YAML，并通过 Python `yaml.safe_load` 语法读取；清理后复核保留的 10 份 YAML 及迁移后的 OpenAPI 文件。
- 检索文件名、关键字段、`YAML::LoadFile` / `YAML::Load` 及包装函数，并沿 `AppConfig` → `Robot` → 硬件 / 绑定 / 负载 / 坐标系追踪调用。
- 核对 CMake 目标是否包含相关解析器，以及测试读取路径和最终工作目录；区分源码声明、测试使用和默认启动使用。
- 使用已有 Release 构建产物执行以下离线测试，结果 **3/3 通过**：

```bash
ctest --preset release \
  -R '^(app_config_test|hardware_test|frame_registry_test)$' \
  --output-on-failure
```

其中 `app_config_test` 校验启动配置和路径解析，`hardware_test` 读取五份硬件样例，`frame_registry_test` 校验临时坐标系文件的读写行为。它们不等于完整硬件启动验证；本次未运行机器人主程序、上使能或运动集成测试，也未确认实际 EtherCAT PDO 名称与现场设备是否一致。对于“未使用”的结论，证据来自当前仓库的静态调用链，而不是仅凭某一次测试未触发加载。
