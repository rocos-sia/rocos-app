# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 概述

ROCOS-App 是 ROCOS（机械臂控制平台）的 C++17 机器人控制器库。它驱动伺服硬件（EtherCAT/CANOpen等等）或仿真、执行运动学/动力学计算、规划运动，并通过 HTTP/JSON REST API 对外暴露全部功能。目标平台为 Linux（Ubuntu 22.04，可选搭配 ROS2 Humble）。

## 构建与测试

项目使用 CMake，采用源外构建（out-of-source）至 `build/`：

```bash
cmake -S . -B build
cmake --build build -j

# 运行全部测试
cd build && ctest

# 直接运行单个测试可执行文件（基于 doctest）
./build/bin/unit_test
./build/bin/unit_interval_motion_profile_test

# 过滤到某一个 doctest 用例
./build/bin/unit_test --test-case="Hardware"

# 运行控制器（先准备配置所对应的硬件或仿真环境）
./build/bin/rocosAppMain --config=config/app.yaml
./build/bin/rocosAppMain --config=config/app.yaml --http_port=8080
```

运行配置直接从指定路径加载，不在构建时复制到 `build/bin/`。`--config` 指定启动 YAML，文件内相对资源路径以该 YAML 所在目录为基准；当前入口没有 `--sim` 参数。

测试使用 [doctest](test/doctest.h)（单头文件，已内置）。新增测试用例放入 [test/unit_test.cc](test/unit_test.cc)，或在 [CMakeLists.txt](CMakeLists.txt) 的 `## Testing ##` 段落下接入新的可执行目标。

## 架构

代码组织为一组分层的共享库（定义于 [CMakeLists.txt](CMakeLists.txt)），每个库都是一个 `rocos::` CMake 目标。依赖自下而上流动——上层链接下层：

- **logger** —— 基于 spdlog 的日志封装（[src/logger.cc](src/logger.cpp)）。
- **hardware** —— EtherCAT 抽象层。`HardwareInterface`（[include/rocos_app/hardware_interface.hpp](src/hardware_interface.hpp)）是抽象基类；`Hardware` 是真实 EtherCAT 驱动，`HardwareSim` 是仿真器。要新增后端，继承 `HardwareInterface` 即可。`ethercat/` 头文件（command、status_word、control_word、drive_state、mode_of_operation）建模 CiA 402 伺服驱动状态机。
- **drive** —— 单轴控制：`Drive` 封装一个伺服，`DriveGuard` 运行实时控制循环，另含插补与机器人数学运算。
- **kinematics** —— 通过 orocos-kdl 实现正/逆运动学（FK/IK），IK 求解器使用 trac_ik 和 nlopt。
- **dynamics** —— 通过 orocos-kdl 实现刚体动力学。
- **robot** —— 顶层。`Robot` 类（[include/rocos_app/robot.hpp](src/robot.hpp)）是核心 API：持有 `HardwareInterface*`、运动学链（从 URDF 解析）以及运动指令（MoveJ/MoveL/MoveC/MoveP、拖拽示教、通过 `WorkMode` 实现的阻抗控制）。`JC_helper_*` 提供运动规划辅助（基于 Ruckig 的轨迹生成）,Robot类中包含一个Boost::sml状态机，其状态机流转图在docs/fsm.png中
- **robot_http_server** —— 提供 HTTP/JSON 接口，`Robot` 类的友元类，传入Robot，实现RESTful API通信。

`rocos_app` 是一个 INTERFACE 目标，打包 hardware+drive+kinematics+robot 供下游使用。`rocosAppMain`（[src/rocosAppMain.cc](src/rocosAppMain.cpp)）是入口：构造硬件（仿真或真实）、一个 `Robot` 和一个 `RobotHttpServer`，然后运行服务器。

### HTTP API

ROCOS API 的唯一权威来源是 Apifox MCP 项目「ROCOS API」的在线 OpenAPI 文档。后续涉及 API 实现、联调、测试、文档或解释时，先刷新并读取在线版本，按需获取引用资源。所有本地 API 文件（包括 `docs/rocos-API.yaml`、设计文档、返回码表及历史记录）仅作参考副本，不作为接口定义依据。

Apifox 不可访问时，应明确说明无法核实最新接口定义，不得把本地文件当作权威替代。源代码可用于核实实际实现行为；实现与在线定义不一致时，应明确指出差异，不得据此擅自改写接口约定。完整规则见 [AGENTS.md](AGENTS.md#rocos-api-source-of-truth)。

HTTP/JSON 服务实现位于 [src/robot_http_server.cpp](src/robot_http_server.cpp)，使用 cpp-httplib 和 nlohmann/json。具体路由、字段、鉴权及返回码以 Apifox 在线版本为准；本地文档更新应与在线定义保持一致。

### 配置

[config/](config/) 是统一配置入口，包含 `app.yaml`、`user.yaml`、`robots/talon/`、`tools/` 和 `devices/talon_001/`。每个路径相对声明它的 YAML 文件；硬件 `joint_name` 直接绑定关节。使用与迁移边界见 [配置说明](config/README.md)。历史使用分析只作旧结构记录。

### 第三方依赖

[3rdparty/](3rdparty/) 以 `add_subdirectory` 方式内置了全部依赖：nlopt、trac_ik、kdl_parser、ruckig、plog、spdlog、gflags、sml（状态机）、gripper，外加头文件库 httplib 和 json。通过 `find_package` 查找的系统依赖：Boost、yaml-cpp、Eigen3、OpenSSL、orocos_kdl。

## 编码规范

摘自 [.github/copilot-instructions.md](.github/copilot-instructions.md)——新代码请遵循：

- **命名**：内部函数 `lowerCamelCase`，对外调用/API 函数 `PascalCase`，类/结构体 `PascalCase`，常量/宏 `UPPER_SNAKE_CASE`，成员变量 `lower_case_`（结尾下划线）。
- **风格**：4 空格缩进，禁用 Tab，遵循 Clang-Format 风格。注释说明*为什么*而非*做了什么*——仅在逻辑复杂、Hack 代码或算法关键点处添加。
- **仅限 C++17** —— 禁止 C++20 特性（concepts、ranges），也禁止 C++98 老旧语法。
- **内存**：禁止裸 `new`/`delete`；使用 RAII，优先 `std::unique_ptr`，仅在确有共享所有权时使用 `std::shared_ptr`。
- 虚函数重写标注 `override`，不修改状态的成员函数标注 `const`，不抛异常的函数标注 `noexcept`。传递大对象用 `const T&`。
- 禁止忽略返回值；对入参指针操作前用 `if (ptr == nullptr)` 防御性检查；通过项目的 `LOG_ERROR` 宏记录错误。
- 解释和注释用中文，技术术语用英文。生成核心业务函数时，应附带单元测试骨架（注意：copilot 规范要求 GTest 骨架，但现有测试套件使用的是 doctest）。
