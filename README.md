<!--
 Copyright (c) 2021 'Yang Luo, luoyang@sia.cn'

 This software is released under the MIT License.
 https://opensource.org/licenses/MIT
-->

<div align="center">
  <img src="./rocos-app.png" alt="" height="150">
  <h1>ROCOS-Application</h1>
  <blockquote>The Application of Robotics for ROCOS </blockquote>
</div>
:warning: Rocos-App is under development, not finished~ :thinking:

版本遵循 [SemVer 2.0.0](https://semver.org/lang/zh-CN/)。当前处于面向 `3.1.0` 的内部测试阶段，使用 `3.1.0-rc.N`，测试稳定并确认后正式发布 `3.1.0`。实际版本以 `CMakeLists.txt` 为准；维护规则见 [版本维护 skill](.agents/skills/rocos-version-maintenance/SKILL.md)。

## Introduction

ROCOS-App is the application of robotics, which is designed as the robot controller for ROCOS. 



## 工具管理与六点法标定

已接入 Viz 的真实工具管理、六点采集与标定计算。工具保存到 `robot.frames` 指定的 YAML，支持重命名、激活与重启恢复；标定计算后需明确保存并单独激活。操作顺序、坐标约定和接口衔接见 [六点法标定与工具管理](docs/tool_calibration.md)。

## Dependencies

- orocos-kdl
- ruckig
- nlopt
- trac_ik
- Eigen3
- protobuf
- grpc
- tinyFSM



## Usage

### CMake Presets

使用预设需要 **CMake 3.21+** 和 **Ninja**，以及项目原有的编译依赖。根目录的 `CMakePresets.json` 提供以下配置：

| 预设 | 构建类型 | 用途 |
| --- | --- | --- |
| `debug` | Debug | 日常开发、断点调试 |
| `release` | Release | 优化构建、性能测试 |
| `relwithdebinfo` | RelWithDebInfo | 优化构建并保留调试符号 |
| `minsizerel` | MinSizeRel | 优先减小产物体积 |

在仓库根目录执行，以下以 `debug` 为例，替换预设名即可切换：

```bash
cmake --list-presets
cmake --preset debug
cmake --build --preset debug --parallel 2
ctest --preset debug

# 只构建一个目标或运行指定测试
cmake --build --preset debug --target rocosAppMain --parallel 2
ctest --preset debug -R '^move_joint_test$'
```

测试前先构建相应测试目标；仅构建 `rocosAppMain` 不会生成测试程序。`--parallel` 可按机器资源调整，测试预设默认输出失败详情。硬件使能和机器人运动集成程序仍需在明确准备好的硬件/模拟器环境中手动运行。

各配置独立使用 `build/<预设名>/`，可执行文件、动态库、编译数据库分别位于其下的 `bin/`、`lib/`、`compile_commands.json`。CMake 不再将 YAML、URDF 或脚本复制到 `bin/`。

支持 CMake Presets 的 IDE 可直接选择上述配置。个人编译器路径或额外缓存变量可放入根目录 `CMakeUserPresets.json`，继承仓库预设；该文件已加入 Git 忽略规则。

未使用预设时，原有 CMake 3.15+ 的手动方式仍可使用：`cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug`，随后执行 `cmake --build build --parallel 2`。

### 启动配置

`rocosAppMain` 的部署默认入口为 `/opt/rocos/config/app.yaml`。源码运行显式选择仓库中的 [config/app.yaml](config/app.yaml)，示例为 Talon 七轴机械臂、ECM 主站、无安装工具。

```bash
# 离线检查：解析配置、URDF 和关节绑定，不连接硬件
./build/release/bin/rocosAppMain --config=config/app.yaml --check_config
# 导出有效快照以及供独立主站使用的配置，然后退出
./build/release/bin/rocosAppMain --config=config/app.yaml --export_config=/tmp/talon-config
# 准备好对应主站或仿真环境、快照目录权限后启动
./build/release/bin/rocosAppMain --config=config/app.yaml
```

全部发行配置与模型资源位于 `config/`：`robots/talon/` 保存型号、硬件、ECM、URDF、mesh 和 ENI；`devices/talon_001/` 保存设备安装与辨识引用；`tools/` 保存工具包。硬件的 `joint_name` 直接绑定关节，不再使用独立 `joint_binding.yaml`。

每个路径相对**声明它的 YAML 文件**，用户覆盖中的路径相对 `user.yaml`。优先级为内置默认值 → 基础文件 → 单份 `user.yaml` → 显式 HTTP 命令行参数。详细字段、部署权限、覆盖示例和独立主站的兼容边界见 [配置说明](config/README.md)。


## Related Projects



## Contributor

:bust_in_silhouette: **Yang Luo (luoyang@sia.cn)**

:framed_picture: Icon Designed by :**Yuhan Ying (1027649507@qq.com)**
