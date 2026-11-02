# Pinocchio / pin_ik 最小迁移实现与验证

更新日期：2026-11-03。本文记录实际实现，取代[前期评估](pinocchio_backend_migration_assessment.md)中尚未落地的分层、可选后端和运行时回退设想。

## 实现范围

按“先隔离类型、验证 KDL，再替换计算后端”的顺序完成迁移。保留一个 `ModelInterface` 和一个 `Model`，增加一个私有 `Impl` 隐藏第三方对象；没有增加后端工厂、插件系统或第二套机器人模型描述。

| 部分 | 实际改动 |
| --- | --- |
| Talon URDF | 七个活动关节由 `continuous` 改为 `revolute`。原位置、速度、力矩限制移入标准 `<joint><limit>`，数值不变；删除 `<hardware>` 扩展块，硬件配置统一由 `hardware.yaml` 管理 |
| 自有类型 | [math_types.hpp](../src/math_types.hpp) 定义 `Vector`、`Rotation`、`Frame`、`Twist`、`Wrench`、`JntArray`、`Jacobian`，保留业务层使用的构造、成员和操作形式 |
| 类型存储 | 使用项目已有的 Eigen 数学类型；自有类型不再是 KDL 别名，也不暴露 Pinocchio 对象。没有额外复制一套矩阵库 |
| 模型入口 | [model.hpp](../src/model.hpp) 对外只使用 ROCOS 类型，删除暴露 KDL 链的 `GetChain()`；调用者用 `GetJointNum()` / `GetJointNames()` 获取必要信息 |
| 后端 | [model.cpp](../src/model.cpp) 内部持有完整 Pinocchio Model/Data，负责 FK、Jacobian、RNEA 逆动力学、ABA 正动力学 |
| IK | pin_ik 接收已经构建的 Pinocchio Model、位置限位、tip frame，仅执行 `CartToJnt`；不使用 pin_ik 的 URDF 加载器或内部模型提供动力学 |
| 构建 | 移除活动构建中的 orocos-kdl、TRAC-IK、kdl_parser 依赖；pin_ik 复用现有 NLopt，静态链接入 robot |

URDF 是共同的机器人描述格式，没有新增“Pinocchio URDF”格式。`Model` 根据 base/tip 提取串联链，将保留惯量和固定连杆的 URDF 交给 Pinocchio 构建模型，再把该模型交给 pin_ik。pin_ik 内部复制模型是其工作区实现细节。

业务层改动主要是把 `KDL::` 类型调用改为 `rocos::`。HTTP/JSON、Lua、UDP 的序列化字段没有随 C++ 类型迁移改名；本次核查了在线 Apifox 的相关定义。C++ ABI 已改变，依赖 `librobot` 的客户端必须重编译；原先依赖 `GetChain()` 或 KDL 原始存储布局的源码需要调整。

旧 `3rdparty/trac_ik`、`3rdparty/kdl_parser` 和未参加构建的 `src/dh_params_loader.*` 暂留仓库，未继续实现第二套运行后端。当前活动目标和库入口不依赖它们。后续若恢复 DH 工具，应另行迁移其 KDL 接口。

## 坐标、索引与限位

- 采用 SI 单位。`Frame` 表示 `T_base_tip`，把 tip 坐标变换到 base；旋转九元素构造按行排列，四元数参数顺序为 xyzw。`diff(a,b)` 的角速度增量在共同参考系表达。
- ROCOS 关节向量按 base→tip 活动关节顺序排列，固定关节不占一列。Talon 的七个 revolute 关节满足 `nq = nv = 7`。其他模型的真正 continuous 关节仍以单个角度向调用者提供，由私有实现转换成 Pinocchio 的 cos/sin 配置；不能把 Pinocchio 的 `nq` 当作驱动器数量。
- Jacobian 是 **tip 原点、base 坐标轴表达**的几何 Jacobian，前三行为线速度、后三行为角速度，对应 `LOCAL_WORLD_ALIGNED`。
- `Wrenches` 按 base→tip 的每个子连杆排列，**包括固定连杆**；力和力矩在该连杆系表达，力矩关于连杆原点。空数组表示无外力；非空数组长度必须与子连杆数一致。实现先变换至对应父关节局部坐标并累加，保留固定连杆产生的力臂。
- 模型位置、速度、力矩限制只读取标准 URDF joint limit，`<hardware><limit>` 不再覆盖它们。硬件 YAML 的驱动保护参数仍由原硬件层管理；本次没有重新定义硬件保护与轨迹参数。acc/jerk 不属于标准 URDF joint limit。
- FK/动力学可以计算限位外状态；IK 必须返回限位内解。pin_ik 成功后还检查维度、有限数值、每个关节限位，以及独立 FK 的位置和姿态残差（各不超过 `2e-5 m` / `2e-5 rad`）。失败不覆盖调用者输出。

支持固定基座串联链中的 revolute、continuous、prismatic 和 fixed 关节，允许 base 不是 URDF 根节点，且 base 必须是 tip 的祖先。不支持 mimic、floating、planar 或跨分支运动链。base 之外的固定安装姿态不计入链内 FK，重力向量需在所选 base 系表达；链内固定连杆惯量仍参与动力学。

## 并发和实时边界

同一个 `Model` 的 FK/Jacobian/动力学共用 Data，通过一个互斥锁串行访问；pin_ik 使用独立求解器和互斥锁。IK 完成后验证 FK 时会短暂使用计算锁。没有为每次请求重建模型，常用 Pinocchio 工作缓冲在初始化时分配。

模型重新加载、限位修改等配置操作仅在初始化或停止计算后使用；不能与业务持有的限位引用并发。重新加载失败保留原模型。pin_ik 采用上游默认 `Speed` 策略、5 ms 搜索预算和 `1e-5` 求解精度；线程调度、锁等待、创建工作线程及独立 FK 验证不包含在该预算保证内，因此不是硬实时的最坏时延承诺。此次没有测量实时周期抖动或证明性能优于 KDL。

## 固定依赖与构建

- Pinocchio **3.9.0**，提交 [`ed3bb75ce96cf26e84aebd8b73785407950a0f1f`](https://github.com/stack-of-tasks/pinocchio/tree/ed3bb75ce96cf26e84aebd8b73785407950a0f1f)。通过 [3rdparty/pinocchio](../3rdparty/pinocchio/) Git submodule 引入。CMake 使用 EXACT 约束；构建其源码需要 CMake 3.22+。
- pin_ik **2.2.0**，提交 [`553fd1f1fc3bad594af3f4b3214535707f83c58b`](https://github.com/thinkexist1989/pin_ik/tree/553fd1f1fc3bad594af3f4b3214535707f83c58b)。通过 [3rdparty/pin_ik](../3rdparty/pin_ik/) Git submodule 引入。App 在子模块外用 [pin_ik.cmake](../cmake/pin_ik.cmake) 集成目标，复用现有 NLopt；不修改上游源码或 CMake。

在仓库根目录执行（系统仍需 Eigen3、Boost、urdfdom、TinyXML/TinyXML2、yaml-cpp、LibArchive、OpenSSL 等现有构建依赖）：

```bash
git submodule update --init 3rdparty/rocos-shm 3rdparty/pinocchio 3rdparty/pin_ik
bash cmake/build_pinocchio.sh build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/build/deps/prefix"
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure \
  -E '^(robot_fsm_test|lua_script_engine_test)$'
./build/bin/rocosAppMain --config=config/app.yaml --check_config
```

主仓库 gitlink 固定两个依赖的提交。脚本直接使用 `3rdparty/pinocchio`，检查提交、初始化其 cmake 子模块，关闭 Python、碰撞支持、示例和 Pinocchio 自身测试，安装到指定 App 构建目录下的 `deps/prefix`，不安装到系统。首次改用子模块时，旧 `deps/pinocchio-build` 的 CMake 缓存仍指向原下载路径，需要移走该构建目录后重新构建；`deps/prefix` 可以复用。也可提供自行安装的相同版本 Pinocchio；使用 preset 时需要把该 prefix 加入对应 CMake 配置。

DEB 工作流使用相同步骤，依赖放在 `build/deb-release/deps/prefix`，让现有打包器收集 Pinocchio 动态库；打包步骤同时复制其许可证。构建目录之外的自定义动态库路径仍会被打包器拒绝。此次未执行远程 CI、制作或安装 DEB；发布流程仍需在干净发行环境验证。

## 构建脚本的保留理由

[cmake/pin_ik.cmake](../cmake/pin_ik.cmake) 和 [cmake/build_pinocchio.sh](../cmake/build_pinocchio.sh) 都是 App 自己维护的构建逻辑，放在根目录 `cmake/`；`3rdparty/pin_ik`、`3rdparty/pinocchio` 保持为未修改的上游子模块。

目前保留 `build_pinocchio.sh`：

- Git submodule 固定和获取源码，不会生成可供 `find_package(pinocchio 3.9.0 EXACT REQUIRED CONFIG)` 使用的安装包。App 配置前仍需先构建、安装 Pinocchio。
- 脚本统一本地和 CI 的构建选项、版本检查及安装前缀，避免在多个位置重复维护命令。
- `deps/prefix` 位于 App 构建目录内，现有 DEB 打包器才能收集这些私有动态库；脚本还准备打包步骤读取的许可证文件。

它是源码依赖的构建辅助工具，不参与 App 运行。开发者已安装兼容版本时可跳过脚本，直接通过 `CMAKE_PREFIX_PATH` 指定；当前 DEB 流程仍使用脚本约定的目录。如果以后统一采用预编译依赖包或独立的 CMake 依赖构建工程，再移除脚本并同步调整 CI 和打包。仅为了减少一个脚本而引入新的构建层，目前没有必要。

## 实际验证结果

环境：Linux x86_64、GCC 11、CMake 3.22.1、Release、Eigen3、Boost 1.74。本机缺少 LibArchive 开发头文件，本次把发行版开发包解压到 `build/deps/usr`，通过 `LibArchive_INCLUDE_DIR` 指定；正常安装 libarchive-dev 的环境无需此项。

| 阶段 | 结果 | 证据/覆盖 |
| --- | --- | --- |
| 自有类型 + KDL/TRAC-IK 后端 | 完整构建成功，离线 CTest **79/79** 通过 | `build/validation/kdl-build.log`、`kdl-tests.log`；阶段源码快照 `kdl-stage-source.tar.gz` |
| Pinocchio + pin_ik 后端 | 完整构建成功，离线 CTest **79/79** 通过 | `build/validation/pinocchio-build.log`、`pinocchio-tests.log` |
| 配置离线检查 | 通过 | 默认 Talon 配置、URDF、七个关节与七个驱动器绑定，未连接硬件 |
| 链接检查 | 通过 | `librobot.so` 依赖 Pinocchio parsers/default 和现有 NLopt，不依赖 KDL 或 TRAC-IK |

数值对照使用在第一阶段真实 KDL 后端生成的 [talon_kdl_reference.json](../test/data/talon_kdl_reference.json)，涵盖四组确定性 q/v/a、非零末端外力、FK、Jacobian、含重力逆动力学、纯重力和正动力学。FD 使用记录的 KDL 力矩输入，不仅做新后端自身的往返测试。比较容差分别为 FK/Jacobian `1e-10`、力矩 `1e-9`、加速度 `1e-8`（使用 doctest::Approx 的相对/小量容差规则）。

新增测试还覆盖解析式单摆惯量/重力/固定末端外力、固定连杆惯量合并、非根 base、Jacobian 有限差分、真正 continuous 与 prismatic 关节、URDF 标准限位优先、无效数值和维度、不覆盖失败输出、加载失败保留原模型、限位附近与不可达 IK、40 组确定性 Talon 近邻初值 IK、同模型多线程 FK/IK。自有几何类型覆盖坐标组合、逆、旋转小角/π、四元数双覆盖、行序构造和 Jacobian 分量顺序。

已有硬件配置解析测试引用了已从生产 `config/` 删除的旧示例，本次将所需五份 YAML 恢复为 `test/data/hardware/` 内的专用夹具，避免依赖不存在的文件；它们不用于启动配置。

79 个 CTest 项目包含第三方测试。未运行 `robot_fsm_test`、`lua_script_engine_test`：它们会创建真实 Robot 并触发使能流程，需准备硬件或仿真环境。离线通过不代表实机运动、冗余解全局成功率、奇异位形连续性、5 ms 最坏时延或力控稳定性已经验收；后续上机前仍需做这些验证。

阶段日志和源码快照位于被忽略的 `build/validation/`，仅作为本次本机证据；持久化回归依据是仓库中的测试与 KDL 数值夹具。本次提交按维护者明确选择继续当前内测周期，版本为 `3.2.0-rc.3`；上述 C++ 接口和 ABI 不兼容仍然存在，调用方需要重编译并适配。不创建标签或发布包。
