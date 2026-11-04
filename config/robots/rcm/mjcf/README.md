# RCM MuJoCo 模型

`scene.xml` 包含机器人、地面和灯光，`rcm.xml` 可单独加载；网格引用上一级机器人目录的 `meshes/`。文件布局和场景外观参照 Talon，运动学、关节限位、质量和完整惯量张量取自当前 `RCM.urdf`。

## 加载

在 rocos-app 仓库根目录运行：

```bash
python3 -m mujoco.viewer --mjcf=config/robots/rcm/mjcf/scene.xml
# 使用已有 rocos-mujoco 仿真器时：
rocos_mujoco_sim --config config/robots/rcm/mjcf/mujoco.yaml
```

Python 中可用 `mujoco.MjModel.from_xml_path()` 加载模型，用 `mujoco.mj_resetDataKeyframe(model, data, 0)` 恢复 `home`。启动 YAML 的硬件映射指向 `../hardware.yaml`。

## 连接 rocos-app

先使用上述 `mujoco.yaml` 启动 rocos-mujoco，再从 rocos-app 仓库根目录运行：

```bash
cmake --build --preset release --target rocosAppMain --parallel 2
./build/release/bin/rocosAppMain --config=config/app_rcm.yaml --check_config
./build/release/bin/rocosAppMain --config=config/app_rcm.yaml
```

`app_rcm.yaml` 选择 `../robot.yaml` 对应的 RCM 型号和 `rcm_sim` 设备，末端为固定传感器连杆 `Link_9`。App 的 `master_type: mujoco` 使用 `../master.yaml` 描述共享内存连接，ID=0、周期=1000 μs，必须与仿真器一致。该配置需要支持 `mujoco` 类型的新版 App，因此先重新编译；App 不负责启动仿真器。两端使用同一份 `hardware.yaml`，每个共享内存 ID 只运行一个 App 控制进程。

`../master.yaml` 及 App 导出的 `mujoco.yaml` 都是 App 连接参数；仿真器启动仍使用本目录的 `mujoco.yaml`。`--check_config` 仅验证配置与模型，不检查正在运行的共享内存。

初始移动关节位置为 0.03 m，比下限 0.02 m 留出 10 mm 余量，避免重力下沉导致 App 启动时读到限位外位置。修改模型后须重启 rocos-mujoco；运行中的实例不会自动重载 XML。

此初始姿态已完成 5 秒物理保持检查：全部关节位于限位内，无 MuJoCo 警告或接触穿透；三组相同关节坐标下的连杆变换与修改前一致。独立共享内存 ID=49 的无界面仿真与 App 联调通过，8 个驱动和 1 个力传感器加载成功，App 进入未使能的 IDLE 并启动 HTTP 服务；测试进程随后退出。用户已运行的 ID=0 实例需重启后采用新姿态。

## 关节与控制

保留 `base_link`、`Link_1` 至 `Link_9` 的名称和连接关系。joint_1、joint_3 为 `slide`，其余六个活动关节为 `hinge`；joint_9 以固定 body 表达。移动关节的 `ref=0.03` 配合 body 原点平移，使初始位置在限位内，同时保持关节坐标和 URDF 正运动学一致。

| ctrl 索引 | 执行器 | 关节 | 单位与行为 |
| --- | --- | --- | --- |
| 0 | actuator_1 | joint_1 | m，位置伺服 |
| 1 | actuator_2 | joint_2 | rad，位置伺服 |
| 2 | actuator_3 | joint_3 | m，位置伺服 |
| 3 | actuator_4 | joint_4 | rad，位置伺服 |
| 4 | actuator_5 | joint_5 | rad，位置伺服 |
| 5 | actuator_6 | joint_6 | 零增益占位，约束为 q6 = q5 |
| 6 | actuator_7 | joint_7 | 零增益占位，约束为 q7 = -q5 |
| 7 | actuator_8 | joint_8 | rad，位置伺服 |

两个 mimic 使用 joint equality 约束。保留 8 个执行器槽位以匹配现有硬件配置及仿真器的 DOF/ctrl 顺序；只需控制 6 个独立关节，从动关节的 ctrl 不施加驱动力。等式约束是数值约束，允许微小求解误差。直接写入 `qpos` 做姿态预览时，需要同时设置 q6/q7；`mj_forward` 本身不会投影到约束面。

伺服增益是针对当前 URDF 惯量设置的仿真参数。控制范围采用 URDF 的标准限位；位置执行器不自动实现 hardware.yaml 中的速度、加速度和 jerk 限制。URDF 的 effort=0 未转成零输出力限幅。

## 几何与动力学边界

- 原 URDF 的 base_link 和 Link_1 至 Link_7 均声明约 612.27 kg，模型总质量约 4922.51 kg。这里按原值保留，动力学精度取决于这些源参数。
- Link_9 引用的 `arm7.STL` 不存在，使用半径 0.03 m、高 0.064 m 的灰色圆柱占位，保留源质量和惯量；这不是实际传感器外形。
- Link_4 是凹形支架。`collision_meshes.xml` 保存由其原始 STL 经 CoACD 1.0.14 生成的 40 个凸碰撞块，原 STL 用于显示。碰撞块位于 group=3，且 rgba 的 alpha=0，避免 rocos-mujoco 默认开启全部 geom 分组时覆盖绿色视觉网格；透明度不影响接触计算。检查碰撞外形时，可临时将这些 geom 的 alpha 改为 1，并关闭 group=2、开启 group=3。其余现有 STL 使用 MuJoCo 默认凸包碰撞。碰撞几何均为近似；只排除相邻装配连杆，保留非相邻自碰撞和地面碰撞。
- CoACD 参数为 threshold=0.01、preprocess_resolution=60、resolution=1000、mcts_nodes=10、mcts_iterations=40、mcts_max_depth=3、max_convex_hull=40、decimate=true、max_ch_vertex=64、seed=0。40 块上限下最终 concavity 约 0.0562，未达到请求的 0.01；不应将该近似当作精密碰撞边界。
- 地面位于 z=-0.103 m，对齐基座 STL 的最低点附近；机器人基坐标保持在原点。geom 质量设为 0，body 使用显式惯量，避免从网格密度重复估算质量。

## 初版模型已执行验证（初始移动关节位置 0.02 m）

- MuJoCo 3.14.0：两个入口均加载成功；8 个活动关节、8 个执行器、2 条 mimic 约束。
- 三组关节姿态与 URDF 正运动学逐连杆对照，最大误差小于 1e-12；质量及完整惯量一致。
- 2 秒静止保持：最大移动关节误差约 0.352 mm，最大转角误差约 0.312°。
- 共 8 秒物理回放，目标为 `[0.12, 0.15, 0.12, 0.1, -0.3, -0.3, 0.3, 0.2]`：状态有限、无 MuJoCo 警告、无碰撞穿透；最大 mimic 误差约 1.42e-6 rad。初始状态及回放采样检查了相邻几何连续性和未声明的自穿透。此检查不覆盖全工作空间。
- MuJoCo 3.4.0 原生库：两个入口均加载并完成 100 步运行。
- Linux EGL 离屏渲染通过。未启动 EtherCAT 共享内存仿真器或连接实际硬件。
