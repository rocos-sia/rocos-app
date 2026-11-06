# Mimic 从动关节：状态、硬件绑定、运动规划与约束推导

整理日期：2026-11-05（北京时间）。

代码基线：`v3` 分支，`b64ae64f097dcc97d1fa0d9e5b42084fb7255ea4`。本文整理本次讨论中的问题、源码行为、公式推导和待解决问题，不代表文中问题已经修复，也不代表已完成实机验证。

文中“从动关节”指 URDF `<mimic>` 关节。必须区分“由其他关节决定位置”与“没有电机”：一个 mimic 关节可以配置独立驱动器，也可以仅通过机械机构被动运动。

## 1. 问题与结论速查

| 问题 | 当前实现的结论 |
| --- | --- |
| 从动关节状态怎么处理？ | 有驱动的从轴保留自己的反馈槽位，位置、速度、力矩读取自身 PDO；模型计算时按主轴重建理想从轴位置。无驱动从轴不产生独立硬件反馈槽位 |
| 底层关节数量会不会包含从轴？ | `Model::GetJointNum()` 包含模型中的非 mimic 活动关节和有驱动 mimic；不包含无驱动 mimic。这不是独立自由度数，也不是 URDF 全部关节数 |
| 从轴影响使能吗？ | 只要配置成硬件驱动，就参与使能、禁用和状态汇总；无驱动从轴不参与 |
| 从轴会下发 PDO 吗？ | 有对应驱动和目标位置 PDO 时会。设置主轴位置会同步写从轴目标位置；直接给从轴独立位置会被忽略 |
| 真实硬件与 URDF 绑定时会忽略从轴吗？ | 模型层先排除无驱动 mimic，有驱动 mimic 必须保留绑定。运行时自动绑定另有 drive ID 与数组下标混用问题，见第 4 节 |
| MoveJ 是主动＋从动的维数吗？ | 是“独立关节＋有驱动从动关节”的指令向量维数。当前 RCM 是 8 维向量、6 个独立变量 |
| 一条标量进度曲线能维持 mimic 吗？ | 对固定倍率、固定偏置的线性 mimic，只要起终点满足约束且共享同一进度，整个插值轨迹就满足约束 |
| MoveL 如何保证 mimic？ | 正常离线轨迹逐点调用耦合 IK；IK 在独立坐标内求解，每次 FK 都展开从轴，并使用合并联动贡献后的 Jacobian |
| 这些改动是什么时候加入的？ | MoveJ 标量进度插值：2026-07-03；mimic 支持及 MoveJ 起终点联动处理：2026-11-04，见第 10 节 |
| 数学正确是否等于真实执行可靠？ | 不等于。逆解收敛、连续性、速度限制、暂停恢复积分、硬件跟随误差和使能失败回退需要分别验证 |

## 2. 三种维数与当前 RCM 配置

### 2.1 模型、独立坐标和指令槽位

| 符号 | 含义 | 当前 RCM |
| --- | --- | --- |
| $q_{\mathrm{full}}\in\mathbb R^n$ | 纳入计算模型的全部标量活动关节，包括无驱动从轴 | $n=8$ |
| $u\in\mathbb R^r$ | 非 mimic 的独立关节坐标 | $r=6$ |
| $q_{\mathrm{cmd}}\in\mathbb R^m$ | 对外控制向量，保留有驱动从轴槽位 | $m=8$ |

这里的 $n$ 是标量关节坐标数量，不应直接等同于 Pinocchio 的 `nq`。例如 continuous 关节在 Pinocchio 配置中可能使用 cos/sin 两个分量，但 ROCOS 对外仍以一个角度表示。

当前 RCM 的 URDF 声明：

```text
joint_6 = joint_5
joint_7 = -joint_5
```

而 [RCM 硬件配置](../config/robots/rcm/hardware.yaml) 仍给 `joint_6`、`joint_7` 分别配置了驱动 ID 5、6。因此当前 App 将其视为“有驱动 mimic”。这说明配置语义，不证明真实机构一定安装了这两个电机。

如果实际只存在 6 个电机，应让硬件配置反映真实驱动，同时保留 URDF 中的机械从动关系。此时模型仍有 8 个活动关节，独立坐标和指令向量均为 6 维；修改配置时还需处理第 4 节的运行时绑定问题。

RCM 的 MuJoCo 模型另有一个区别：保留了 8 个执行器槽位，但两个从轴执行器 `kp=0、kv=0`，通过 equality 约束被动联动。因此“App 写了从轴槽位”与“仿真执行器实际施加驱动力”不是同一件事。见 [rcm.xml](../config/robots/rcm/mjcf/rcm.xml)。

### 2.2 计数的源码依据

`Robot` 初始化时先读取硬件 `joint_name` 列表，再传给 `Model`。模型保留控制槽位的条件为：

```cpp
joint->type != urdf::Joint::FIXED &&
    (!joint->mimic || driven.count(joint->name))
```

因此，无驱动 mimic 在进入绑定流程之前就已从控制关节名称列表中排除，并不是绑定失败后再尝试跳过。

硬件批量位置操作使用 `ModelJointCount()`：绑定完成后取绑定表长度，否则取硬件驱动配置数量。使能则遍历全部已配置驱动，不依赖 IK 的独立变量数。

证据：[Robot 初始化](../src/robot.cpp#L626)、[模型槽位筛选](../src/model.cpp#L523)、[硬件计数与 ID 映射](../src/hardware.hpp#L377)。

## 3. 真实反馈、模型状态与 PDO 下发

### 3.1 有驱动从轴的反馈没有被主轴值替换

`Hardware::GetPosition()`、`GetVelocity()`、`GetTorque()` 等按模型槽位找到真实 drive ID，再读取该驱动自身 PDO 并换算单位。没有相应反馈 PDO 时，相关读取函数可能返回 0，不能把这种返回值当作已验证的实测零值。

状态快照保留这些反馈。但是 FK、Jacobian 和动力学中的位置/速度计算会提取独立主轴坐标，重新展开理想从轴状态。这意味着：

```text
关节位置反馈：可以显示从轴实际偏差
法兰/TCP 模型位姿：按理想 mimic 关系计算
```

不能因为模型位姿正确，就判断真实从轴已经同步。见 [反馈读取](../src/hardware.cpp#L357)、[状态快照](../src/robot.cpp#L749)、[模型配置展开](../src/model.cpp#L131)。

### 3.2 有驱动从轴确实会收到位置 PDO

`Hardware::SetJointPosition()` 的实际逻辑可简化为：

```text
直接写从轴位置：
    忽略该次独立设置

写主轴位置：
    写主轴 target_position
    对每个有驱动从轴计算 multiplier × 主轴目标 + offset
    写这些从轴的 target_position
```

位置联动在用户单位中计算，再分别使用每个驱动自己的换算参数转为编码器值。因此偏置 $b$ 是关节位置单位中的偏置，不是可以直接加到任意驱动原始计数上的值。

“忽略从轴独立输入”不等于“不向从轴发 PDO”。只有未配置驱动的 mimic 才会在硬件联动配置时被跳过。有驱动但缺少目标 PDO 时，写函数不会完成该项输出；配置是否符合真实设备仍需验证。

证据：[ConfigureMimicJoints](../src/hardware.cpp#L807)、[SetJointPosition](../src/hardware.cpp#L484)。

### 3.3 速度和力矩不能直接套用位置联动结论

`SetJointVelocity()`、`SetJointTorque()` 没有同样的主从覆盖逻辑。耦合动力学会考虑惯量和力矩映射，但这不等于已经实现速度/力矩模式下的主从同步伺服。关节阻抗控制仍逐槽使用实际位置、速度计算反馈项。

见 [硬件速度/力矩写入](../src/hardware.cpp#L501)、[关节阻抗控制](../src/joint_impedance_controller.cpp#L172)。

### 3.4 对外逐关节 status 的已有问题

`Robot::getJointStatus()` 当前恒定返回 0，状态快照和 Lua `GetJointStatus` 使用该函数，因此没有透传真实逐轴驱动状态。普通轴和从轴都受影响。

2026-11-05 已通过 Apifox MCP 刷新并读取在线“ROCOS API”及状态相关资源：

- `/paths/_api_robot_state.json`
- `/components/schemas/JointState.json`
- `/components/schemas/HardwareState.json`

在线契约规定 `JointState.status` 为 `0=disabled、1=fault、2=enabled`；`HardwareState.state` 为 `0=DISABLED、1=ENABLED、2=ERROR`。这两套枚举顺序不同。以后修复逐轴状态时必须显式映射，不能直接强转内部 `JntState`。

这是实现与在线契约的差异；本文不是新的 API 定义。见 [getJointStatus](../src/robot.hpp#L351)、[HTTP 序列化](../src/robot_http_server.cpp#L888)、[内部枚举](../src/types.hpp#L27)。

## 4. URDF 与真实硬件绑定：设计行为和发现的问题

### 4.1 哪些从轴需要绑定

设计上：

- 无驱动 mimic 不在 `Model::GetJointNames()` 的控制列表里，不需要驱动绑定。
- 有驱动 mimic 保留槽位，必须与对应驱动绑定。
- 模型列表中的每个关节都必须有合法、唯一的 drive ID。
- 联动源必须能够映射到被控制的独立关节。

新结构使用硬件 `joint_name` 自动绑定；旧的显式 `joint_binding.yaml` 和缺失文件时的顺序回退分支仍然存在，不能把所有配置路径视为同一个实现。

### 4.2 运行时自动绑定混用了 drive ID 和数组下标

当前 `Robot::jointBinding()` 的自动绑定分支执行：

```cpp
for (const auto id : hardware_drive_ids) {
    const auto name = hardware->getJointName(id);
    // 将 name 绑定到 id
}
```

但 `Hardware::getJointName(int32_t model_index)` 将参数解释为模型/数组下标，绑定尚未建立时按硬件配置列表顺序取元素。

例如移除 RCM 两个无实际电机的从轴驱动条目，并保留其他真实 ID 后：

```text
驱动 ID：0, 1, 2, 3, 4, 7
驱动数量：6
```

处理 ID 7 时会调用 `getJointName(7)`，因下标越界返回空名称，导致运行时绑定失败。驱动 ID 连续但配置列表乱序时，也可能将错误名称绑定到某个 ID。

这个问题不是 mimic 数学造成的，但移除从轴配置后可能暴露它。应直接使用配置中的 `drive.joint_name → drive.id`，或建立语义明确的按 drive ID 查询接口；不应为了绕过下标错误而随意修改真实设备 ID。

证据：[自动绑定调用](../src/robot.cpp#L548)、[名称查询实现](../src/hardware.cpp#L758)、[未绑定时按配置下标取 ID](../src/hardware.hpp#L383)。

`AppConfig::validateResources()` 直接使用 `drive.joint_name` 和 `drive.id`，并未经过上述名称查询。这解释了为什么配置校验通过，不足以证明实际 `Robot` 初始化绑定也正确。见 [配置校验](../src/app_config.cpp#L713)。

## 5. MoveJ 的维数、标量进度和约束证明

### 5.1 MoveJ 使用哪一种维数

MoveJ 输入、起点和输出使用对外控制向量维数 $m$：

| 配置 | MoveJ 向量维数 | 独立变量数 |
| --- | --- | --- |
| 6 个独立驱动＋2 个无驱动 mimic | 6 | 6 |
| 6 个独立驱动＋2 个有驱动 mimic | 8 | 6 |

当前 RCM 属于第二种。不能因为机构只有 6 个独立变量，就向当前 8 槽位的 MoveJ 传入一个 6 维目标。

`Robot::MoveJ()` 先对目标调用 `ApplyPositionCoupling()`，在准备轨迹时再对反馈起点应用相同关系，然后创建 `MoveJoint`。从轴槽位中的有限输入值会被联动结果覆盖；输入维度和有限性检查仍然存在。

`MoveJoint` 使用一条标量进度曲线驱动全部槽位：

$$
q(t)=q_0+s(t)(q_1-q_0),\qquad s\in[0,1].
$$

见 [MoveJ 入口](../src/robot.cpp#L1246)、[统一插值](../src/move_joint.cpp#L186)。

### 5.2 为什么共同进度能保证线性 mimic

设主轴为 $q_m$，从轴满足：

$$
q_f=a q_m+b.
$$

如果起点和终点满足：

$$
q_{f0}=a q_{m0}+b,\qquad q_{f1}=a q_{m1}+b,
$$

则使用相同 $s(t)$ 插值时：

$$
\begin{aligned}
q_f(t)
&=q_{f0}+s(t)(q_{f1}-q_{f0})\\
&=a q_{m0}+b+s(t)\left[a q_{m1}+b-a q_{m0}-b\right]\\
&=a\left[q_{m0}+s(t)(q_{m1}-q_{m0})\right]+b\\
&=a q_m(t)+b.
\end{aligned}
$$

对时间求导还可得到：

$$
\dot q_f=a\dot q_m,\qquad
\ddot q_f=a\ddot q_m.
$$

矩阵形式同样成立。若两个端点分别为 $Au_0+b$、$Au_1+b$，则：

$$
(1-s)(Au_0+b)+s(Au_1+b)
=A\left[(1-s)u_0+su_1\right]+b.
$$

因此，满足仿射约束的两个端点之间做共同线性插值，仍留在约束空间中。条件是固定 $a,b$、起终点合法且使用同一个 $s(t)$。独立选择不同的轴进度曲线，或采用一般非线性从动关系时，不能直接使用这个证明。

### 5.3 从轴仍可能影响 MoveJ 的时间

当前 `MoveJoint` 按全部槽位的最大绝对位移归一化速度、加速度和 jerk：

$$
\Delta_{\max}=\max_i|q_{1i}-q_{0i}|,
$$

$$
v_s=\frac{v_{\mathrm{limit}}}{\Delta_{\max}},\quad
a_s=\frac{a_{\mathrm{limit}}}{\Delta_{\max}},\quad
j_s=\frac{j_{\mathrm{limit}}}{\Delta_{\max}}.
$$

有驱动从轴倍率绝对值大于 1 时，较大的从轴位移可能成为最大项，从而拉长轨迹时间。RCM 的两个倍率为 $+1$、$-1$，它们的位移绝对值与源轴相同。

这是一套使用公共标量限制的实现，不应理解为已完成所有轴独立动力学约束下的最优时间规划。对于混合移动/转动关节，米和弧度的尺度选择也值得明确。见 [归一化计算](../src/move_joint.cpp#L101)。

### 5.4 规划约束与实际起点之间的边界

当前代码把反馈起点转换为理想联动起点，没有单独生成主从纠偏轨迹。如果真实从轴偏离较大，规划起点会与从轴真实位置不同。

位置控制器在尚无上一周期指令、且反馈不满足联动时，会使用实际反馈作为速度检查基线。因此首次下发可能返回 `SpeedLimit`。后续速度检查主要比较相邻指令，并不是持续的主从同步误差监测。

见 [首次位置检查](../src/position_controller.cpp#L155)。

## 6. MoveL 约束逆运动学的完整推导

### 6.1 用独立坐标消去联动约束

对 RCM 选择：

$$
u=\begin{bmatrix}q_1&q_2&q_3&q_4&q_5&q_8\end{bmatrix}^{T}.
$$

完整活动关节向量为：

$$
q_{\mathrm{full}}=Au+b,
$$

$$
A=\begin{bmatrix}
1&0&0&0&0&0\\
0&1&0&0&0&0\\
0&0&1&0&0&0\\
0&0&0&1&0&0\\
0&0&0&0&1&0\\
0&0&0&0&1&0\\
0&0&0&0&-1&0\\
0&0&0&0&0&1
\end{bmatrix},\qquad b=0.
$$

一般 mimic 的 multiplier 对应 $A$ 中的系数，offset 对应 $b$。多级 mimic 可以合成到最终非 mimic 源轴。例如：

$$
q_2=a_2u+b_2,\qquad q_3=a_3q_2+b_3
$$

可展开为：

$$
q_3=a_3a_2u+(a_3b_2+b_3).
$$

循环引用和缺失源关节无法形成合法映射，当前加载逻辑会拒绝。见 [mimic 链解析](../src/model.cpp#L43)。

原始逆解问题可写为：

$$
FK(q)=T_d,\qquad q_6-q_5=0,\qquad q_7+q_5=0.
$$

替换为独立坐标后，只需求：

$$
\boxed{FK(Au+b)=T_d.}
$$

如果一般线性约束写作 $Cq=d$，一个合法参数化应满足 $CA=0$、$Cb=d$。代入任意 $u$ 都有 $C(Au+b)=d$，因此约束从求解器的待优化条件变成了参数化的恒等性质。

这是该方法最优美的地方：每一个候选解、每一个中间迭代点都满足联动，不需要给“末端误差”和“联动误差”设置竞争的惩罚权重。满足约束的状态构成一个仿射空间；逆解始终在这个空间内搜索。

### 6.2 完整模型与驱动向量的映射不同

为了区分无驱动从轴，可以再定义驱动映射：

$$
q_{\mathrm{cmd}}=Bu+c.
$$

$B,c$ 只取有控制槽位的关节，对应代码的 `actuation`、`active_offset`；$A,b$ 包含全部参与计算的活动关节，对应 `expansion`、`offset`。

再用选择矩阵 $R$ 从对外向量中提取非 mimic 源轴：

$$
u=Rq_{\mathrm{cmd}},\qquad RB=I,\qquad Rc=0.
$$

无驱动从轴虽然不出现在 $q_{\mathrm{cmd}}$ 中，仍然通过 $Au+b$ 参与 FK。硬件反馈不满足约束时，$Rq_{\mathrm{actual}}$ 是信任源轴的坐标提取，并不是综合所有编码器读数求出的最优估计，也不是欧氏意义下的正交投影。

### 6.3 Jacobian 的链式法则

完整模型的几何 Jacobian 描述末端速度：

$$
V=J_{\mathrm{full}}(q)\dot q,
$$

其中 $V$ 是六维 twist；其坐标系必须与 Jacobian 的定义一致。

由于 $A,b$ 为常量：

$$
\dot q=A\dot u.
$$

代入可得：

$$
V=J_{\mathrm{full}}(Au+b)A\dot u.
$$

所以独立坐标下的 Jacobian 为：

$$
\boxed{J_{\mathrm{ind}}(u)=J_{\mathrm{full}}(Au+b)A.}
$$

令完整 Jacobian 的列为：

$$
J_{\mathrm{full}}=
\begin{bmatrix}J_1&J_2&J_3&J_4&J_5&J_6&J_7&J_8\end{bmatrix}.
$$

RCM 的独立 Jacobian 就是：

$$
\boxed{
J_{\mathrm{ind}}=
\begin{bmatrix}
J_1&J_2&J_3&J_4&J_5+J_6-J_7&J_8
\end{bmatrix}.
}
$$

原因是 $u_5$ 增加一点时，$q_5$、$q_6$ 同向增加，$q_7$ 反向变化，三处运动对末端的贡献必须合并。删除从轴驱动槽位不等于删除它的运动学贡献。

虽然 $b$ 的导数为零，它仍通过 $Au+b$ 改变 Jacobian 的求值姿态，不能在 FK 中忽略。当前耦合 IK 使用局部坐标系 `LOCAL` 下的 `full_jacobian * expansion`；公开 `GetJacobian()` 则使用 `LOCAL_WORLD_ALIGNED`，调用者不能混用相应的 twist。见 [耦合 IK Jacobian](../src/model.cpp#L217)、[公开 Jacobian](../src/model.cpp#L321)。

### 6.4 位姿误差与逆解迭代

定义当前位姿和相对目标变换：

$$
T(u)=FK(Au+b),\qquad E(u)=T(u)^{-1}T_d.
$$

用刚体变换的对数构造六维误差：

$$
e(u)=\operatorname{Log}(E(u))^{\vee}\in\mathbb R^6.
$$

$\vee$ 表示把李代数矩阵整理为向量。后三个分量表示旋转误差，前三个为对应的李代数平移分量；在较大旋转误差下，前三个分量并不简单等于世界坐标下的位置差。

当前代码用几何 Jacobian 构造局部修正，近似写作：

$$
e(u+\Delta u)\approx e(u)-J_{\mathrm{ind}}\Delta u.
$$

令 $J=J_{\mathrm{ind}}$，希望求出满足 $J\Delta u\approx e$ 的修正。阻尼最小二乘问题是：

$$
\min_{\Delta u}\ \|J\Delta u-e\|^2+\lambda\|\Delta u\|^2.
$$

其解可写作：

$$
\boxed{\Delta u=J^T(JJ^T+\lambda I)^{-1}e.}
$$

第一项减小末端误差，第二项抑制过大的关节步长。代码通过线性方程求解而不是显式计算矩阵逆；这里用逆矩阵记号说明数学关系。

当前 `solveCoupled()` 使用：

- 固定阻尼 $\lambda=10^{-8}$。
- 最大绝对关节步长超过 0.5 时整体缩放。
- 按独立关节限位裁剪候选值。
- 最多 10 次回溯减半，要求真实残差下降。
- 最多 4 组初值、每组最多 150 次迭代，并受总计约 5 ms 搜索预算限制。

代码在每次候选计算中都通过 $Au+b$ 展开从轴。因此即使某一步没有逼近目标，也不会把 mimic 当成独立变量破坏其关系。见 [solveCoupled](../src/model.cpp#L183)。

### 6.5 几何 Jacobian 与严格误差 Jacobian 的区别

对于上述误差定义和局部 Jacobian，严格的一阶误差导数为：

$$
\boxed{
H(u)=\frac{\partial e}{\partial u}
=-\operatorname{Jlog6}(E^{-1})J_{\mathrm{ind}}.
}
$$

严格误差线性化为 $e(u+\Delta u)\approx e(u)+H\Delta u$，相应的阻尼步骤是：

$$
\Delta u=-H^T(HH^T+\lambda I)^{-1}e.
$$

[Pinocchio 官方 IK 示例](https://gepetto.github.io/doc/pinocchio/doxygen-html/md_doc_b-examples_d-inverse-kinematics.html) 使用了 `Jlog6` 修正。当前 ROCOS 耦合求解器没有这一项。

当误差很小时，`Jlog6` 接近单位阵，当前近似合理。有阻尼、较大位姿误差或秩不足时，不能把它直接视为严格的误差最小二乘步骤。某些无阻尼且任务可完全满足的情形，两种表达可能给出等价修正；不能仅凭缺少这一项就断言每次逆解会失败。

这是收敛性和任务度量方面的改进点，不是 mimic 几何关系失效。当前回溯检查仍会验证真实残差是否下降。

此外，平移与旋转误差的量纲不同，独立变量还可能混合米和弧度。固定阻尼、统一步长 0.5 和未加权残差都隐含了尺度选择。更明确的形式是：

$$
\min_{\Delta u}\quad
\|W_e^{1/2}(H\Delta u+e)\|^2
+\lambda\|W_u^{1/2}\Delta u\|^2,
$$

其中 $W_e$、$W_u$ 表示任务误差和关节修正的尺度/权重。它们属于进一步设计，当前代码没有实现上述一般加权形式。

### 6.6 限位如何折算

若从轴为：

$$
q_f=-0.5u+0.2,
$$

且位置限制为：

$$
-0.1\le q_f\le0.4,
$$

代入并注意负倍率导致不等号方向变化：

$$
-0.1\le-0.5u+0.2\le0.4
\quad\Longrightarrow\quad
-0.4\le u\le0.6.
$$

最终将这一范围与主轴自身范围以及其他同源从轴折算范围取交集。如果交集为空，模型拒绝加载或更新限位。倍率为零时，从轴固定在偏置位置，只需验证该位置是否满足限位。

速度限制同理：

$$
|\dot q_f|=|a|\,|\dot u|
\quad\Longrightarrow\quad
|\dot u|\le\frac{v_{f,\max}}{|a|},\qquad a\ne0.
$$

上述例子若 $v_{f,\max}=0.4$，得到 $|\dot u|\le0.8$。

当前实现包含位置和速度限制的折算；不能据此推断已经实现逐轴加速度、jerk、力矩和碰撞约束的完整轨迹优化。见 [位置限位](../src/model.cpp#L146)、[速度限位](../src/model.cpp#L614)。

## 7. MoveL 的实际执行路径与保证边界

### 7.1 正常执行先逐点求解，再回放关节轨迹

当前 `Robot::MoveL()` 使用 `MoveLineOffline`：

```text
当前反馈 → 理想联动 FK → 起始法兰位姿
目标位姿 → 工具偏置换算 → 目标法兰位姿
笛卡尔路径进度 → 平移线性插值、旋转 Slerp
逐点耦合 IK → 保存完整控制向量 → 执行时回放
```

对于每个采样目标 $T_k$，求解：

$$
FK(Au_k+b)=T_k,
$$

再生成对应驱动向量 $q_{\mathrm{cmd},k}=Bu_k+c$。前一点逆解作为下一点初值。任何一点 IK 失败，会清空轨迹并返回规划错误。

每次 IK 成功后仍检查各槽位位置限位，并重新 FK 验证位置和姿态残差，分别不超过 `2e-5 m` 和 `2e-5 rad`。内部迭代退出使用的六维误差阈值为平移、旋转分量范数分别 `1e-5`；内部对数误差和最终 FK 检查不应混为同一计算。

位置控制器下发前还会应用硬件联动关系并检查位置/指令速度。这是额外约束执行层，正常 IK 本身已经考虑 mimic，并非先把所有槽位当独立轴求解，再靠覆盖从轴补救。

证据：[MoveL 入口](../src/robot.cpp#L1341)、[离线逐点 IK](../src/move_line_offline.cpp#L316)、[IK 最终复核](../src/model.cpp#L400)、[位置下发](../src/position_controller.cpp#L128)。

### 7.2 逐点存在逆解不保证连续、低速或全局可达

两个相邻采样点分别满足目标位姿，并不限制：

$$
\frac{u_{k+1}-u_k}{\Delta t}.
$$

临近奇异点或切换逆解分支时，这个速度可能很大。暖启动可以帮助保持连续，但求解器存在重启初值，不能据此证明永不跳解。

当前 MoveL 离线循环主要检查每点 IK 成功；执行阶段位置控制器再检查指令速度。因此可能出现“离线全部 IK 通过，执行时触发速度限制”。完整轨迹验收还需检查连续性、速度、加速度、jerk，以及必要的碰撞约束。

局部迭代失败也不证明目标一定不可达：失败可能来自初值、限位裁剪、奇异性、回溯停滞或搜索预算耗尽。

### 7.3 独立变量数不等于实际局部运动能力

末端的局部运动能力取决于：

$$
\operatorname{rank}(J_{\mathrm{ind}}).
$$

$6\times6$ 的 Jacobian 不保证满秩。RCM 的 $J_5+J_6-J_7$ 可能在某些姿态下出现贡献抵消，反映机构的真实运动约束或奇异性。阻尼可以抑制数值放大，但不能创造机构没有的自由度。

### 7.4 对外槽位形成的虚假零空间

当前公开 Jacobian 为：

$$
J_{\mathrm{out}}=J_{\mathrm{full}}AR.
$$

对 RCM，相当于：

$$
J_{\mathrm{out}}=
\begin{bmatrix}
J_1&J_2&J_3&J_4&J_5+J_6-J_7&0&0&J_8
\end{bmatrix}.
$$

两个从轴槽位是零列，因为模型函数从这些输入槽位中不提取独立位置。若直接把这个 $6\times8$ 矩阵用于零空间分析，会得到“只改变从轴槽位而末端不动”的方向，但这些方向不满足物理允许的切向关系 $\Delta q_{\mathrm{cmd}}=B\Delta u$。

真正允许的运动零空间应在独立坐标中计算：

$$
\dim\ker(J_{\mathrm{ind}})=r-\operatorname{rank}(J_{\mathrm{ind}}).
$$

当前 `MoveSvdJog` 按对外槽位数减 6 计算零空间维数，会把 RCM 两个从轴槽位误当成额外维度。单独点动从轴的参考也会在位置下发时被主轴联动结果覆盖，不会反向驱动主轴。这些是静态代码审查发现，尚未做实机复现。

证据：[SVD 点动维数](../src/move_svd_jog.cpp#L66)、[普通点动向量](../src/move_jog.cpp#L264)、[公开 Jacobian](../src/model.cpp#L321)。

### 7.5 暂停、停止和恢复后采用积分路径

当前 `MoveLineOffline` 首次暂停或停止会切换到 Jacobian 速度积分；恢复后继续积分，不回到原先的离线轨迹回放。

```text
当前进度速度 → 笛卡尔 twist
公开 Jacobian 的 SVD 求解 → 关节速度
q_current += q_dot × dt
位置控制器重新应用 mimic → 下发
```

这个过程仍可通过最后的位置联动保持目标从轴关系，但有限步长积分可能导致末端偏离理想路径，并且没有逐点重做完整 IK/FK 目标残差检查。其内部完整槽位参考中的从轴还可能停留在旧值，最终由位置控制器重新生成；不应把内部中间向量直接当作全部满足约束的输出。

源码中“全轨迹 IK 已通过，因此 Jacobian 非奇异”的注释不是有效数学推论。位姿存在逆解，不保证该解处 Jacobian 满秩，也不保证最小奇异值足够大。

见 [速度求解](../src/move_line_offline.cpp#L203)、[积分输出](../src/move_line_offline.cpp#L371)。

### 7.6 理想模型不会证明真实驱动同步

实际反馈可表示为：

$$
q_{\mathrm{actual}}=Au+b+\varepsilon.
$$

$\varepsilon$ 可以来自跟随误差、机械弹性、间隙或零点标定差异。当前模型根据源轴重建理想从轴，忽略了从轴相对理想关系的偏差。偏差较小时，在一致的局部速度坐标约定下，末端误差一阶近似为：

$$
\delta x\approx J_{\mathrm{full}}\varepsilon.
$$

所以模型 IK/FK 准确，不等于真实末端准确。应将主从残差、持续时间和允许阈值作为独立监测项；当前位置联动本身不是这样的监测机制。

### 7.7 非线性机械约束不属于固定 mimic 映射

若真实关系是：

$$
q=f(u),
$$

则应使用：

$$
J_{\mathrm{ind}}=J_{\mathrm{full}}(f(u))\frac{\partial f}{\partial u}.
$$

加速度还包含：

$$
\ddot q=
\frac{\partial f}{\partial u}\ddot u+
\frac{d}{dt}\left(\frac{\partial f}{\partial u}\right)\dot u.
$$

固定 $A,b$ 的优点是映射恒定、导数简单、插值可保持约束；它的边界是只能表达相应的仿射关系。一般闭链机构如果不能准确化成线性 mimic，需要更完整的约束模型，不能仅凭 URDF 标签宣称已经处理全部闭链约束。

## 8. 使能判断：计数、成功条件与已有问题

### 8.1 使能只看硬件驱动列表

`SetEnabled()` 对全部配置驱动发布 `OperationEnabled` 目标；`GetState()` 也遍历全部配置驱动：

```text
任一驱动 ERROR       → 整体 ERROR
否则任一驱动 DISABLED → 整体 DISABLED
否则                 → 整体 ENABLED
```

对合法、非空硬件配置而言：

$$
\mathrm{IsEnabled}=
\bigwedge_{d\in\mathrm{configured\ drives}}
\left(d.\mathrm{state}=\mathrm{OperationEnabled}\right).
$$

这与 IK 的 $r$ 维、MoveJ 的 $m$ 维无关。没有配置驱动的被动从轴不会因出现在 URDF 中而增加使能要求；配置了驱动的 mimic 与其他驱动一样必须使能。

`GetJointState()` 从自身状态字解析驱动状态：OperationEnabled 为 ENABLED；Fault、FaultReactionActive、NA 或缺少状态字映射会被判为 ERROR；其他状态判为 DISABLED。实际 PDO 缺失也可能更早在初始化阶段暴露。

当前 RCM 配置要求 8 个驱动全部使能。如果真实只有 6 个电机，应先纠正配置和运行时绑定，而不是直接在使能循环中忽略所有 mimic。见 [使能与汇总](../src/hardware.cpp#L413)、[单轴判定](../src/hardware.cpp#L529)。

### 8.2 超时结果被丢弃，后台目标没有取消

`setDriverStates()` 批量发布目标后统一等待最多 150 ms。后台驱动状态机要求连续 3 次读到目标状态才标记切换成功；后台循环约每 10 ms 运行一次，控制字变更间隔至少 20 ms。

当前存在以下行为：

1. 超时返回 `false`，但 `Hardware::SetEnabled()` 返回 `void`，丢弃该结果。
2. 超时没有清除各驱动 `conductStateChange`，后台仍可能继续推进使能。
3. `Robot::on_fsm_enabling()` 随后通过当前状态字汇总再次判断成功，不直接使用“连续确认成功”的结果。

因此可能出现机器人已报告使能失败并进入 ERROR_STATE，慢驱动稍后却完成使能；也可能在超时边缘仅凭当前状态字通过判定。150 ms 是否足够还取决于实际驱动响应，不能只靠离线代码确定。

证据：[驱动状态推进](../src/hardware.cpp#L663)、[等待与超时](../src/hardware.cpp#L701)、[Robot 成功判定](../src/robot.cpp#L283)。

### 8.3 部分使能失败没有统一回退

某些轴成功、某个从轴失败时，整体 IsEnabled 为 false，机器人进入 ERROR_STATE。但是当前错误处理主要停止 motion，没有统一调用硬件下使能。

因此“整机使能失败”不代表“全部电机都未使能”。需要明确失败时取消未完成目标，并将已成功轴回退到什么状态；这属于待实现的失败处理策略，本文没有修改它。

见 [on_fsm_error](../src/robot.cpp#L395)。

### 8.4 使能成功不包含主从位置对齐

所有状态字都是 OperationEnabled 时，即使 $q_f\ne a q_m+b$，当前 IsEnabled 仍为 true。状态字判断只证明驱动使能状态，不能证明主从位置准备完成。

若产品需要“可以开始联动运动”的含义，应另设联动误差阈值和准备检查。不能把通过状态字检查、通过 IK 检查、通过位置下发检查视为同一个条件。

### 8.5 运行中硬件故障与机器人 FSM 尚未形成完整联动

从轴故障会让 `Hardware::GetState()` 变成 ERROR，`Robot::IsEnabled()` 变成 false。但当前 `RunCycle()` 主要根据 `executor->Update()` 返回值触发 FSM 错误；所检查的执行器和位置控制路径没有周期性依据硬件 ERROR 自动触发全轴停机的检查。

因此不能宣称“从轴报警必然使 App 自动进入 ERROR_STATE 并关闭所有驱动”。驱动器、主站或外部安全系统可能有自身响应，但不属于本文已经验证的 App 行为。

见 [RunCycle](../src/robot.cpp#L1212)、[Executor::Update](../src/executor.cpp#L41)。

## 9. 数学设计的优点与工程改进优先级

| 层次 | 已有优点或保证 | 尚不能据此保证 |
| --- | --- | --- |
| $q=Au+b$ | 仿射 mimic 恒等满足，减少独立变量，无需联动惩罚权重 | 任意非线性闭链关系、实际编码器同步 |
| $J_{\mathrm{ind}}=J_{\mathrm{full}}A$ | 合并主从共同运动的末端贡献 | 任意目标可达、Jacobian 非奇异 |
| 位置/速度限位折算 | 从轴限制可以约束源轴 | 完整动力学限制、全部轨迹速度都能通过 |
| 耦合 IK 与最终 FK 复核 | 成功采样点满足模型位姿与位置限制 | 全局可解性、逆解分支连续、实际末端精度 |
| MoveJ 共同进度 | 起终点合法时保持仿射关系 | 任意非线性联动、主从实测同步 |
| 全配置驱动使能汇总 | 有驱动从轴不会被错误豁免 | 失败回退、超时取消、联动位置就绪 |

建议按以下顺序处理，均属于后续工作：

1. 修复运行时硬件自动绑定的 ID/下标混用，并覆盖稀疏 ID、乱序列表、去除被动从轴后的初始化。
2. 使能接口传递切换结果，明确超时取消和部分成功回退，同时补全运行中故障响应。
3. 明确区分独立坐标、模型坐标和驱动槽位；零空间、冗余判断与内部运动求解使用独立坐标。
4. 离线验证轨迹连续性和运动限制，暂停恢复积分加入位姿误差校正或重新规划。
5. 评估严格误差 Jacobian、尺度权重、自适应阻尼和更完善的限位求解。
6. 修复逐轴 status 的 API 映射，并增加主从实测残差监测。

## 10. 相关改动时间

时间均为北京时间，由本次 `git blame` 和 `git show` 核对。

| 改动 | 提交时间 | 提交 | 作者 |
| --- | --- | --- | --- |
| MoveJ 公共标量进度与关节向量插值实现 | 2026-07-03 19:47:45 | `af9a5b48ca4b045a522621b9718c7c17d6827b6c`，`update` | cheng9911 |
| 有驱动/无驱动 URDF mimic 支持；MoveJ 起终点联动处理；耦合模型及位置指令支持 | 2026-11-04 21:12:26 | `43bf7575c8da6eb9d72fc23101019ac53d029aa5`，`feat: support driven and passive URDF mimic joints` | Yang Luo |

因此，共用标量进度并非此次 mimic 功能才引入；新增加的是在模型和控制链路中明确处理联动，特别是 MoveJ 规划前修正起点、终点。

## 11. 本次核查与验证范围

本次讨论期间没有修改控制源码，没有启动真实机器人，也没有提交 Git。本文是代码审查和数学解释的记录。

此前直接运行旧测试产物时，`model_test` 出现符号不匹配，`position_controller_test` 出现断言失败/崩溃。针对当前源码重编译相关目标后，四套测试全部通过：

```bash
cmake --build build --target model_test shared_memory_config_test position_controller_test app_config_test --parallel 2
ctest --test-dir build -R '^(model_test|shared_memory_config_test|position_controller_test|app_config_test)$' --output-on-failure
```

| 测试 | 重编译后结果 | 与本文相关的覆盖 |
| --- | --- | --- |
| `model_test` | 通过 | 有驱动/无驱动 mimic、FK/IK/Jacobian、限位、耦合动力学等 |
| `shared_memory_config_test` | 通过 | 模拟共享内存、硬件位置联动、映射顺序等 |
| `position_controller_test` | 通过 | 位置联动后检查、初始不一致时速度拒绝等 |
| `app_config_test` | 通过 | 配置层有驱动/被动 mimic 绑定校验 |

四套测试通过不代表本文发现的问题已被测试覆盖或修复。特别是配置校验与实际 Robot 初始化路径不同；未验证真实驱动使能超时、部分失败回退、运行中故障停机、主从不同步、暂停恢复轨迹精度及全工作空间奇异性。

相关背景：[Pinocchio 后端实现说明](pinocchio_backend_migration.md)、[配置说明](../config/README.md)、[Pinocchio 官方 IK 示例](https://gepetto.github.io/doc/pinocchio/doxygen-html/md_doc_b-examples_d-inverse-kinematics.html)。
