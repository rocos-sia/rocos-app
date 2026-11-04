# Dynamics ID：整机辨识与 Hardware 阻抗验证

这两个独立程序直接连接新版 EtherCAT v2 共享内存，不创建 Robot、Executor 或 HTTP 服务。
Hardware 内部仍负责 CiA 402 上下使能；程序显式调用 `WaitForSignal()` 同步周期。
这两个入口为 Hardware 构造函数传入周期等待超时；其他调用方默认仍保持无限等待行为。
运行时应由该独立程序独占所选驱动器的目标 PDO，不能同时运行另一个控制这些轴的 App。

## 文件与工作流程

本目录完成整机激励采集、辨识模型接入和阻抗验证，不涉及负载辨识。

| 文件 | 作用 |
| --- | --- |
| [dynamics_id_main.cc](dynamics_id_main.cc) | CSP 激励执行，记录实测 `q/dq/tau` |
| [excitation_trajectory.hpp](excitation_trajectory.hpp) | 读取并计算 Studio 的 Fourier 轨迹 |
| [export_impedance_model.py](export_impedance_model.py) | 将 Studio 的 `Pβ` 导出为 C++ 可加载的 JSON |
| [identified_joint_impedance.cpp](identified_joint_impedance.cpp) | 辨识刚体模型、摩擦补偿与 PD 阻抗计算 |
| [joint_impedance_main.cc](joint_impedance_main.cc) | CST 启动、周期力矩下发与诊断记录 |
| [experiment_common.hpp](experiment_common.hpp) | 关节绑定、反馈检查、周期同步和退出处理 |

```text
Studio 生成轨迹 → Hardware 执行与采集 → 实测 CSV
→ Studio 辨识 → 导出 Pβ 和摩擦模型 → joint_impedance 计算 Nm
→ Hardware 转换并下发
```

以下构建和运行命令均从仓库根目录执行。

## 构建

```bash
cmake -S . -B build
cmake --build build --target dynamicsIdMain jointImpedanceVelocityMain -j2
```

`identification_hardware` 复用 `src/hardware.cpp`；两个入口不链接 `librobot`。
`IdentifiedJointImpedanceController` 从旧 `gs_dyn` 分支的 `joint_impedance` 迁入，
与 App 自带的 `JointImpedanceController` 使用不同名称，保留辨识模型与摩擦补偿算法。

## 单位与映射

- 位置 rad，速度 rad/s，力矩按关节侧 Nm 接口约定。
- `SetPosition(q)` 内部转换码值并加零点；`GetJointVelocity()` 不减位置零点。
- CSV 的 `tau_i` 直接来自 `GetJointTorque()`，不再乘 `ratedTorque/1000`。
- 控制器输出 Nm，直接传给 `SetTorque()`，不在外部转电流。
- `torque_per_unit` 使用 Hardware YAML 中的现有值，程序不标定或覆盖它。
  真正的 Nm 量纲依赖该值正确，当前迁移不代表标定完成。
- 以关节名称匹配硬件 ID，使用 `SetJointBinding()` 绑定模型顺序。
  整机实验要求模型/轨迹覆盖配置中的所有驱动器，避免批量使能额外轴。

位置和速度使用不同的零点规则，当前速度修复为：

```text
位置读取：(position_raw - offset_pos_cnt) / cnt_per_unit / ratio
速度读取：velocity_raw / cnt_per_unit / ratio
位置下发：q_rad * cnt_per_unit * ratio + offset_pos_cnt
```

`GetVelocity()` 批量调用 `GetJointVelocity()`，因此同样不减位置零点。
这里按已约定的驱动速度反馈为码值/秒使用。回归测试覆盖非零零点下的静止、正转和反转，
并检查位置读取仍扣除零点。

Hardware YAML 与模型 URDF 分离，不再读取 URDF 中旧版 `<hardware>` 标签。
实验程序验证必需 PDO 确实存在，避免 Hardware 对缺失反馈返回零而被当作实测。
当前 Hardware 未提供模式显示读取接口；程序写模式并等待周期后使能，
这不等于核实了驱动器的 `6061` 模式反馈。设备 PDO/模式配置仍须与 CSP/CST 对应。

## 激励采集

输入 Studio 导出的 `trajectory.json`。也接受 `trajectory.csv` 路径，实际读取同目录 JSON。
执行保存的 sin⁴ 包络 Fourier 函数，不重新优化轨迹，不使用参考导数代替实测速度。

```bash
# 只解析文件、检查轨迹和硬件配置，不连接共享内存。
./build/bin/dynamicsIdMain --check_only \
  --hardware=/absolute/path/hardware.yaml \
  --trajectory=/absolute/path/trajectory.json

# 已准备好的 EtherCAT 主站或兼容仿真主站；会切换 CSP 并使能。
./build/bin/dynamicsIdMain --id=0 \
  --hardware=/absolute/path/hardware.yaml \
  --trajectory=/absolute/path/trajectory.json \
  --loops=4 --time_scale=1 \
  --output=/absolute/path/measurements.csv
```

顺序为：确认下使能 → CSP → 预置当前位置 → 使能确认 → 五次平滑过渡到中心 →
稳定 → 激励和采集 → 结束保持 → 下使能 → 写盘。
`SetPosition()` 不等待周期，所以等待由主循环显式完成。
轨迹推进使用实际时间；`time_scale` 范围 `(0,1]`，参考导数按其相应幂次缩放。
CSV 中的时间与反馈始终是实际值，导入 Studio 时不要再次应用该缩放。

主列：`time,loop,q_1,dq_1,tau_1,...`，顺序等于轨迹 `joint_names`。
速度来自驱动器反馈；Studio 再从实测速度估计加速度。
采集前预分配固定容量缓存，运行中不写 CSV；失败时下使能后保存已有样本。
旁路文件 `<output>.meta.yaml` 记录关节映射、原始换算参数、标称周期和完成/失败状态。
输出以排他方式创建，不覆盖已有实验。

`--tracking_error` 默认 0.2 rad，比较反馈与上一条位置命令。
`--max_cycle_gap` 默认 0.020 s。时间断流、状态异常、超限或 SIGINT/SIGTERM 都结束采集并请求下使能。
若主站完全停止交换 PDO，程序无法确认物理驱动器已下使能，会报告确认失败。
`--check_only` 采用 1 ms 网格预检，连接后还会按实际总线周期复检，运行时也逐点检查目标。
该网格检查不宣称是整个连续轨迹的数学极值证明。

## Studio 与模型导出

Studio 选择 CSV 的 `tau_i` 力矩列，配置 rad、rad/s 和秒，不启用电流输入换算。
导出整机辨识结果 `identification.json`、`identification.npz` 和 `friction_model.json`。

```bash
# 需要 Python 3 与 NumPy；也可使用 Studio 的 Python 环境。
python3 examples/dynamics_id/export_impedance_model.py \
  /absolute/path/identification_run \
  /absolute/path/rigid_model.json
```

### export_impedance_model.py 的作用

Studio 的辨识输出使用基参数 `β`，并保存对应的参数基矩阵 `P`。
该工具在离线阶段计算 `rigid_parameters = P @ β`，使 C++ 无需引入 NumPy NPZ 解析依赖。
控制器启动时加载 JSON，周期内计算：

```text
rigidTorque = Y(q, dq, 0) * rigid_parameters
            = Y(q, dq, 0) * P * β
```

`Pβ` 是参与回归计算的参数向量。脚本不重新辨识、不做各连杆物理惯量恢复，
也不写入或修改 URDF；该向量不应直接当作一套已验证物理可行的连杆惯性配置。

设关节数为 `n`、基参数数为 `r`，输入和输出如下：

| 文件 | 读取/写入内容 |
| --- | --- |
| 输入 `identification.json` | 收敛状态、关节名称、URDF 指纹、`base_parameters` 与参数数量 |
| 输入 `identification.npz` | `P`（`10n × r`）与 `base`（`r` 维） |
| 输出 `rigid_model.json` | `schema`、`urdf_hash`、`joint_names`、`rigid_parameters`（`10n` 维） |

输出文件名可自定义。输出父目录需已存在；脚本采用排他创建，已有文件不会被覆盖。
输入必须来自同一次辨识结果，脚本检查：

- 辨识已收敛，关节名称非空且不重复。
- `P`、`β` 形状正确且数值有限。
- JSON 中的 `β` 与 NPZ 中的 `base` 一致。
- `P` 的列满足正交归一条件。

`friction_model.json` 不经过该脚本，直接使用同次 Studio 结果。
原始 URDF 也直接保留。阻抗配置中的模型与硬件路径分别指向：

```yaml
urdf: /absolute/path/original_robot.urdf
rigid_model: /absolute/path/rigid_model.json
friction: /absolute/path/identification_run/friction_model.json
hardware: /absolute/path/hardware.yaml
```

完整配置还需要增益与限制数组，见[配置模板](../../config/identified_impedance.yaml)。
控制器启动时验证 URDF 指纹、关节顺序和限位。当前支持有界单自由度 revolute 模型，
不支持浮动基座、mimic 或负载辨识模型。

### 常见导出问题

| 提示/现象 | 处理 |
| --- | --- |
| 找不到 `numpy` | 使用安装了 NumPy 的 Python 环境，例如 Studio 的环境 |
| `Identification has not converged` | 检查辨识结果的收敛情况，不修改结果标志绕过检查 |
| `Invalid or mismatched Studio P/beta/joint names` | 核对 JSON/NPZ 是否来自同一次任务，是否完整 |
| 输出文件已存在 | 指定一个新的输出文件名 |
| 控制器报告 URDF hash mismatch | 使用辨识时的同一份原始 URDF |

## 阻抗验证

复制并填写 `config/identified_impedance.yaml`，其中模型文件路径相对于该 YAML。
模板没有附带实际辨识参数；需要提供原始 URDF、导出的刚体 JSON 和摩擦 JSON。

```bash
./build/bin/jointImpedanceVelocityMain \
  --config=/absolute/path/identified_impedance.yaml --check_only

# 会切换 CST、预置补偿力矩并使能，保持启动姿态 5 秒。
./build/bin/jointImpedanceVelocityMain --id=0 \
  --config=/absolute/path/identified_impedance.yaml \
  --duration=5 --output=/absolute/path/impedance.csv
```

控制公式：`Y(q,dq,0) Pβ + friction_scale * friction + Kp(qDesired-q) - Kd*dq`。
初始目标为当前位置；先计算并检查初始补偿力矩，在下使能时切换 CST、写入初值，再上使能。
每周期使用同一驱动速度反馈计算刚体、摩擦、阻尼以及速度保护。
幅值限制为 Nm，变化率限制为 Nm/s，变化率使用实际周期秒数。

`--duration=0` 持续到信号退出；`--max_cycle_gap` 默认 0.020 s。
固定容量诊断缓存保留最后 512 个周期，停机后写盘，包含实测 q/dq/tau、目标、
刚体与摩擦分量、请求和下发力矩。`computed/written` 标记本周期是否完成计算/写 PDO，
`written` 不代表驱动器已执行命令；未完成计算时分量列不应解释为有效模型结果。
记录中断期间，`sent` 保留上一成功写入的命令。

控制器接口支持实测温度，但此保持程序尚无温度反馈接线。
带非零温度斜率的模型可以离线检查，在线执行会在连接前拒绝，不能用常温冒充实测温度。
旧 `--sim`、HTTP、`rated_torque` 和 Robot 配置参数不再使用；仿真使用兼容 v2 主站的实例 ID。

## 离线验证

```bash
cmake --build build --target identified_hardware_test identification_execution_test shared_memory_config_test -j2
ctest --test-dir build -R '^(identified_hardware_test|identification_execution_test|dynamics_id_check_test|identified_impedance_check_test|shared_memory_config_test)$' --output-on-failure
python3 test/identified_export_test.py
```

测试使用独立共享内存实例模拟 PDO/状态字，不连接真实机器人。
包括 Studio 数值参考、关节顺序、CSP 采集、CST 预置、力矩比例、非零位置偏移、缺失 PDO 和断流停机。
