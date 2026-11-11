# ROCOS-App HTTP REST API 测试与评估文档

## 1. 测试环境

### 1.1 基础要求

| 项目 | 要求 |
|------|------|
| 服务端 | `rocos-app --sim=true --http_host=0.0.0.0 --http_port=8080` |
| 客户端工具 | curl / Python requests / 浏览器 / 任意 HTTP 客户端 |
| 网络 | 默认 `localhost:8080`，仿真模式无需 EtherCAT 硬件 |
| 前置条件 | 机器人控制器已启动，处于仿真模式 |

### 1.2 测试工具

```bash
# 环境变量（所有测试共用）
export BASE_URL="http://localhost:8080"

# 快速连通性检查
curl -s $BASE_URL/api/robot/enabled | python3 -m json.tool
```

---

## 2. 功能测试用例

### 2.1 机器人状态查询（GET 类）

#### T-STATE-001: 获取机器人完整状态

```bash
curl -s $BASE_URL/api/robot/state | python3 -m json.tool
```

**预期：**
- HTTP 200，`success: true`
- `data` 包含 `joint_states` 数组（长度 = 关节数）
- 每个关节包含 `name`、`position`、`velocity`、`torque`、`load_torque`、`status`
- 包含 `flange`、`tool`、`object` 位姿（含 position + orientation）
- 包含 `hw_state`（hw_type、cycle_time、slave_num）

**边界检查：**
- 连续调用 10 次，每次 position 值应与硬件状态一致
- 并发 5 个请求同时查询，不应出现 segfault 或响应为空

---

#### T-STATE-002: 获取机器人信息

```bash
curl -s $BASE_URL/api/robot/info | python3 -m json.tool
```

**预期：**
- `data.joint_infos` 数组，每项含 `name`、`pos_per_unit`、`vel_per_unit`、`torque_per_unit`、`load_torque_per_unit`、`ratio`、`unit_name`、`zero_offset`
- 所有值为数值类型（非 null/空）

---

#### T-STATE-003: 查询使能状态

```bash
# 初始状态（刚启动）
curl -s $BASE_URL/api/robot/enabled

# 使能后
curl -s -X POST $BASE_URL/api/robot/enable
curl -s $BASE_URL/api/robot/enabled
```

**预期：**
- 初始返回 `data.enabled: false`
- 使能后返回 `data.enabled: true`

---

#### T-STATE-004: 获取 URDF 模型

```bash
curl -s $BASE_URL/api/robot/model | python3 -m json.tool
```

**预期：**
- `data.name` 为机器人名称
- `data.links` 数组，每项含 `name`、`order`、`type`（revolute/fixed 等）
- 有 visual 的 link 包含 `mesh` 字段

---

#### T-STATE-005: 获取 mesh 文件

```bash
# 先查模型中第一个 mesh 路径
MESH=$(curl -s $BASE_URL/api/robot/model | python3 -c "import sys,json; links=json.load(sys.stdin)['data']['links']; print(next(l['mesh'] for l in links if 'mesh' in l))")
curl -s -o /dev/null -w "%{http_code}" "$BASE_URL/api/robot/model/mesh?path=$MESH"
```

**预期：** HTTP 200，Content-Type 为 `application/octet-stream`

**边界检查：**
- 缺少 `path` 参数：`curl $BASE_URL/api/robot/model/mesh` → 返回 code 400
- 路径不存在：`curl "$BASE_URL/api/robot/model/mesh?path=nonexistent.stl"` → 返回 code 404

---

### 2.2 机器人控制

#### T-CTRL-001: 使能 / 禁用机器人

```bash
# 使能
curl -s -X POST $BASE_URL/api/robot/enable | python3 -m json.tool
curl -s $BASE_URL/api/robot/enabled

# 禁用
curl -s -X POST $BASE_URL/api/robot/disable | python3 -m json.tool
curl -s $BASE_URL/api/robot/enabled
```

**预期：**
- enable 返回 `success: true`，之后查询 `enabled: true`
- disable 返回 `success: true`，之后查询 `enabled: false`

---

#### T-CTRL-002: 设置工作模式

```bash
# 正常设置
curl -s -X POST $BASE_URL/api/robot/workmode \
  -H "Content-Type: application/json" \
  -d '{"mode": "position"}'

# 错误的模式字符串
curl -s -X POST $BASE_URL/api/robot/workmode \
  -H "Content-Type: application/json" \
  -d '{"mode": "invalid_mode"}'

# 缺少 mode 字段
curl -s -X POST $BASE_URL/api/robot/workmode \
  -H "Content-Type: application/json" \
  -d '{}'
```

**预期：**
- 正常设置返回 `success: true`
- 无效模式返回 `success: false, code: 400`
- 缺少字段返回 `success: false, code: 400`

---

### 2.3 运动控制（核心）

#### T-MOVE-001: 关节运动 MoveJ（同步）

```bash
# 先使能
curl -s -X POST $BASE_URL/api/robot/enable

# 获取当前位置作为起点
CURRENT=$(curl -s $BASE_URL/api/robot/state | python3 -c "
import sys, json
state = json.load(sys.stdin)
joints = [j['position'] for j in state['data']['joint_states']]
print(json.dumps(joints))
")

# 移动到当前位置 + 0.1 rad（小偏移，仿真安全）
TARGET=$(echo $CURRENT | python3 -c "
import sys, json
joints = json.load(sys.stdin)
print(json.dumps([round(j + 0.1, 4) for j in joints]))
")

curl -s -X POST $BASE_URL/api/move/joint \
  -H "Content-Type: application/json" \
  -d "{\"joints\": $TARGET, \"speed\": 0.5, \"acceleration\": 1.0}"
```

**预期：**
- `success: true`，code 0
- 运动完成后再次查询 state，关节位置发生变化

---

#### T-MOVE-002: 关节运动 MoveJ（异步 + 任务追踪）

```bash
# 异步发起运动
RESP=$(curl -s -X POST $BASE_URL/api/move/joint \
  -H "Content-Type: application/json" \
  -d '{"joints": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6], "speed": 0.3, "asynchronous": true}')

TASK_ID=$(echo $RESP | python3 -c "import sys,json; print(json.load(sys.stdin)['data']['task_id'])")
echo "Task ID: $TASK_ID"

# 立即查询状态（应该 RUNNING）
curl -s "$BASE_URL/api/move/status?task_id=$TASK_ID" | python3 -m json.tool

# 等待完成后再次查询（应该 COMPLETED）
sleep 5
curl -s "$BASE_URL/api/move/status?task_id=$TASK_ID" | python3 -m json.tool
```

**预期：**
- 异步响应含 `data.task_id`
- 首次查询 status = RUNNING
- 完成后 status = COMPLETED, result = 0

---

#### T-MOVE-003: 查询不存在的 task_id

```bash
curl -s "$BASE_URL/api/move/status?task_id=nonexistent_123" | python3 -m json.tool
```

**预期：** `success: false, code: 5001`（任务不存在）

---

#### T-MOVE-004: 直线运动 MoveL

```bash
curl -s -X POST $BASE_URL/api/move/linear \
  -H "Content-Type: application/json" \
  -d '{
    "pose": {
      "position": {"x": 0.4, "y": 0.1, "z": 0.3},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    "speed": 0.3
  }'
```

**预期：** `success: true`，法兰位姿发生变化

---

#### T-MOVE-005: 圆弧运动 MoveC

```bash
curl -s -X POST $BASE_URL/api/move/circle \
  -H "Content-Type: application/json" \
  -d '{
    "pose_via": {
      "position": {"x": 0.35, "y": 0.15, "z": 0.3},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    "pose_to": {
      "position": {"x": 0.4, "y": 0.1, "z": 0.3},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    "speed": 0.2
  }'
```

**预期：** `success: true`

---

#### T-MOVE-006: 停止运动

```bash
# 先发起一个长时间异步运动
curl -s -X POST $BASE_URL/api/move/joint \
  -H "Content-Type: application/json" \
  -d '{"joints": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5], "speed": 0.05, "asynchronous": true}'

# 立即停止
curl -s -X POST $BASE_URL/api/move/stop | python3 -m json.tool
```

**预期：** `success: true`，运动被中断

---

#### T-MOVE-007: IK 运动 MoveJ_IK

```bash
# 先获取当前法兰位姿
FLANGE=$(curl -s $BASE_URL/api/robot/state | python3 -c "
import sys, json
state = json.load(sys.stdin)
f = state['data']['flange']
p, o = f['position'], f['orientation']
# 沿 Z 轴偏移 0.05m
p['z'] += 0.05
print(json.dumps({'position': p, 'orientation': o}))
")

curl -s -X POST $BASE_URL/api/move/joint_ik \
  -H "Content-Type: application/json" \
  -d "{\"pose\": $FLANGE, \"speed\": 0.3}"
```

**预期：** `success: true`，法兰位姿改变约 0.05m

---

### 2.4 单轴/多轴控制

#### T-AXIS-001: 单轴运动

```bash
# 使能后移动第 0 轴
curl -s -X POST $BASE_URL/api/robot/enable
curl -s -X POST $BASE_URL/api/axis/single/move \
  -H "Content-Type: application/json" \
  -d '{"id": 0, "position": 0.5, "max_vel": 1.0, "max_acc": 2.0}'
```

**预期：** `success: true`

---

#### T-AXIS-002: 单轴 ID 越界

```bash
curl -s -X POST $BASE_URL/api/axis/single/move \
  -H "Content-Type: application/json" \
  -d '{"id": 99, "position": 0.5}'
```

**预期：** `success: false, code: 1002`

---

#### T-AXIS-003: 单轴使能/禁用/停止

```bash
curl -s -X POST $BASE_URL/api/axis/single/enable \
  -H "Content-Type: application/json" -d '{"id": 0}'

curl -s -X POST $BASE_URL/api/axis/single/stop \
  -H "Content-Type: application/json" -d '{"id": 0}'

curl -s -X POST $BASE_URL/api/axis/single/disable \
  -H "Content-Type: application/json" -d '{"id": 0}'
```

**预期：** 每个返回 `success: true`

---

#### T-AXIS-004: 多轴运动

```bash
curl -s -X POST $BASE_URL/api/axis/multi/move \
  -H "Content-Type: application/json" \
  -d '{
    "target_pos": [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
    "max_vel": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    "max_acc": [2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
    "max_jerk": [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
  }'
```

**预期：** `success: true`

---

### 2.5 拖拽示教

#### T-DRAG-001: 启动/停止拖拽

```bash
# 启动 J1 正方向拖拽
curl -s -X POST $BASE_URL/api/drag/start \
  -H "Content-Type: application/json" \
  -d '{"flag": "J1", "direction": "POSITIVE", "max_speed": 0.5, "max_acceleration": 1.0}'

sleep 0.5

# 停止拖拽
curl -s -X POST $BASE_URL/api/drag/stop | python3 -m json.tool
```

**预期：** 两次都返回 `success: true`

---

#### T-DRAG-002: 无效拖拽参数

```bash
curl -s -X POST $BASE_URL/api/drag/start \
  -H "Content-Type: application/json" \
  -d '{"flag": "INVALID_FLAG", "direction": "POSITIVE", "max_speed": 0.5, "max_acceleration": 1.0}'
```

**预期：** `success: false`

---

### 2.6 标定

#### T-CAL-001: 完整标定流程

```bash
# 1. 获取当前法兰位姿作为标定点
POSE=$(curl -s $BASE_URL/api/robot/state | python3 -c "
import sys, json
f = json.load(sys.stdin)['data']['flange']
print(json.dumps(f))
")

# 2. 设置 6 个标定点（仿真中用相同点）
for i in 1 2 3 4 5 6; do
  curl -s -X POST $BASE_URL/api/calibration/pose \
    -H "Content-Type: application/json" \
    -d "{\"id\": $i, \"pose\": $POSE}"
done

# 3. 执行工具标定
curl -s -X POST $BASE_URL/api/calibration/run \
  -H "Content-Type: application/json" \
  -d '{"frame": "tool"}'

# 4. 查询标定结果
curl -s $BASE_URL/api/calibration/result | python3 -m json.tool
```

**预期：** 最终返回标定结果（仿真中可能误差较大，返回 ErrorState: true）

---

#### T-CAL-002: 无效标定类型

```bash
curl -s -X POST $BASE_URL/api/calibration/run \
  -H "Content-Type: application/json" \
  -d '{"frame": "invalid"}'
```

**预期：** `success: false, code: 4004`

---

### 2.7 CORS 与 404

#### T-HTTP-001: OPTIONS 预检请求

```bash
curl -s -X OPTIONS $BASE_URL/api/move/joint -v 2>&1 | grep -i "access-control"
```

**预期：** 响应头包含 `Access-Control-Allow-Origin: *`

---

#### T-HTTP-002: 不存在的接口

```bash
curl -s $BASE_URL/api/nonexistent | python3 -m json.tool
```

**预期：** `success: false, code: 404, message: "endpoint not found"`

---

#### T-HTTP-003: 错误的 HTTP 方法

```bash
curl -s -X PUT $BASE_URL/api/robot/enable | python3 -m json.tool
```

**预期：** `success: false, code: 404`

---

## 3. 参数异常测试

### 3.1 JSON 格式错误

```bash
# 非 JSON
curl -s -X POST $BASE_URL/api/move/joint \
  -H "Content-Type: application/json" \
  -d 'not json at all'

# 空 body
curl -s -X POST $BASE_URL/api/move/joint \
  -H "Content-Type: application/json" \
  -d ''

# 语法错误
curl -s -X POST $BASE_URL/api/move/joint \
  -H "Content-Type: application/json" \
  -d '{"joints": [}'
```

**预期：** 全部返回 `success: false, code: 1001`，不崩溃

---

### 3.2 关节数据异常

```bash
# 关节数量不匹配
curl -s -X POST $BASE_URL/api/move/joint \
  -H "Content-Type: application/json" \
  -d '{"joints": [0.1, 0.2, 0.3]}'

# 关节值为字符串
curl -s -X POST $BASE_URL/api/move/joint \
  -H "Content-Type: application/json" \
  -d '{"joints": ["abc", 0.2, 0.3, 0.4, 0.5, 0.6]}'

# 关节值为 null
curl -s -X POST $BASE_URL/api/move/joint \
  -H "Content-Type: application/json" \
  -d '{"joints": [null, 0.2, 0.3, 0.4, 0.5, 0.6]}'
```

**预期：** 返回 `success: false`（1003 或 1001），不崩溃

---

### 3.3 位姿数据异常

```bash
# 缺少 orientation
curl -s -X POST $BASE_URL/api/move/linear \
  -H "Content-Type: application/json" \
  -d '{"pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.5}}}'

# 缺少 position
curl -s -X POST $BASE_URL/api/move/linear \
  -H "Content-Type: application/json" \
  -d '{"pose": {"orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}}'
```

**预期：** 返回 `success: false, code: 1004`，不崩溃

---

### 3.4 缺少必填字段

```bash
# MoveJ 缺少 joints
curl -s -X POST $BASE_URL/api/move/joint \
  -H "Content-Type: application/json" \
  -d '{"speed": 1.0}'

# MoveC 缺少 pose_to
curl -s -X POST $BASE_URL/api/move/circle \
  -H "Content-Type: application/json" \
  -d '{"pose_via": {"position": {"x":0,"y":0,"z":0}, "orientation": {"x":0,"y":0,"z":0,"w":1}}}'
```

**预期：** 返回 `success: false, code: 400`，不崩溃

---

## 4. 稳定性与压力测试

### 4.1 并发状态查询

```bash
# 使用 xargs 并发 20 个状态查询
seq 1 20 | xargs -P 20 -I {} curl -s -o /dev/null -w "{}: %{http_code} %{time_total}s\n" \
  $BASE_URL/api/robot/state
```

**预期：** 全部返回 200，无超时或崩溃

---

### 4.2 快速连续请求

```bash
# 100 次快速 enable/disable 交替
for i in $(seq 1 50); do
  curl -s -X POST $BASE_URL/api/robot/enable > /dev/null &
  curl -s -X POST $BASE_URL/api/robot/disable > /dev/null &
done
wait
echo "Done. Server should still be alive."

# 验证服务器存活
curl -s $BASE_URL/api/robot/enabled
```

**预期：** 服务器未崩溃，最后一个查询正常返回

---

### 4.3 异步任务并发

```bash
# 同时发起 5 个异步运动
TASK_IDS=""
for i in $(seq 1 5); do
  RESP=$(curl -s -X POST $BASE_URL/api/move/joint \
    -H "Content-Type: application/json" \
    -d '{"joints": [0.1, 0.1, 0.1, 0.1, 0.1, 0.1], "speed": 0.1, "asynchronous": true}')
  TASK_ID=$(echo $RESP | python3 -c "import sys,json; print(json.load(sys.stdin).get('data',{}).get('task_id','FAIL'))")
  echo "Task $i: $TASK_ID"
  TASK_IDS="$TASK_IDS $TASK_ID"
done

# 查询所有任务状态
for tid in $TASK_IDS; do
  curl -s "$BASE_URL/api/move/status?task_id=$tid" | python3 -c "
import sys, json
d = json.load(sys.stdin)
data = d.get('data', {})
print(f\"{data.get('task_id','?')}: {data.get('status','?')} (result={data.get('result','?')})\")"
done
```

**预期：** 所有任务状态可追踪，无 ID 冲突

---

### 4.4 长时间运行稳定性

```bash
# 持续 60 秒，每秒查询一次状态
for i in $(seq 1 60); do
  CODE=$(curl -s -o /dev/null -w "%{http_code}" $BASE_URL/api/robot/state)
  if [ "$CODE" != "200" ]; then
    echo "FAIL at second $i: HTTP $CODE"
    exit 1
  fi
  sleep 1
done
echo "60s stability test passed."
```

**预期：** 全部 200，无波动

---

### 4.5 大 Payload 测试

```bash
# 发送超大 joints 数组（1000 个元素，远超关节数）
HUGE_JOINTS=$(python3 -c "print(json.dumps([0.1]*1000))")
curl -s -X POST $BASE_URL/api/move/joint \
  -H "Content-Type: application/json" \
  -d "{\"joints\": $HUGE_JOINTS}"
```

**预期：** `success: false, code: 1003`（关节数量不匹配），不 OOM

---

## 5. 代码质量评估

### 5.1 已知问题

| 编号 | 问题 | 严重度 | 说明 |
|------|------|--------|------|
| **Q-01** | `std::filesystem` 使用 C++17 | 中 | `robot_http_server.cc:28` 引入 `<filesystem>`，但 CMakeLists 设定 C++11。部分编译器可能报错 |
| **Q-02** | MovePath 返回 501 | 低 | `Robot::Path`/`PathEntry` 为 private，无法在 HTTP 层构建。需修改 Robot 类暴露接口 |
| **Q-03** | `urdf_file_path_` 直接访问 | 低 | `robot_http_server.cc:327` 直接访问 `robot_->urdf_file_path_`（protected），依赖 friend 关系 |
| **Q-04** | detached thread 无上限 | 中 | 异步运动使用 `std::thread(...).detach()`，无线程池管理。极端情况可能创建过多线程 |
| **Q-05** | taskMap_ 无清理机制 | 低 | 历史任务持续累积在内存中，长时间运行后可能膨胀 |
| **Q-06** | MoveC mode 字段为 int | 低 | 设计文档定义 mode 为 `"UNCONSTRAINED"/"FIXED"` 字符串，实现中直接用 int |
| **Q-07** | IK 失败错误码不精确 | 中 | MoveJ_IK 的 IK 失败返回通用 500 而非 2001 |
| **Q-08** | 无超时保护 | 中 | 同步运动可能永久阻塞 HTTP 线程（如果底层运动卡死） |

### 5.2 与设计文档对齐检查

| 设计要求 | 实现状态 | 备注 |
|----------|----------|------|
| 30+ API 端点 | ✅ 已实现 | 31 个端点 |
| 异步 task_id 追踪 | ✅ 已实现 | task_1, task_2... |
| 业务错误码 1xxx-5xxx | ⚠️ 部分 | 参数校验已覆盖，IK 错误码 2001 未使用 |
| CORS 支持 | ✅ 已实现 | 所有响应附带 |
| JSON SE3Pose 格式 | ✅ 已实现 | position + orientation |
| 独立可移植类 | ✅ 已实现 | 仅依赖 Robot* |
| MovePath pathway | ❌ 返回 501 | Robot 类 Path 为 private |
| WebSocket 实时推送 | ❌ 未实现 | P3 优先级，可后续添加 |

### 5.3 改进建议（优先级排序）

| 优先级 | 建议 | 影响 |
|--------|------|------|
| **高** | 修复 C++17 filesystem 兼容性 | 编译失败风险 |
| **高** | IK 失败时返回 code 2001 而非 500 | LLM 错误恢复依赖精确错误码 |
| **高** | MoveC mode 支持字符串解析（"UNCONSTRAINED"/"FIXED"） | 与设计文档对齐 |
| **中** | 用线程池替代 detach()，限制并发任务数 | 防止线程爆炸 |
| **中** | taskMap_ 添加 TTL 清理（如 1 小时自动删除已完成任务） | 内存管理 |
| **中** | 同步运动添加超时参数 | 防止永久阻塞 |
| **低** | Robot::Path/PathEntry 改为 public 或提供工厂方法 | MovePath 支持 |
| **低** | 添加 WebSocket 实时状态推送端点 | 高频状态更新场景 |

---

## 6. 测试通过标准

### 必须通过（P0）

- [ ] 所有 GET 类接口正常返回有效 JSON
- [ ] 所有 POST 类接口正常处理请求
- [ ] 异步运动返回 task_id，可追踪状态
- [ ] 参数校验（空 body、字段缺失、类型错误）不崩溃
- [ ] 20 并发查询不崩溃
- [ ] CORS 头正确设置

### 建议通过（P1）

- [ ] 100 次快速 enable/disable 无异常
- [ ] 5 个异步运动并发正常
- [ ] 60 秒持续运行无性能衰退
- [ ] MoveC mode 字符串解析正确

### 可选通过（P2）

- [ ] 大 payload（1000 关节）不 OOM
- [ ] MovePath 端点可用（需先修改 Robot 类）
- [ ] WebSocket 实时推送可用
