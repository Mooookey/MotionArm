# 04｜FastAPI 后端、SQLite 数据模型与安全示教回放

## 1. 后端职责边界

**FastAPI 是唯一 PC 侧调度入口**，机器人 ESP32 是唯一 PWM 输出仲裁者。后端承担：发现/独占 USB CH340、协议收发、时间线关联、校准版本一致性校验、录制保存、回放预检与指令流、REST/WS 视图。机械安全的最后一道软件限位始终在机器人固件内；即使 Python 崩溃，robot 也应自己执行超时 HOLD。

推荐 Python 3.11+、FastAPI、Uvicorn、PySerial、`sqlite3` 标准库和 Pydantic；可选 NumPy 仅用于离线数据分析，不是 MVP 实时执行必须依赖项。不要为这个单机系统另起 Express/MySQL/Redis/消息中间件，不需要依赖云服务。

### 1.1 推荐代码结构与所有权

```text
backend/
├── app/main.py                   # lifespan、依赖注入、统一异常映射
├── app/config/settings.py        # 校准路径、串口筛选、rate limit
├── app/core/protocol.py          # NDJSON v1 解码/编码/版本/seq 校验
├── app/core/serial_bridge.py     # 唯一 pyserial handle + RX/TX workers
├── app/core/command_broker.py    # 控制权/lease/ACK 追踪，目标去重
├── app/core/telemetry_cache.py   # 最新机器人/IMU 状态，只读 snapshot
├── app/services/recording.py     # 录制状态机与写入批处理
├── app/services/replay.py        # preflight/重采样/执行/停止
├── app/db/repository.py          # sqlite WAL/事务/批量写入
├── app/api/status.py             # 状态
├── app/api/control.py            # 授权、手动目标、停止
├── app/api/recordings.py         # 开始、结束、列表、详情、导出
├── app/api/replays.py            # 预检、开始、停止、进度
└── tests/{test_protocol,test_broker,test_recorder,test_replay}.py
```

### 1.2 串口**唯一拥有者**

- 使用 `serial.tools.list_ports` 搜索 USB CH340（VID/PID/描述只能作为候选线索），对各端口轮流 `HELLO` 握手，通过**role、协议版本、calibration_id、robot boot_id**确认真实设备；避免写死 `COM3`。如果某端口被 Arduino Monitor/上传占用，应显示“串口被其他程序占用”，不循环无意义地抢串口。
- **只有 `SerialBridge` 能读写 pyserial。** API 处理线程通过 `CommandBroker` 提交命令并等待有限时长的 ACK，不允许另一个任务并发读取或直接写端口。
- 每种队列职责不同：`motion_latest` 大小 1，新的实时手动目标覆盖未发送旧目标；`critical_commands` 小型高优先级、按序、可被 STOP 抢占；`telemetry_subscribers` 非阻塞丢弃旧帧；`database_writer` 按序保留录制样本并监控溢出。
- RX 解码后做 `v/type/seq/boot_id` 检查；控制 ACK 和统计事件先解析，不因图表消费者慢阻塞串口线程。TX 序列化时应保证线长上限与非法浮点拒绝。
- 机器人重启（boot_id 改变）立即失效所有 PC owner/lease，当前录制写入 `interrupted` 并归档，回放终止；重连不自动复原动作。
- 如果后端重启，页面立即显示 `reconnecting`，但机械端不允许沿用此前的 PC motion stream；重新发送 `request_owner` 前必须人工确认。
- 某些 CH340/ESP32 开发板在电脑串口打开/关闭时会被 DTR/RTS 控制线复位（取决于板载自动下载电路和驱动行为）；**不能假设连接 PySerial 不会让 robot 重启**。握手后必须比较 `boot_id`、确认固件处于 SAFE_BOOT/READY，并严禁自动移动到上次的目标。

## 2. 配置、设备状态和错误语义

`system.yaml` 建议包含：`serial.baudrate=230400`、设备发现超时、robot 状态超时、WebSocket 推送频率、`recordings.db_path`、`max_record_minutes`、回放插值频率、预检 token TTL。校准读独立文件，写入前固化其 SHA-256；每一份轨迹绑定录制时的 `calibration_id`，禁止因“更换了 Home”而静默复用旧轨迹。

**API 错误规范：** `400` 请求语义不合法；`409` 控制权/系统状态冲突；`422` 非法关节值、预检失败或未提供人工确认；`503` 设备离线/后端失去串口。响应包含稳定的 `error_code`、简明 `message`、`details`，Vue 根据 `error_code` 展示可操作提示。

后端状态缓存应明确：`robot_state`、`owner`、`wearable_online`、`robot_online`、`serial_online`、`calibration_compatible`、`position_known_externally=false`、`operator_initial_pose_confirmed`、`q_requested_deg[4]`、`q_applied_target_deg[4]`、`last_robot_seq`、`last_robot_t_ms`、`recording_status`、`replay_status`。不提供 `q_actual` 假值。

## 3. SQLite 模型（建议关系设计）

MVP 规模约 25–50 条每秒，单电脑 SQLite `WAL` 足够；写端由一个 DB writer 负责批量事务，不能每一帧触发多个连接的写锁竞争。表名和列名如下，具体 ORM/原生 sqlite 均可，需提供 migration 版本。

```sql
PRAGMA journal_mode = WAL;
PRAGMA foreign_keys = ON;

CREATE TABLE IF NOT EXISTS recordings (
  id TEXT PRIMARY KEY,                  -- UUID
  name TEXT NOT NULL,
  created_at_utc TEXT NOT NULL,
  finished_at_utc TEXT,
  status TEXT NOT NULL,                 -- recording/completed/interrupted/invalid
  robot_boot_id TEXT NOT NULL,
  wearable_boot_id TEXT,
  fw_version TEXT NOT NULL,
  protocol_version INTEGER NOT NULL,
  calibration_id TEXT NOT NULL,
  initial_target_json TEXT NOT NULL,   -- [base,shoulder,elbow,gripper]
  workcell_id TEXT NOT NULL,           -- fixed_pick_place_v1
  total_samples INTEGER DEFAULT 0,
  total_events INTEGER DEFAULT 0,
  duration_robot_ms INTEGER,
  missing_frames INTEGER DEFAULT 0,
  note TEXT
);

CREATE TABLE IF NOT EXISTS samples (
  recording_id TEXT NOT NULL REFERENCES recordings(id),
  sample_index INTEGER NOT NULL,
  robot_seq INTEGER NOT NULL,
  robot_t_ms INTEGER NOT NULL,
  pc_received_ns INTEGER NOT NULL,
  q_requested_json TEXT NOT NULL,       -- 4 有限数
  q_applied_target_json TEXT NOT NULL,  -- 4 有限数；非外部轴角反馈
  imu_roll_deg REAL,
  imu_pitch_deg REAL,
  imu_mode TEXT,
  deadman INTEGER,
  owner TEXT,
  fault_code TEXT,
  PRIMARY KEY(recording_id, sample_index)
);

CREATE INDEX IF NOT EXISTS idx_samples_robot_time
ON samples(recording_id, robot_t_ms);

CREATE TABLE IF NOT EXISTS events (
  id TEXT PRIMARY KEY,
  recording_id TEXT NOT NULL REFERENCES recordings(id),
  robot_t_ms INTEGER,
  pc_received_ns INTEGER NOT NULL,
  event_type TEXT NOT NULL,            -- claw_open/claw_close/mode_change/hold/error etc
  event_seq INTEGER,
  payload_json TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS replay_runs (
  id TEXT PRIMARY KEY,
  recording_id TEXT NOT NULL REFERENCES recordings(id),
  started_at_utc TEXT NOT NULL,
  stopped_at_utc TEXT,
  status TEXT NOT NULL,                -- started/completed/aborted/failed
  speed_scale REAL NOT NULL DEFAULT 1.0,
  preflight_json TEXT NOT NULL,
  stop_reason TEXT,
  operator_outcome TEXT                -- success/failure/unknown, 人工填写
);
```

**必要约束：** 写库前应用 Pydantic/手工验证 JSON 四元组均为 finite number；`samples` 在同一 recording 内 `robot_t_ms` 单调不递减，按 `robot_seq` 去重；两个不同 boot_id 的流不能混成一条轨迹；`missing_frames` 与 `status` 使中断轨迹默认不可回放。采用 SQLite 事务确保 `recordings.status` 与时间线提交一致。

**示教存什么：** `q_applied_target` + 机器人本地时间是主轨迹，`q_requested` 辅助分析斜坡滞后；IMU、模式、deadman 是解释性数据；`gripper` 即使在 q 中存在，也应额外记录**开/闭事件时间**，因为仅靠角度采样可能错过瞬时操作。

## 4. 示教录制完整流程

### 4.1 录制前条件

- robot 状态 `TELEOP` 且当前可以正常移动，`calibration_id` 完整并已与后端配置一致。
- 起点在台面固定 A 区，机械臂由人工确认已处于已标定的初始姿态；记录 `workcell_id` 和演示工件说明。
- 数据写入系统可用，SQLite 文件目录可写；若当前录制未完结，禁止开启第二个新录制。
- 从机器人接收状态稳定后才允许开始，避免在前几帧缺少初始关节向量。

### 4.2 启停状态机

```text
IDLE
  | POST /recordings/start
  v
RECORDING ──> FINALIZING ──> COMPLETED
  | robot timeout / serial disconnect / invalid boot / missing critical data
  v
INTERRUPTED ──> FINALIZING ──> INVALID_FOR_REPLAY
```

1. Vue 发起开始请求（显示响应中的 `record_id`）。服务端在数据库建立 `recordings` 并保存当前校准与首个可用目标；写入 `START` 事件。
2. 每收到 `state` 遥测用机器人时间和 PC 接收单调时间写入样本；同 `seq` 忽略重传。接收 `event` 写入夹爪和故障事件。IMU 是与最近一帧关联的估计数据，不伪装为完全同步测量。
3. 录制时间间隔由 **robot uptime** 提供。USB 偶发抖动不应直接改变轨迹速度；机器人重启、包缺失持续/时间轴不可恢复时设置 `interrupted` 标志。
4. 点击停止后，记录 `STOP` 事件、关闭本次样本写入并做一致性检测，提交 `completed` / `invalid`，返回样本数、持续时长、缺帧/异常统计、事件数及配置哈希。
5. 允许查看和导出原始时间线；后续轨迹平滑和时间缩放生成**派生版本**，绝不覆写原始录制样本。

### 4.3 数据完整性检验

- `>= 2` 条有效采样，时间戳可解析；机器人每条状态必须是相同 `boot_id`，相同 `calibration_id`。
- `[base, shoulder, elbow, gripper]` 长度固定 4，每个元素为有限数且落在记录时的标定范围内；事件 ID 不得非法重叠。
- `dt > 0` 是插值相邻唯一时标的要求；重复时间戳要去重或合并，不可造成除零。
- 若样本间隔远超 `expected_period × 3`（建议初值），标记 gap 并记录确切范围；该轨迹默认**拒绝自动回放**，需要人工查明问题，不能凭补插几帧“掩盖掉线”。
- 示教中 `HOLD/FAULT/ESTOP` 应写 event 并标记录制不适合直接回放；断电中断后必须新建录制。
- 正常结束且存在抓爪关闭与打开事件，记录是否有长时间闭合；本项目的物理抓取成功仍需人工观察，不可仅凭事件“完成”判断。

### 4.4 导出格式（仅示例）

```csv
robot_t_ms,base_target_deg,shoulder_target_deg,elbow_target_deg,gripper_target_deg,mode,deadman
0,88.0,64.0,110.0,25.0,SHOULDER,1
40,88.2,64.2,110.0,25.0,SHOULDER,1
80,88.4,64.5,110.0,25.0,SHOULDER,1
```

导出应附 `metadata.json`，声明 `angles_are="applied_servo_targets_not_encoder_readings"`、`calibration_id`、工作区 ID、是否完整及帧率/时间基准。CSV 只是人读和分析，后端回放推荐读取正式的 SQLite 记录和事件表，不直接信任用户编辑后的 CSV。

## 5. 回放前预检 Preflight（独立于 START）

### 5.1 全部为硬门槛

1. 轨迹必须 `completed` 且完整性 `valid`，记录与当前机器人 `calibration_id` 完全一致；固件至少与协议兼容。更换机械臂位置/机械连杆/夹爪行程后，需要新标定和新示教。
2. 所有四关节样本与事件在当前限制之内；无 `NaN/Infinity`，没有无界长 gap；单位明确是 degree/ms。
3. 机器人没有 `FAULT/ESTOP_LATCHED/HOLD 未恢复`，串口稳定在线，当前 owner 已释放，物理供电与支撑被工作人员确认。
4. 起点的**软件目标值**与录制第一点在每关节允许的**启动容差**内；若不一致，禁止瞬时跃迁，只允许安全低速、人工监督地返回标定初态（且返回动作已另行验收）。
5. **因为没有外部关节反馈，软件目标值一致也不足以证明实体起点相同。** UI 必须要求工作人员依据机械标记、工位照片或实体目视检查起点、泡沫块 A/B 位置、无人处于运动区域，然后二次确认。
6. 轨迹相邻采样对各关节的 `|Δq|/Δt` 不得超出当前已验收速度；检查加速度、夹爪动作间隔和同一时段多关节运动负载。若时间轴不可行，拒绝或生成有记录的统一放慢版本（`speed_scale <=1` 速度倍率），不得偷偷把角度裁剪造成偏差。
7. 回放闭爪总时间满足校准的 `max_closed_hold_ms` 与展示工件风险约束；夹爪悬空闭合超时不得自动打开，必须在演示开始前预检并拒绝超时轨迹。

返回可读预检报告：`preflight_id`（短期有效）、`record_id`、`ok`、`violations[]`、`warnings[]`、`calibration_id`、`initial_q_tolerance_deg`、`speed_scale`、`operator_checklist`。`POST /replays/start` 必须携带 `preflight_id` 与人工确认，不能重新利用旧 token。

### 5.2 时间戳插值：实现方式与误差

原轨迹样本 `(t_i, q_i[4])` 使用机器人单调时钟，转换为 `t_i_rel = t_i - t_0`；夹爪事件按相对时间存储。常规关节采用**分段线性插值**（忠实于原来的已应用目标），机器人侧再做限速；若希望看起来更柔和，可采用 `smoothstep(s)=3s²-2s³` 但必须意识到它会改变原始速度曲线，因此只能生成**版本化的派生轨迹**，并在预检中重新验速。

```python
# 伪代码，不是对真实设备立即可运行的命令
for tick_t in fixed_rate_ticks(period_ms=40):
    q = interpolate_recorded_applied_targets(tick_t, samples)
    if any_joint_or_gripper_constraint_violated(q, tick_t):
        abort_preflight_or_stop()
    else:
        send_complete_four_joint_target(q, segment_ms=40)
```

起始执行时，后端在开始前向机器人申请 `REPLAY` lease；每 20–40 ms 按轨迹的相对时间发送目标（实际推荐起始 25Hz，固件仍 50Hz 更新 PWM）。不要用 `for row: send(); sleep(20ms)` 追赶 USB 抖动；应用 `perf_counter_ns()` 单调时钟确定每一个目标应该发生的时刻，**错过 tick 则跳到当前时刻的正确插值值而非积压所有过期目标**。机器人侧仍再次校验每帧 `ttl/lease/seq` 和 PWM 斜坡。

当 `q_out` 受到额外速度限幅而落后于轨迹，不能宣称时间同步准确；可记录误差并在需要时整体放慢 `speed_scale`，或者选择有显式“等待斜坡结束”的事件（结束只代表软件 setpoint 已追上，并非实体位置到位）。如要确保实际夹住工件，应在录制轨迹里留足安全停顿并通过人工反复试验，而非依靠无编码器的到位确认。

### 5.3 夹爪事件与时间关系

`claw_close` / `claw_open` 独立事件由 `robot` 端实际**接受**事件时产生。回放按事件时间触发相应 `gripper` 目标，不从回放的随机插值误差推断开合；在开爪前的放置工位应留可复现的静止时间。若示教中的夹持时间超出容许值，预检直接拒绝而不是空中超时“保护性打开”。

## 6. 回放运行状态机与停止后语义

```text
IDLE → PRECHECKED → ACQUIRING_LEASE → RUNNING → FINISHING → COMPLETED
                                     |         |
                                     |         +--- operator_stop → STOPPED / HOLD
                                     +------------- timeout/fault → ABORTED / HOLD
```

- `stop` 是**优先级最高且幂等**，取消尚未发送的目标流、清空待发送未执行目标，并请求机器人 HOLD。
- 机器人 ACK `joint_target` 不代表“关节真实到位”；PC 应持续读取 `state`，记录 `q_out` 与跟踪 `desired` 的软件误差。
- 回放失败后保存 `replay_runs.stop_reason` 与结束状态，必须人工检查现场、重新预检。**禁止自动重试抓取**，因为无物体传感器不能判断之前是否已抓取/放下。
- 回放完成后只代表目标流全部被发送/接受且软件定时结束。物理动作完成与否由操作员选择 `success/failure/unknown`，不可自动填 `success`。
- `HOLD` 后不能自动 HOME；若需要将机械臂移出工作区，请使用另外经过验证的安全手动流程。

## 7. 前后端权限与运行部署

仅在本机运行 `uvicorn app.main:app --host 127.0.0.1 --port 8000`；Vite devserver 通常在另一端口，CORS 精准配置本地开发源，不允许 `*` 加凭据访问。可通过 `vite.config.ts` 代理 `/api` 和 `/ws`，减少硬编码 URL。

多浏览器/多个操作者：服务端保留唯一 `operator_session`（短期有效 token/owner），其他页面可以只读，所有操作都必须由后端检查状态并限频；Vue 按钮 disabled 只是 UX，不是安全保护。默认不接受互联网访问，课程展示时也不需要公网。

如果软件需要做 Windows 开机运行，不要令开机服务直接控制舵机运动。设备启动默认 SAFE_BOOT，必须在现场人工确认并有硬件防护才能激活。

## 8. 测试（不接硬件也能完成）

| test | 预期 |
| --- | --- |
| `test_serial_codec` | 小于 512B 的合法 NDJSON 可解析；异常浮点、超长行、乱序 seq 拒绝 |
| `test_owner_lease` | TELEOP 与 REPLAY 互斥；lease/session 重启清空；STOP 可抢占 |
| `test_database_roundtrip` | 1000 条样本+夹爪事件写入后有序恢复，配置哈希不丢失 |
| `test_recording_interrupted` | 串口或 robot boot 改变，轨迹写 interrupted；preflight 拒绝 |
| `test_preflight_limits` | 超限/缺帧/版本不一致/起始姿态不一致/夹持过久被拒绝 |
| `test_replay_timing` | 使用 FakeClock 无需等待真实 10s，验证插值/重采样无旧命令积压 |
| `test_stop_race` | 在开始/录制/回放中重复 STOP，不能先 ACK 停止又启动旧任务 |
| `test_ws_disconnect` | 客户端断开只停止其显示，不让后端误报实体关节已经断电 |

**外部位置真值的测试范围：** 有人工标尺和视频记录时可以统计工位搬运成功率，但由于不做视觉或编码器反馈，数据库不能出现臆测的 `actual_joint_angle`、`object_pose` 或 `grasp_success=true` 自动字段。

## 9. 后端启动/维护原则

- 维护 `requirements.txt`/锁定版本与 Python 虚拟环境，离线工作站先用可访问的缓存安装。
- DB 路径位于英文工作区的 `data/`（忽略入 git），自动创建目录；源码中不保留凭据、用户个人绝对路径。
- 调试日志记录“控制请求 ID + 结果 + state transition + fault”，不持续打印完整 50Hz JSON 到终端导致阻塞。
- SQLite WAL 的备份需一致性关闭或使用 SQLite backup API；不要在程序运行时直接复制 `.db` 作为保证一致的快照。
- 每次改动 `calibration_id` 和 `protocol_version` 先跑全部模拟测试，再在空载硬件跑一轮，旧录制标为不可直接回放。
