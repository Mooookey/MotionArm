# 03｜通信协议 v1：ESP-NOW、USB Serial、REST 与 WebSocket

> 通信协议是双端固件、Python 后端和 Vue 3 的唯一数据契约。本文中的消息体是**拟定规格**，应在编码前锁定并为每次修改提高 `protocol_version`；示例不是声称已有固件支持。

## 1. 关键设计决策

- **实时手腕控制**：ESP-NOW 传输小尺寸、固定长度的二进制姿态/事件帧，机器人只处理最新帧，不为控制命令建立可积压的 FIFO；链路无需校园 Wi-Fi。
- **机器人↔电脑**：USB CH340 串口，推荐起始 `230400 8N1`，以 `\n` 分隔 UTF-8 NDJSON，单行最大 **512 bytes**（应用层限制）；不读取无界字符串、不接受超长帧、不用 Arduino `String` 无限拼接。
- **电脑↔Vue3**：REST 执行低频管理操作，WebSocket 发布状态/事件；操作命令只发到 FastAPI，绝不直接访问物理串口/PCA9685。
- **通道的可信度**：MAC 层成功只表示送达同伴网卡，不代表机器人应用已接纳或舵机实体到位；串口 ACK 是协议解析/接纳结果，不是机械位置反馈。
- **版本 / 新鲜度**：所有消息必须携带版本与序号；状态含机器人 `boot_id`，控制命令必须绑定当前 `session_id` 和 `lease_id`；重启后旧 session/lease 失效。
- **时间**：每个设备持有本地单调时钟，不假设 wearable、robot、PC 已同步。所有关联由 PC 收包时间与设备启动标识建立，只在同一 `boot_id` 内比较设备 `uptime`；不从两个设备的 `uptime_ms` 简单相减计算网络延迟。

## 2. ESP-NOW WearableFrameV1（二进制定长 24 字节）

**编码固定小端、无编译器填充，不得用不同平台的未经 `packed` 且未经 `sizeof` 检验的结构体裸发。** 发送前计算 CRC16-CCITT-FALSE，接收端先检查长度、版本、预期 MAC，再解析字段并检验 CRC。

| Offset | 字段 | 类型/字节 | 解释 |
| --- | --- | --- | --- |
| 0 | `version` | `uint8`,1 | 固定 `1` |
| 1 | `msg_type` | `uint8`,1 | 固定 `1` = IMU_CONTROL |
| 2 | `flags` | `uint16`,2 | bit0 `imu_valid`；bit1 `deadman`；bit2 `neutral_calibrated`；其余 0 |
| 4 | `boot_id` | `uint32`,4 | 腕端启动时产生的随机非 0 标识，用于区分重启 |
| 8 | `seq` | `uint32`,4 | 每发送一次 +1；在同 boot_id 内按模 2³² 顺序处理 |
| 12 | `uptime_ms` | `uint32`,4 | 手腕本地采样完成时间，毫秒；可能回绕 |
| 16 | `roll_cdeg` | `int16`,2 | Roll 角（百分之一度），减中立偏置后的姿态 |
| 18 | `pitch_cdeg` | `int16`,2 | Pitch 角（百分之一度），减中立偏置后的姿态 |
| 20 | `mode` | `uint8`,1 | 0=SHOULDER, 1=ELBOW；其他值拒绝 |
| 21 | `gripper_event_id` | `uint8`,1 | 有效的短按完成事件计数，按模 256 增长；不能靠按下电平每帧翻转 |
| 22 | `crc16` | `uint16`,2 | CCITT-FALSE（poly 0x1021, init 0xFFFF, no xorout），覆盖 [0,22) |

**唯一动作来源：** `roll/pitch`、`deadman` 与当前 `mode`。Robot 本地 50Hz 积分；重复同 `seq` 被丢弃，`gripper_event_id` 只在存在**合法新事件**时改变一次且事件需满足当前 TELEOP/使能条件。若事件计数变化发生在无效 deadman/错误 owner 期间，robot 仍须更新 `last_seen_event_id` 但**不执行也不缓存**动作，防止后来重新 ARM 时把旧事件执行出来。对于重连时的 `boot_id` 变化，先强制进入非运动状态，要求 A 按钮释放/重新按下才能恢复，避免手腕持按钮重启后直接运动。

**包丢弃条件：** 长度≠24；CRC/版本错误；未知 MAC；未校准、IMU 无效、异常角（例如绝对值明显超出可解释范围）；倒序/重复 seq；消息年龄/接收时间不满足安全策略；机器人控制者不是 WEARABLE。无线包回调**不操作 I²C 和舵机**。用于故障统计的 `rssi` 是接收侧属性（若 SDK 提供），不属于上述腕端 24 字节帧。

**时效性注意：** 两端没有同步时钟，不能通过直接比较 `uptime_ms` 证明空中单向时延；`robot` 以最后一帧**成功验证并接纳的本地时间**启动例如 250ms 的 watchdog。可配合“只保留最新帧、发送回调后再发送下一帧、序号单调”减少延迟数据堆积。若要准确估计单程链路延迟，另做 ping/echo 往返测量。

**ESP-NOW 设置：** `WiFi.mode(WIFI_STA)` + 明确固定信道（如实验确定的 CH1 / CH6，须同值）；获取两板 STA MAC 后建立单一 peer；启动时输出本机 MAC/信道并校验 peer。**不需要**开启 AP Web 服务器，也避免与校园路由器关联造成频道改变。MAC 白名单是输入筛选，不等同加密或强身份鉴别；局域演示环境仍要求物理 deadman。若具备条件，可配置 ESP-NOW PMK/LMK 加密，但这不是 P0 演示功能的替代安全措施。

## 3. Serial NDJSON v1：统一外层字段

串口一行一个 JSON object，编码 UTF-8，`
` (`\n`) 结尾；每行长度上限 512 bytes。所有消息有：

```json
{"v":1,"type":"hello","seq":1,"role":"pc"}
```

`v` 固定 1；`type` 为明确定义的小写字符串；`seq` 发送端按 session 递增；`boot_id` 由机器人报告，PC 不能任意指定；PC 命令必须含当前 `session_id`（握手时由 robot 生成），控制操作必须有 `lease_id`。字段缺失、`NaN/Infinity`、长度不等于 4 的 `q_deg`、非法类型、重复 seq、错误 session 都要返回 NACK 或静默丢弃（视是否能安全解析）。所有角度都是**逻辑度**，**不是** PWM tick 或 µs。

### 3.1 握手与设备识别

```json
{"v":1,"type":"hello","seq":1,"role":"pc"}
{"v":1,"type":"hello_ack","seq":1,"role":"robot","boot_id":"r-24d51","session_id":"s-8e91a","fw_version":"0.1.0","protocol_version":1,"calibration_id":"sha256:...","channels":[14,15,13,12],"joint_names":["base","shoulder","elbow","gripper"],"state":"SAFE_BOOT"}
```

`boot_id/session_id` 为**示例值**，正式实现应使用可区分重启与会话的随机/非重复 ID，不能固定这些字符串。后端枚举 CH340 或可访问的串口，逐端口以短超时询问 `hello`，仅选择明确回答 `role=robot` 且协议兼容的设备；不得写死 `COM3`，也不得因 Arduino 上传/监视器占用串口而让后端持久重试失败时误报“已连接”。

### 3.2 控制者申请、释放、停止、ACK

```json
{"v":1,"type":"request_owner","seq":2,"session_id":"s-8e91a","requested_owner":"PC_MANUAL","operator_confirmed_safe":true}
{"v":1,"type":"ack","seq":2,"request_type":"request_owner","result":"OK","lease_id":"l-a401","robot_state":"PC_MANUAL"}
{"v":1,"type":"release_owner","seq":3,"session_id":"s-8e91a","lease_id":"l-a401"}
{"v":1,"type":"stop","seq":4,"session_id":"s-8e91a","reason":"operator_stop"}
{"v":1,"type":"ack","seq":4,"request_type":"stop","result":"OK","robot_state":"HOLD"}
```

`operator_confirmed_safe=true` 表示人已在 UI 完成检查，不代表机器人自动测到真实位置。`request_owner` 请求需要内部状态/校准/物理初态检查；并非有 `true` 就必然允许接管。`stop` 为高优先级操作，不被普通 owner 限制；若 PC 已断线，机器人仍靠本地超时机制停止新动作。ACK 表示**接收与状态变更确认**，不表示运动结束或抓取成功。

### 3.3 单点关节目标与回放流

```json
{"v":1,"type":"joint_target","seq":5,"session_id":"s-8e91a","lease_id":"l-a401","q_deg":[90.0,65.0,105.0,30.0],"ttl_ms":200,"segment_ms":100}
{"v":1,"type":"ack","seq":5,"request_type":"joint_target","result":"ACCEPTED","robot_state":"PC_MANUAL"}
```

`ttl_ms`：机器人从接收时起允许本条命令存活的最大时间；`segment_ms`：希望底层在控制周期内平滑接近该目标的建议节奏，**不可用于跳过限速**。`q_deg` 必须符合部署时经过核准的四路角度范围，这里只是格式示例，不意味着 `[90,65,105,30]` 对真实机械臂安全。数值可使用有限浮点，禁止 NaN/Infinity，验证角度、速度及冲突后才 ACK。

对 `REPLAY`，PC 服务以 25–50 Hz 发送来自经预处理轨迹的 `joint_target`，每一条都是**完整四元组**而非单路增量；机器人通过 `lease_id` 和状态为 `REPLAY` 来区分来源。回放中还应有 `heartbeat`（例如 10Hz）以覆盖夹爪驻留段/轨迹暂停时的租约延续；当轨迹流连续超时或 PC 进程崩溃时，固件转 HOLD，不执行缓冲中未收到确认的旧目标。重新开始回放必须新建 lease 且重新人工预检。

### 3.4 状态遥测

```json
{
  "v":1,"type":"status","seq":101,"boot_id":"r-24d51","uptime_ms":123456,
  "state":"TELEOP","owner":"WEARABLE","fault":null,
  "calibration_id":"sha256:...","operator_initial_pose_confirmed":true,
  "q_requested_deg":[90.0,65.0,105.0,30.0],
  "q_applied_target_deg":[89.5,64.8,105.3,30.0],
  "q_measured_deg":null,
  "wearable":{"connected":true,"last_seq":3452,"last_valid_age_ms":32,"imu_valid":true,"deadman":true,"mode":"SHOULDER","roll_deg":-6.1,"pitch_deg":12.4},
  "pc":{"connected":true,"active_lease_id":null},
  "counters":{"wireless_drop":2,"serial_parse_error":0,"control_overrun":0}
}
```

`q_requested_deg` 是通过当前 owner 生成的期望目标；`q_applied_target_deg` 是底层斜坡/限幅后真正输出到 PWM 映射的**软件目标**，仍非实际角度。`q_measured_deg=null` 是有意保留的诚实字段，不得填入 `q_applied_target_deg`。`operator_initial_pose_confirmed` 是人工检查标志，不是传感器位置确认。**此示例 JSON 大于建议单行 512 bytes 上限**，实际串口应采用下面的轻量级遥测规范，详见 3.5。

### 3.5 最终生产串口遥测（最大 512 字节）

建议将上述状态拆为两个消息类型：

```json
{"v":1,"type":"state","seq":101,"boot_id":"r-24d51","t_ms":123456,"state":"TELEOP","owner":"WEARABLE","q_req":[90,65,105,30],"q_out":[89.5,64.8,105.3,30],"fault":null,"calib":"c1"}
{"v":1,"type":"imu","seq":102,"boot_id":"r-24d51","t_ms":123456,"imu_seq":3452,"roll_deg":-6.1,"pitch_deg":12.4,"mode":0,"deadman":true,"valid":true,"age_ms":32,"drop":2}
```

后端根据 `boot_id`、`t_ms`、状态序号合并成更丰富的 WebSocket 对象；串口保留紧凑、低阻塞、有限长度的数据。`calib="c1"` 可映射到 PC 本地已知的完整 SHA-256 校准哈希；握手阶段传完整哈希。详细故障信息另发 `event`，避免单行膨胀。

```json
{"v":1,"type":"event","seq":103,"boot_id":"r-24d51","t_ms":123458,"code":"WIRELESS_TIMEOUT","severity":"WARN","details":"teleop_hold"}
```

**编码要求：** 机器人以固定大小数组拼装 JSON（或使用固定内存池的 JSON 库），输出前严格校验最终序列化长度；打印调试日志使用其他专用类型，不混入同一串口协议的数据行。若 UART 输出拥塞，优先丢弃**非关键遥测状态**，而不是阻塞 50Hz 舵机任务。ACK、FAULT 与 STOP 的优先级高于图表数据，丢弃遥测需计数报告。

## 4. WebSocket 状态契约

`/ws/telemetry` 由 FastAPI 发布合并后的业务状态，每条消息独立自洽：

```json
{
  "event":"telemetry","server_time_ms":1794480000000,
  "connection":{"robot":"online","wearable":"online","serial_reconnecting":false},
  "control":{"state":"TELEOP","owner":"WEARABLE","deadman":true,"mode":"SHOULDER","fault":null},
  "imu":{"roll_deg":-6.1,"pitch_deg":12.4,"valid":true,"age_ms":32},
  "joints":{"names":["base","shoulder","elbow","gripper"],"requested_deg":[90,65,105,30],"applied_target_deg":[89.5,64.8,105.3,30],"measured_deg":null},
  "recording":{"active":false,"record_id":null,"frames":0},
  "replay":{"active":false,"record_id":null,"progress":0.0}
}
```

状态在 Vue 端映射为 **read-only source of truth**；不能由页面自己推算“舵机已经完成动作”。推荐广播 10–25 Hz，浏览器 ECharts 最多保留最近 1–5 分钟的环形缓存，不无限累积历史数组。WebSocket 建连后发送一次完整 snapshot、后续发状态增量/刷新，断开后 UI 立刻显示 `disconnected` 并禁用所有动作入口，**不自动续播**。

## 5. REST 请求与幂等性

| API | 方法 | 作用 | 预期失败情形 |
| --- | --- | --- | --- |
| `/api/status` | GET | 合并系统当前状态 | 一部分设备失联时仍返回带健康字段的响应 |
| `/api/control/arm` | POST | 授权 PC_MANUAL / 指定模式 | 409 控制权冲突 / 422 未确认初态 / 503 设备离线 |
| `/api/control/release` | POST | 放弃 PC 控制权 | 无效 lease / 已释放（幂等） |
| `/api/control/stop` | POST | 优先级高的软件停止 | 串口不可用时明确告警使用实体断电 |
| `/api/control/manual-target` | POST | 四关节低速目标（仅 PC_MANUAL） | 非授权/超限/无效数值 |
| `/api/recordings/start` | POST | 为当前 TELEOP 开始旁路录制 | 重复录制 / 未有有效机器人数据 |
| `/api/recordings/stop` | POST | 停止，保存完整性报告 | 未录制时返回当前结果 |
| `/api/recordings` | GET | 列出轨迹及配置兼容信息 | 无 |
| `/api/recordings/{id}` | GET | 查看元信息与曲线 | 404 |
| `/api/replays/preflight` | POST | 校验目标轨迹与环境/初态 | 409 已被控制 / 422 数据不完整或不安全 |
| `/api/replays/start` | POST | 使用预检 token + 人工确认启动 | 409/422/503 |
| `/api/replays/stop` | POST | 停止回放并 HOLD | 停止应幂等 |
| `/api/config/calibration` | GET | 读取只读标定和版本 | 未标定时返回 blocked |

所有有副作用的 POST 应携带服务端可识别的 `request_id`（或 `Idempotency-Key`），避免浏览器重试重复抓爪/重复开始。若 Python 重启，旧 token/lease 不能继续用；不允许 CSRF/跨站网页绕过人工确认。开发期只监听 `127.0.0.1` 并设置可控 CORS，若跨机器访问需额外鉴权和来源限制。不要暴露到公共网络。

## 6. 典型消息时序

### 6.1 遥操作与开始录制

```text
WEARABLE       ROBOT ESP32             FASTAPI              VUE3
  | 25Hz IMU ------>|                      |                    |
  |                  | safety + PWM         |<----status----------|
  |                  |-----state/imu------->|----ws telemetry---->|
  |                  |                      |<-- POST record/start|
  |                  |                      |----SQLite BEGIN     |
  |  new grip event->|------event---------->|----record event---->|
  |                  |                      |<-- POST record/stop |
  |                  |                      |----COMMIT/metadata  |
```

Recorder 只是监听状态，不能因为点击“录制”就从 wearable 抢走运动控制权。录制开始/结束的时间在 PC 端有明确 `server_monotonic_ns`，robot 数据也保留自身的 `boot_id + uptime`。

### 6.2 轨迹回放

```text
VUE3           FASTAPI                  ROBOT ESP32           WEARABLE
 |-- preflight -->|  validate file, calibration, initial pose | 
 |<-- report ------|                  |                        |
 |-- confirm/start>|-- request_owner(REPLAY) ----------------->| 
 |                 |<---------------------------- ACK lease ----|
 |                 |-- 25/50Hz joint_target + heartbeats ----->|
 |                 |<------------------------ ACK / state -----|
 |                 |-- log replay_run ------------------------>|
 |<-- progress ----|                  |                        |
 |-- stop -------->|-- STOP ---------------------------------->| 
 |<-- HOLD --------|                  |                        |
```

REPLAY 期间正常 wearable 命令不能驱动机器人；若用户松开/按下 A 但 robot 在 REPLAY，固件不把它解释为抢权。人工急停优先执行硬件开关/后台高优先级 STOP；如果硬件断电，servo 姿态未知、不得自动恢复。

## 7. 数据一致性与失败处理规范

1. **错误包：** 二进制 CRC 错、长度错、非法 JSON、非有限数、未知类型 → 拒绝；不更新 last_valid_control 时间；记录错误计数。
2. **旧包：** 同一 boot/session 下 seq 重复/逆序 → 不再次应用；`gripper_event_id` 改变但 seq 旧 → 不再次开爪。
3. **波形拥塞：** 高频控制只保留最新目标；不能积累几十条未完成角度命令，产生“松手后继续移动”。
4. **串口断联：** 机器人 `PC_MANUAL/REPLAY` watchdog → HOLD，PC 将录制标记 `interrupted`；串口重连必须先 HELLO 并比较新 `boot_id/calibration_id`。
5. **设备重启：** 新 boot_id；状态自动 `SAFE_BOOT`，旧 `owner/lease/event_id` 全失效；不从上一次 NVS 指令角直接启用电机。
6. **多浏览器：** FastAPI 保留一个明确操作会话，其他 WebSocket 可只读，避免两个页面同时提交回放请求。
7. **按键松开：** robot 在接收下一帧或超时到达前可能继续使用上一帧，因此必须发送低频心跳及设置明确 250ms 量级 watchdog，保证在链路失效时终止运动指令的继续积累。
8. **统计：** 记录每分钟 `rx_count`, `drop_count`, `duplicate_count`, `crc_error`, `serial_line_error`, `deadline_miss`, `watchdog_hold` 以便诊断。

## 8. 兼容性测试矩阵

| 用例 | 期待行为 |
| --- | --- |
| Core 3.3.12 两端编译 | 固定 24 字节结构尺寸、CRC 与接收回调签名正确 |
| ESPNOW 不同信道 | 不得接收任何有效控制，最终 watchdog HOLD |
| `boot_id` 改变 | 消除旧状态，要求重新 ARM |
| 中途断开 CH340 | 不可再接收 PC 回放指令，机器人 HOLD，Vue 报离线 |
| 异常大角度/空数组 | Serial NACK 422 等价错误码，不写 PWM |
| 两个页面同时请求 REPLAY | 仅一个 owner/lease 成功，第二个 409 |
| 消息体伪造 `q_measured` | 后端应仅使用机器人实际协议来源；设备源字段永远 `null` |
| 25Hz / 50Hz 混合数据 | ECharts 曲线可稳定渲染，机器人控制节拍无明显超时 |

## 9. 官方参考

- Espressif ESP-NOW （MAC ACK 不等于应用接纳、序号、Wi-Fi 信道）：https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html
- Arduino-ESP32 ESP-NOW API：https://docs.espressif.com/projects/arduino-esp32/en/latest/api/espnow.html
