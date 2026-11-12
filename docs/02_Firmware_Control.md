# 02｜双 ESP32 固件、IMU 姿态解算与实时控制

## 0. 技术基线

MVP **优先使用现有 Arduino ESP32 Core 3.3.12**，不把五轴 ESP-IDF 底座硬搬进 `.ino`。两个固件保持低耦合：`wearable` 负责“观察手腕 + 交互意图”；`robot` 负责“控制权 + 软件安全 + 目标积分/斜坡 + PWM + 状态”。此处的控制器仍然只知道**软件目标值**，不能把它称为实际关节测量。

依赖建议：

- `wearable`：Arduino Core 自带 `Wire.h`、`WiFi.h`、`esp_now.h`；已有 `Adafruit_MPU6050`、`Adafruit_Sensor`、`Adafruit_BusIO`；按钮使用 `digitalRead` + 单调时钟状态机。
- `robot`：Arduino Core `Wire.h`、`WiFi.h`、`esp_now.h`，需补装/固定 `Adafruit_PWMServoDriver` 版本；建议借用它成熟的 `begin/setPWMFreq/setPWM` 驱动接口，不再自写 PCA9685 I²C 寄存器层。
- 设备运行数据存放 NVS 仅用于较小的**设备配对参数/已批准校准版本**；完整示教数据存电脑 SQLite，避免反复写 ESP32 Flash。
- 固件版本与协议版本分开；固件均提供角色、版本、构建哈希、校准哈希与 ESP32 当前 `boot_id`。

## 1. `wearable` 固件模块与循环

```text
setup:
  serial for diagnostics → GPIO button setup → Wire/MPU6050 probe → IMU bias cal
  Wi-Fi STA + fixed channel → ESP-NOW init + robot peer → SAFE/NEUTRAL

imu_task (100 Hz; 可测试 200 Hz):
  read accel/gyro with sensor timestamp
  validate finite/range/timeout
  estimate gyro bias/roll/pitch (complementary filter)
  update neutral-offset & validity

button_task (100–200 Hz):
  debounce active-low A/B
  short/long B release events; A level = deadman
  prevent B from generating short click after long click

packet_task (25–50 Hz):
  transmit latest filtered roll/pitch + mode + deadman + gripper_event_id
  seq++, uptime, boot_id; no unbounded backlog

diagnostic_task (1–10 Hz / 手腕 USB 调试时):
  print WHO_AM_I, bias, pose, flags, sent/lost counts, mode
```

中断或 ESP-NOW 回调中禁止做 I²C 或重计算；`onReceive`/`onSend` 回调处理短消息后投递有界队列。Arduino-ESP32 与 ESP-IDF 在不同版本存在 ESP-NOW 回调签名差异；**实际编译应以 Core 3.3.12 自带头文件为准**，不要从古老示例直接复制 `const uint8_t *mac` 老签名。

## 2. MPU6050 预处理与稳定姿态

### 2.1 上电标定与有效性

- 查询 `WHO_AM_I`，不为 `0x68` 或 I²C 连续读失败则 `imu_valid=false`，阻止新的遥操作位移。
- 静置约 2–5 秒（建议初值，实验实际调整）采集 100–300 帧陀螺零偏；若传感器在校零过程中晃动，应放弃校准，提示重新静置。保存此轮 `bias` 和时间/温度条件，禁止写固定的通用偏置。
- IMU 固定在手腕/手背，保证佩戴时不会相对手腕滑动，标定后先让操作员摆出舒适中立姿势，记录 `roll0/pitch0` 参考角；松开 A 时可以仅重校**中立姿态**，不要擅自不断重设惯性参考。
- 加速度模长在重力附近且短时变化不大时才认为加速度重力方向可信；动态加速度大时减小加速度校正权重或暂时只短时间依赖陀螺，避免机械运动诱发错误倾角。

### 2.2 滤波公式及方向约定

静止/轻动条件下 Roll/Pitch 由加速度与陀螺融合。以实验确定 MPU6050 的安装轴和旋转正方向；下式为方法示例，不是已校准轴向：

```text
gyro_angle_k = angle_{k-1} + (gyro_rate_k - gyro_bias) * measured_dt
acc_angle_k  = gravity_tilt_from_accel(ax, ay, az)
angle_k      = alpha * gyro_angle_k + (1-alpha) * acc_angle_k
```

`alpha` 可以先以 0.98（当 IMU 更新接近 100 Hz）试验，再根据静态漂移、快速转动响应与佩戴抖动调节；`dt` 必须来自 `micros()` 差分并限制到合理范围。死区建议从 5° 左右开始，运动指令采用可连续的分段线性/平滑映射以避免死区边界抖动。`roll/pitch` 单位必须明确为**degree**，协议中可用整型 `centidegree` 防止跨平台大小端及浮点 ABI 问题。

MPU6050 无地磁传感器，长时间偏航 Yaw 会飘；**不要**拿 Yaw 当绝对底座方向，也不能把手的线性加速度做两次积分得到“手部 XYZ”，否则漂移无法满足本 MVP 交互质量。转动“前/后”若与机器人方向相反，仅调整可追踪的 `axis_sign` 标定项，不改动硬件驱动函数。

### 2.3 输入防抖和事件处理

约定按钮 A=GPIO25（deadman）、B=GPIO26（gripper/mode），均**低有效**。参数初值：`debounce_ms=30`、`long_press_ms=700`、`min_rearm_release_ms=100`。这是待测值：具体按钮模块可能有自带消抖，不能因此跳过软件边沿事件处理。

```text
button A: pressed ⇒ deadman=1, released ⇒ deadman=0 immediately after debounce
button B on press: remember t_pressed
button B on release:
    if duration >= long_press_ms: mode = shoulder ↔ elbow; increment mode_event_id
    else if debounce_ms <= duration: increment gripper_event_id; gripper toggle requested
```

**两个常见陷阱：**

1. 若 B 在 `deadman=false` 或 `robot` 非 TELEOP 时触发，不得缓存待执行夹爪动作；只显示拒绝原因。
2. B 长按改变模式时不得再产生一次短按夹爪切换。需要事件 ID（递增）保证无线重复帧不会多次切换夹爪。切换模式的时刻应该让目标速度先归零（重新通过中立区更安全），不能让大幅 Pitch 自动传给新关节。

## 3. 机械臂关节增量式控制

### 3.1 物理映射

默认四个逻辑关节 `q=[base, shoulder, elbow, gripper]`；`servo_channel=[14,15,13,12]`。其中 `gripper` 单独走开/闭事件与低速斜坡，不参与持续 Pitch/Roll 目标速度积分。

推荐默认两种运动模式（不依赖 IK）：

| 模式 | Roll | Pitch | 按钮 A | 按钮 B |
| --- | --- | --- | --- | --- |
| `SHOULDER` | 底座 `base` 增量转动 | 大臂 `shoulder` 增量转动 | 按住使能 | 短按爪、长按切到 ELBOW |
| `ELBOW` | 底座 `base` 增量转动 | 小臂 `elbow` 增量转动 | 按住使能 | 短按爪、长按切到 SHOULDER |

**对用户体验的解释：** “前倾→伸出/后倾→缩回”是操作目标，不意味着单一肩关节的弯曲在所有姿态下都能实现严格的径向移动。先让操作员用两个关节模式完成实际抓放；之后若测试需要更自然动作，可在已测安全工作区加入**受限两关节耦合速度表**（对应关节组合，非 IK），并用碰撞测试验证。未经验证不要在固件中假装任何 Pitch 都指向末端的 XYZ。

### 3.2 限幅速度与时间积分

令 `theta = roll-roll0` 或 `pitch-pitch0`，设死区 `d=5°`、最大有效倾斜 `a=30°`（初值）：

```text
x = clamp((abs(theta) - d) / max(a - d, epsilon), 0, 1)
v_desired = sign(theta) * max_joint_speed_deg_s * x
if not deadman or not fresh or not valid or not in TELEOP:
    v_desired = 0
    v_prev = 0
    q_target = q_applied_target  # 冻结在当前已输出软件目标，不继续积分
else:
    v_limited = clamp(v_desired, v_prev-a_limit*dt, v_prev+a_limit*dt)
    q_target = clamp(q_target + v_limited*dt, calibrated_joint_min, calibrated_joint_max)
    v_prev = v_limited
```

**增量积分在 robot 端发生一次**，且必须以本机控制时钟驱动。不能按“每收到一个无线包就加 1°”实现（丢包/重复包会改变速度）。如果无线数据过时，速度强制趋于零，且不能继续朝老目标无限积分。对单次 `dt` 设置上下限，避免重启/调试断点/CPU 卡顿造成巨大角度跳变。

遇到 A 松开/无效 IMU/过期帧时，**先冻结 `q_target` 为最新 `q_applied_target`，禁止残余 `v_prev` 继续向前积分**。PWM 仍需维持承重姿态，不能为“停住”直接关掉肩/肘的供电。若需显式的减速度曲线，必须先设计有界停止距离并在实际机械臂上验证，不能让停机继续无界运动。

角度目标 → PWM 的速率限制再运行一层，确保从 `q_target` 到 `q_applied` 的最大每周期变化量有界：

```text
q_applied[k+1] = q_applied[k] + clamp(q_target - q_applied[k],
                                    -v_pwm_max * dt, +v_pwm_max * dt)
```

建议 `v_pwm_max` 和加速度限制 **每个关节分别标定**，肩/肘显著低于空载底座；不能把某个网上教程的 `180°/s` 当作已安全。对于大负载，应额外限制两个承重关节同时大幅加速。**各关节独立软限位不等于机械臂无碰撞**：还应把测试中发现的肩/肘组合禁区与底座扫掠危险区存成允许组合表/保守限制，在每周期关节组合验证后才发送 PWM；不必为此先实现复杂 IK。软件约束不替代机械限位与独立电源保护。

### 3.3 夹爪的离散状态机

```text
OPEN → CLOSING → CLOSED_LIMITED_HOLD → OPENING → OPEN
                           |
             max_closed_hold exceeded
                           v
                 ALERT_HOLD / MANUAL_ACTION
```

记录 `gripper_event_id`，只对新的事件 ID 切换；重复 ESP-NOW 包不得重复开关。达到接近夹爪 `close_us` 目标后不得继续加大脉宽；超时进入报警与不再移动任务阶段，**禁止在悬空状态定时自动松爪**。若要继续抓持，必须在供电、温升和机械负载符合要求的展示时间内，监护人员全程观察。

## 4. robot 固件实时调度与状态机

### 4.1 循环预算

建议固定 50 Hz（20 ms）更新目标和 PWM，在主 `loop()` 或独立 FreeRTOS task 运行：

```text
control_tick (20 ms):
  read latest one validated wrist sample from 1-slot overwrite mailbox
  read latest allowable PC command from bounded queue
  update input freshness / lease expiry / safety state
  select exactly one owner
  if TELEOP: integrate safe joint velocities
  if PC_MANUAL/REPLAY: accept target only if lease and seq valid
  apply joint software limits and servo slew/acceleration constraints
  write CH14/CH15/CH13/CH12 PWM via I2C
  emit snapshot to telemetry queue

wireless_callback: copy validated length and bytes to mailbox; return immediately
serial_task: read bounded lines; parse; issue ACK; enqueue allowed commands
telemetry_task (25 Hz): publish setpoints/state/fault/IMU summary over USB
watchdog: asynchronous check of last valid wrist, last PC replay command, task overrun
```

如果 `loop()` 阻塞在 `Serial.print`、`delay(500)` 或数据库操作上，20 ms 任务就没有实时保证；数据库只在 PC 端运行。ESP-NOW 接收时间一律记录为**机器人本地单调时钟**，腕端的 `uptime_ms` 只用于源内顺序，不能直接与 PC 时间相减当延迟。

### 4.2 安全状态定义（推荐机器可读字段）

| state | 输入权限 | PWM / 离开条件 |
| --- | --- | --- |
| `SAFE_BOOT` | 只接受 HELLO/诊断 | 不自动跳 Home；需实物初态确认、标定正确、支持结构就绪 |
| `READY` | 可请求一个控制者 | 保持已确认安全姿态，等待指令 |
| `TELEOP` | 仅 wearable 新鲜有效帧（含 deadman） | 失效/松开时停止新增位移；控制权主动释放后返回 READY/HOLD |
| `PC_MANUAL` | 仅后端授予的租约与单次低速目标 | UI 持续操作释放或超时后停止 |
| `RECORDING_TELEOP` | 仍是 TELEOP，记录旁路观察 | 录制不会切换运动控制者 |
| `REPLAY` | 后端已预检的轨迹流 | wearable 普通运动被拒绝；PC 超时/STOP→HOLD |
| `HOLD` | 仅 STOP、状态查询、人工恢复请求 | 不积累新目标，尽量保持安全支撑，禁止自动 HOME |
| `FAULT` | 诊断、STOP、需人工 reset | 未通过故障复位/初态检查不运动 |
| `ESTOP_LATCHED` | 只读诊断、重启后仍不自动 ARM | 物理断电/人工确认，不能靠网页按钮直接恢复 |

**安全规则：**

- 一次只能有一个 `motion_owner`：`NONE`、`WEARABLE`、`PC_MANUAL`、`REPLAY`；日志标明 owner 与被拒绝命令来源。
- 优先级：物理断电 > 固件故障锁定 / `STOP` > 现有 owner > 其他新 owner。`REPLAY` 不能因新无线指令抢权；无线手柄也不能覆盖正在执行的回放。
- `session_id`/`lease_id` 标识本轮控制授权；重启后随机 `boot_id` 使旧帧全部失效；PC 命令必须带 `seq`，有过期时效的 `deadline` 或短 `ttl`。
- `HOLD` 是软件策略，不保证无反馈舵机实体位置静止；在异常供电、齿轮损坏或不可用 PWM 时必须人工介入。
- **软件 STOP 与断开伺服电源不同。** 在需要瞬时消除机械危险时立即使用物理断电，并先部署机械支撑以避免重力下落。

### 4.3 超时初值（待测）

| 对象 | 建议初值 | 行为 |
| --- | --- | --- |
| 无有效无线帧 | 250 ms | 停止 TELEOP 速度积分 → HOLD / 等待重授权 |
| deadman 从 1 变 0 | 当前有效控制周期 | 立即令期望角速度=0，斜坡降速 |
| 无有效 PC REPLAY 续约/帧 | 250 ms | 停止消费轨迹、HOLD，不自动续播 |
| 无线重新连接 | 不自动 ARM | 需要按钮释放后重新按住 + 状态允许 |
| CH340 断线重连 | 不接受旧 lease | 新会话握手、校验配置，回放由人工重新启动 |

发送方必须周期持续发新鲜数据，而不是将旧帧排队。**停止定义为停止后续新增控制目标**，并非机械臂实体瞬时无惯性停住；应记录从检测事件到指令目标停止变化的实测时间。

## 5. 固件中的独立安全检查接口

```cpp
// 推荐函数边界；接口名是设计约定，不表示某上游库已经实现。
bool load_and_validate_calibration();
bool verify_hardware_safe_to_arm();
bool validate_wearable_packet(const uint8_t* buf, size_t len);
bool request_owner(ControlOwner next, uint32_t lease_id);
bool validate_pc_motion_command(const PcCommand& cmd);
void update_imu_control(float roll_deg, float pitch_deg, bool deadman, float dt_s);
void enforce_joint_soft_limits(float q_target_deg[4]);
void apply_rate_limited_setpoints(float dt_s);
void write_calibrated_pwm_to_channels();
void transition_to_hold(FaultCode reason);
RobotTelemetry make_status_snapshot();
```

约束检查应在**最靠近输出的 `robot` 控制任务中重新执行**；不能因为后端已验证就省略。没有通过校准的 `min_us/max_us` 和逻辑角范围时 `write_calibrated_pwm_to_channels()` 必须拒绝执行。

## 6. 实验与单元验证

1. **无舵机/模拟 PWM：** 输入角度序列，断言四路映射只会写 12–15，且始终在已标定脉宽中。
2. **姿态：** 静止、缓转、快速倾斜、回中立；检查角度连续、单位/符号正确、不能让 IMU 错误数据驱动目标。
3. **按钮：** 100 次短按、10 次长按、A/B 同时按、A 松开时 B 点击；无重复夹爪动作，长按不触发短按。
4. **通信：** 强制重复 seq、倒序包、断电重启、不同 MAC、不同 channel、过期/无效帧，确认无新动作。
5. **控制：** 对每个关节输入最大 Roll/Pitch，检查角速度/加速度/限位/单周期最大变化；确认未授权来源被拒绝。
6. **物理：** 先空载、再轻载；每步只恢复一个输入来源并可立即断电；承重关节支撑到位再测试。

## 7. 参考

- Arduino ESP32 ESP-NOW API / 回调签名：https://docs.espressif.com/projects/arduino-esp32/en/latest/api/espnow.html
- ESP-IDF ESP-NOW 官方序列号、信道、MAC 层 ACK 注意事项：https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html
- Adafruit PWM 库文档：https://learn.adafruit.com/16-channel-pwm-servo-driver/library-reference
