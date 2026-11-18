# 机械臂 ESP32 控制固件

`robot.ino` 用于机械臂端 ESP32。 Arduino 板型为 `ESP32 Dev Module`，FQBN 为 `esp32:esp32:esp32`。使用仓库的 `libraries/MotionArmLink`，其对端 STA MAC 须与当前穿戴端一致。

在已经安装 ESP32 Core 3.3.12 的纯英文工作副本中编译。

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32 --library .\libraries\MotionArmLink .\firmware\robot
```

仓库中的 `ArmProfile.h` 没有通电动作参数。使用这一版本时可以在 115200 baud 读取 `hello_ack`、`state`、`imu`，检查 PCA9685 和无线链路。请求取得控制权会返回 `CALIBRATION_REQUIRED`。首次使用串口时发送 LF 结尾的一行握手请求。

```json
{"v":1,"type":"hello","seq":1,"role":"pc"}
```

## 实机标定配置

复制 `config/robot_calibration.example.json` 到实机专用文件，填写在本机测过的数据。保留夹爪、肘、底座、肩的接线 `CH12/13/14/15`，逻辑顺序为底座、肩、肘、夹爪。除示例文件的字段，还需记录以下实测数据。

- 使用当前 `joint_demo_v2` 或历史标定的名义串口脉宽值时，把 `board.pulse_command_timebase` 设为 `calibrator_nominal_us`。两份固件都用每计数 4.88 名义 µs，配置字段记录同一套命令值，舵机安全行程按实物观测填写。使用示波器量得的实际脉宽时，选择 `measured_us` 并填写 `board.servo_pwm_period_us`，同一份配置统一使用一种时间基准。
- `joints[].home_us` 为人工摆到见证初态时验证的目标脉宽，并与同一轴的 `home_deg` 配对。夹爪的 `home_us` 必须等于测得的 `open_us`。
- `workcell.safe_joint_regions` 填入经验证可达的四维区间，例如用 `min_deg` 和 `max_deg` 各记录四个边界。移动时当前目标和下一步必须在同一个已验证区间内。区间验证应为 PWM 量化、舵机机械回差和跟踪误差预留边界余量。
- `workcell.gripper_capture_surface_verified` 只在验证全部允许夹持姿态下都有承接面或工件受托，且打开夹爪不会造成危险时设为 `true`。省略该字段时夹爪保持标定的打开位。
- `joints[3].max_closed_hold_ms` 必须实测并控制在 700 ms 以内。从夹爪偏离打开位开始计时。到时暂停运动并发出夹爪超时状态。只有经上述验证的工位允许闭合，超时、STOP、断链或故障时底座和两条臂保持最后命令，固件尝试低速打开夹爪。PCA9685 或供电失效时需要人工处置。

把实机专用文件的 `calibration_id` 留空，运行生成器后把打印的完整 SHA-256 记录到同一文件并提供给后端。

```powershell
python .\firmware\robot\generate_profile.py .\config\robot_calibration.local.json .\firmware\robot\ArmProfile.h
```

首次取得控制权须核对 `calibration_id`，并同时发送 `operator_confirmed_safe:true` 与 `operator_confirmed_initial_pose:true`。舵机首次收到 PWM 后会直接追向所确认初态对应脉宽，因此必须先人工摆到该姿态。运行中使用 `stop` 暂停及释放控制权，承重关节保持输出。机械关节没有位置反馈，串口 `q_out` 表示软件输出目标，`q_measured_deg` 为空。


当前演示的3秒夹持不适用于本生成器；映射锚点与45°启动位的区分见 [腕端交接](../../docs/Wearable_Handoff.md)。当前基线不会自动解锁本固件。
