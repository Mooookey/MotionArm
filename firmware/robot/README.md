# robot：无动力锁定与空载小范围执行端

两种构建均以ESP-NOW v2作为实时输入，执行端本地管理完整四元数固定参考、A点击跟随、B点击夹爪、限位/速度/加速度和5000ms不可续期保护。默认构建仍是永久锁定的无动力固件。详细协议见 [通信协议](../../docs/03_Communication_Protocol.md)，操作见 [无动力执行端](../../docs/17_No_Power_Executor.md) 及 [空载执行阶段](../../docs/18_Powered_Empty_Trial.md)。

**无动力构建的舵机动力必须关闭**。默认镜像只配置和回读PCA9685 FULL_OFF，有效PWM驱动和使能命令被编译排除；串口、Web或配置无法解除该锁。FULL_OFF是无动力启动锁，不是有动力阶段的松爪策略。

USB115200、LF、每行≤512字节。PC先hello；仅支持init、stop和heartbeat。新握手/重连不初始化，管理心跳300ms过期本地停止、清参考、开爪；无线有效输入250ms过期同样处理。软件目标与实物位置区分，measured始终null；MAC层发送成功、I²C成功和串口ACK均不表示到位。

## 标定仍锁定

ArmProfile.h仍为空，不运行生成器为本机解锁。local/example配置schema2的安全标志保持false，未知值仍null。home_deg/home_us是映射锚点，startup_deg是独立启动角；保留竖直1610/1630/1570µs依据，不能用约45°1030/1050替换锚点。无动力默认0/45/45只属于软件联调请求，不确认机械初态。

生成器继续核对实际供电/断电/重力支撑、四轴行程/速度/加速度、工位区间与打开可达性；新策略要求max_closed_hold_ms=5000，不表示热安全认证。省略startup_deg、旧schema、缺少任何实测条件均拒绝生成；配置哈希覆盖启动与策略字段。后续有动力阶段另行验证，不以本轮通过代替。

```powershell
.\tools\build-local.ps1 -Action Build -Sketch robot -ArduinoData C:\Users\21211\AppData\Local\Arduino15
```

常规构建入口仅wearable/robot；历史joint_demo_v2位于firmware/archive，不烧录、不运行。原按住A、mode、速度积分和旧实体PC_MANUAL/REPLAY不进入当前固件。

## 显式有动力空载构建

`-RobotMode powered` 才编入PWM层，USB v3/230400，5Hz六组同快照编号遥测；每行≤512字节、批次≤3072字节。无线协议及腕端att:3均保持不变。启动仍FULL_OFF，构建、烧录、hello、初始化均不构成动力或动作授权。

```powershell
.\tools\build-local.ps1 -Action Build -Sketch robot -RobotMode powered -ArduinoData C:\Users\21211\AppData\Local\Arduino15
```

`config/robot_trial.json` schema3只生成`PoweredTrialProfile.h`：来源ad5644f，保留1610/1630/1570竖直锚点和1030/1050约45°点，底座1010/1610/2000分段；启动0/45/45/open；本轮允许运行底座±45°、双臂20–60°，三轴15°/s、加速度参数30°/s²，夹爪1/s和5000ms。它不填写空的正式ArmProfile，不伪造完整工作区、承重或热安全验证。5V/6A是现场报告的电源额定值，非电流/温升实测。

管理命令`prepare / enable_output / test_joint / disable_output / init`必须绑定当前`session_id / boot_id / profile_hash / request_id`。`enable_output`还要求`supported / power_confirmed / startup_aligned`均为true；首次PWM前必须托住并人工对齐启动姿态。`test_joint`仅接纳axis0–2和相对启动姿态的离散offset：底座0/±5/±10/±15°，双臂0/±5/±10°（0返回启动；新增范围仅作跟随限位），上一诊断完成后才可下一次；不是连续USB目标接口。相同request_id与相同标量内容只ACK不重执行，内容改变拒绝；32个已处理绑定请求后需重新握手，缓存不驱逐旧确认。

`prepare`后管理心跳100ms，300ms到期即停止/清参考/开爪。STOP、无线失效、会话变化均保持承重轴最后成功提交PWM，夹爪独立打开；`output_locked=false`表示仍在保持，不代表跟随已运行。初始化回启动姿态采用受限轨迹，完成后暂停。PWM层以单次CH12–15批量写入提交、定期回读；失败后stage=unknown、不再推进或自动恢复，不声称保持/开爪已完成。人工处置并重新准备。

计划结束先STOP并等待软件开爪；**提醒托住并等待确认后**，`disable_output(supported=true)`才释放PWM，再人工切动力。正常通电且承重轴保持PWM无需持续托扶。未知输出故障允许在确认托住后释放，不以软件开爪完成作为虚假前提。`measured`始终null，ACK和I²C成功均不是实体到位反馈。
单轴诊断 offset_deg=0 表示该轴按同样的限速轨迹返回启动目标；可按批准的离散级别逐级观察并返回0；禁止任意连续值或扩大诊断白名单之外的目标。
有动力串口在 begin 前显式选择 ESP32 APB 时钟，波特率仍为230400。ACK/NACK/HELLO采用4条整行有界待发队列，先于事件及整批遥测发送；拥塞时保留应答而不重执行动作，新HELLO清除旧会话未发应答，饱和时STOP仍执行并保留应答。v3 health额外上报serial_errors以区分接收解析错误与drop_uart发送拥塞；v2字段保持原义。

控制限位、启动角、推进参数、离散诊断集合与PWM限幅均读取生成的 PoweredTrialProfile；遥测的联调范围和速率也取同一配置。死区仍5°，底座仍由Roll而非Yaw控制，原四元数坐标与slew算法未改。以上是扩大空载观察范围，完整工作区、机械极限、承载与热安全仍未验证。
已知末端软件状态问题仍保留并记录：slew到达限位时直接清零速度，可能突破单步配置加速度的速度状态差分约束。本轮不宣称修复该问题，也不把配置值当作实体实测加速度。

本轮按用户明确选择使用三轴15°/s及加速度参数30°/s²，跳过剩余分档观察，直接进行同一范围内的受限跟随；离散诊断白名单保留，但不要求完成所有档位。仍需显式准备、现场通电/启动对齐确认、使能保持与稳定参考初始化；此调整不授权自动取得控制权。已知末端速度清零问题未修复，不宣称全程严格满足30°/s²。

本轮实体配置映射版本 coordinated-angle-v4：按现场反馈仅将底座关节符号+1改为−1，双臂Pitch符号仍−1；对应joint_signs=[-1,-1]并进入配置哈希。传感器/腕部坐标仍Roll=−sensorY、Pitch=+sensorX，完整四元数与Yaw忽略规则不改。virtual及no_power继续原v3方向[+1,-1]；本轮不做分档，原离散诊断白名单保留。范围为底座±45°、双臂20–60°，15°/s和加速度参数30°/s²不变，不据此声明新增范围或实物方向已验收。
