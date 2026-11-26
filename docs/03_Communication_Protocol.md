# 当前通信协议：ESP-NOW v2 / robot USB v2、v3

ESP-NOW v2同时用于无动力与 [小范围有动力空载](18_Powered_Empty_Trial.md)，无线字节和控制坐标不变。robot USB v2仅用于无动力，显式有动力构建使用下述v3。旧24字节协议、deadman、mode和速度控制设计见 [归档](archive/Communication_Protocol_Pre_V2.md)，当前两端拒绝旧协议。

## ESP-NOW

Wi-Fi STA、信道6、25Hz，wearable `EC:E3:34:EF:81:64` → robot `C0:CD:D6:D0:9B:54`。固定82字节，小端，显式encode/decode，不传C结构体。接收回调只验证固定界限并交接有界邮箱。

| 偏移 | 类型 | 内容 |
| --- | --- | --- |
| 0,1 | uint8各一 | version=2，type=1 |
| 2 | uint16 | bit0输入有效，bit1 A按下，bit2 B按下，bit3两按钮启动消抖完成；其余0 |
| 4,8,12,16 | uint32各一 | 非0 boot、seq、uptime_ms、sample_age_ms |
| 20 | int32[4] | Hamilton `[w,x,y,z]`，传感器→世界姿态，单位10⁻⁶ |
| 36 | int32[3] | 传感器XYZ、零偏修正角速度，单位0.01°/s |
| 48 | int16[3] | 传感器XYZ原始加速度，单位0.001g |
| 54 | uint16 | 校准样本数0–250 |
| 56,60,64,68 | uint32各一 | ac、bc、I²C错误、拒绝样本计数 |
| 72,74 | uint16 / int16 | 加速度模长0.001g / 温度0.01°C |
| 76,77 | uint8各一 | WHO_AM_I / 拒绝原因0–5，沿用USB att:3 |
| 78 | uint16 | 保留，必须0 |
| 80 | uint16 | CRC16/CCITT-FALSE，覆盖0–79，poly=0x1021/init=0xFFFF/xorout=0 |

四元数范数误差≤0.01，接收主循环归一化。角速度范围沿用±2031°/s诊断界限，±2000量程饱和由腕端rej/valid体现。加速度各轴±4g，模长0–7g。ac/bc是uint32消抖按下计数，按住和释放不增加，开机已按下不产生点击。

同boot序号按差值`0 < delta < 2^31`接纳；重复/倒序不刷新有效时间。按钮计数同规则回绕；B累计多个事件强制打开，不补执行闭合。boot变更、无效输入、年龄≥250ms均清参考/控制权并更早开爪。年龄为样本自身age加robot本地接收后时间，不将两个uptime相减估算无线延迟。

## USB

腕端USB `v=1,att=3`保持 [原字段](16_Virtual_Console.md)，115200 8N1、25Hz、最大384字节。无线独立运行，串口发送失败不会阻断无线。

robot使用115200 8N1、LF、每行最多512字节（含LF）、`v=2`。PC先发送`hello`（role=pc、seq），收到`hello_ack`包含role=robot、protocol_version=2、boot_id、session_id、MAC、mode=robot_no_power、output_locked=true。每次新握手停止旧会话，不会授权。

管理命令仅`init/stop/heartbeat`，携带session_id与递增seq；init还需要长度1–64的request_id，重复请求不重新采参考。没有joint_target、request_owner或unlock。未知动作返回POWERED_OUTPUT_UNAVAILABLE。init/stop返回带request_type和ok/error的ACK/NACK；成功heartbeat无ACK，状态显示管理租约。ACK不代表动作完成。

完整快照5Hz，5组`control/imu/targets/gripper/health`共有boot_id、snapshot_id、t_ms以及各自USB seq。后端仅原子发布同一编号的完整5组，拒绝倒序、超时、不兼容和输出锁不一致快照。完整批预算2048字节、每行512；最坏序列化测试与真实格式共享代码。UART拥塞丢整个快照，事件优先，不阻塞控制循环。

- control：启用/运行/owner、稳定门控、会话参考、三维相对向量、大角保护和故障。
- imu：robot接纳的无线样本身份及完整att:3诊断语义，不使用另一USB流替换。
- targets：几何Pose对象requested/output_target、速度、触限、measured=null；夹爪明确用open_ratio，不复用旧q_req/q_out第四项角度语义。
- gripper：独立状态机、5000ms期限、经过/剩余时间、周期、打开原因及重闭合门控。
- health：执行模式、MAC、输出锁、PCA逻辑回读、管理租约及无线/串口统计。
- event：boot_id、event_seq、t_ms和code，后端按boot/event_seq去重。事件缺口明确记录，不补造动作。

## Web

`GET /api/status`与`/ws/telemetry`及新录制为schema5，保留input/imu/buttons/executor/control/joint_state/recording/replay、health/gripper，新增有动力output和人工观察记录。requested/output_target均是软件目标，实体时来自robot，measured始终null。旧schema2/3/4录制按原数据保留，只按明确的保存目标在虚拟执行端回放。

执行端选择、连接、初始化和停止使用`/api/executor/*`，虚拟旧接口仅在virtual模式可用。浏览器管理心跳250ms/750ms租约；后端→robot心跳100ms，robot本地300ms失效。实时按钮和目标只走ESP-NOW。

## 有动力 robot USB v3

仅显式 `RobotMode powered` 构建使用230400 8N1、LF、每行含LF≤512字节。hello/hello_ack的v和protocol_version均为3，mode=robot_powered，额外返回profile_hash；启动output_locked=true。客户端须匹配实际MAC、模式、版本和本次配置哈希，不降级连接。hello停止并清参考，不恢复使能或控制权，已经保持的承重PWM按当前输出状态如实报告。

v3完整快照5Hz，在v2相同语义的五组外增加output，同boot/snapshot_id/t_ms后才原子发布。总预算3072字节，最大5Hz数据率15360 bytes/s，230400 8N1容量23040 bytes/s，其余留给管理与优先事件。v2仍保持原2048预算和115200。

output包含profile_hash、stage（locked/holding/disabled/unknown）、pwm_ticks（底座/大臂/小臂/夹爪顺序）、last_write_ms（robot单调毫秒）、write_errors、pwm_enabled、committed、trial_limits_deg、max_velocity_deg_s、max_acceleration_deg_s2、power_confirmed、prepared、diagnostic_active。committed表示曾成功提交过软件目标；unknown/disabled时保留的PWM计数为历史值，Web当前output_target为null、last_committed_target保留最后成功目标。unknown时PWM是否有效为未知，不能写为已禁能或已保持；power_confirmed是本会话人工确认，不是电压测量。

管理动作prepare、enable_output、test_joint、disable_output、init携带session_id、递增seq、当前boot_id、profile_hash和1–64字符request_id。相同ID与相同内容只确认原请求，不重做动作；不同内容复用ID拒绝。心跳与stop不构成运动目标流。

- prepare检查逻辑健康与当前状态，建立受看门狗约束的管理准备。
- enable_output需supported、power_confirmed、startup_aligned三个明确true，独立于初始化；提交0/45/45/open后暂停保持。未作现场确认时必须拒绝。
- test_joint仅axis=0/1/2的单次受限轨迹：底座相对启动位offset_deg仅0、±5、±10、±15；大小臂仅0、±5、±10。集合来自当前哈希绑定的trial配置，禁止运行中或已有诊断时插入连续目标。当前跟随限位为底座±45°、双臂20～60°；扩大跟随范围不扩大离散诊断白名单。
- init由robot稳定采集参考，从已提交目标限速回启动位；不允许参考与目标跳变。
- stop清参考、停止推进并开爪，承重轴保持最后成功目标；不设FULL_OFF。
- disable_output需supported=true；正常结束先停止并到软件打开目标，再在托住后释放PWM。输出未知的人工处置不能被视为自动恢复许可。

Web对应POST `/api/executor/prepare|enable-output|test-joint|disable-output|acceptance`。prepare建立页面管理令牌，后续运动管理绑定boot、profile_hash与当前manager_token。停止或会话失效后，disable-output仍可凭当前USB会话、boot、配置及supported=true明确释放PWM，不恢复控制权。ACK表示接纳，后续快照表示软件输出阶段，实物动作只由现场观察记录，均不等于编码器反馈。

有动力`POST /api/executor/initialize`可显式携带`direct_observation:true`和非空`note`，仅在当前boot/配置/管理会话匹配、PWM保持、诊断结束、有效稳定输入且无录制/回放冲突时接纳。该路径省去人工逐轴验收门槛，仍下发相同robot init并保持暂停；普通初始化仍要求逐轴观察结果。`control.observation_ready`报告该路径当前是否可用。成功接纳后记录OPERATOR_DIRECT_OBSERVATION事件及metadata.direct_observation，列出未验收轴，不改acceptance的passed含义；停止/失效/会话变化撤销，不自动恢复。当前观察配置速度15°/s、加速度参数30°/s²，范围底座±45°、双臂20～60°。

实体当前profile的mapping_version为`coordinated-angle-v4`、joint_signs为[-1,-1]，仅反转相对旧实体v3的底座方向。ESP-NOW v2四元数/坐标及USB v3字段布局不变，以profile_hash绑定实际控制配置；后端control.follow.version/joint_signs与配置、录制metadata反映该版本。virtual/no_power仍为v3、[+1,-1]；历史记录按保存目标及自身元数据查看，绝不以当前符号重解算旧IMU。
