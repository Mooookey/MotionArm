# 当前通信协议：ESP-NOW v2 / robot USB v2

本协议仅用于 [第一阶段无动力执行端](17_No_Power_Executor.md)。旧24字节协议、deadman、mode和速度控制设计见 [归档](archive/Communication_Protocol_Pre_V2.md)，当前两端拒绝旧协议。

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

`GET /api/status`与`/ws/telemetry`为schema4，保留input/imu/buttons/executor/control/joint_state/recording/replay，新增health/gripper。requested/output_target均是软件目标，实体无动力时来自robot，measured始终null。

执行端选择、连接、初始化和停止使用`/api/executor/*`，虚拟旧接口仅在virtual模式可用。浏览器管理心跳250ms/750ms租约；后端→robot心跳100ms，robot本地300ms失效。实时按钮和目标只走ESP-NOW。
