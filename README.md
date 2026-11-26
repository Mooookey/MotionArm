# MotionArm：真实IMU与实体执行端

当前工作为 [小范围空载实体联调](docs/18_Powered_Empty_Trial.md)。保留无动力构建的永久输出锁；有动力构建需明确选择，启动仍为FULL_OFF。烧录、连接或软件检查不授权通电；首次使能前必须托住、对齐启动姿态并取得现场确认。本阶段不进行实体回放或带载抓放。

主链路：MPU6050 → wearable ESP32 → ESP-NOW v2 → robot本地控制与保护 → 软件输出目标 → PCA9685。USB用于开发、管理、有界单轴检查和Web遥测，不转发实时姿态目标。

- [本机启动](docs/Local_Quickstart.md) · [有动力空载流程与待测项](docs/18_Powered_Empty_Trial.md)
- [完整控制语义和阶段边界](docs/17_No_Power_Executor.md)
- [无线及USB协议](docs/03_Communication_Protocol.md)
- [虚拟控制台及既有实测记录](docs/16_Virtual_Console.md)
- [机械零姿态、尺寸与标定依据](docs/Robot_Baseline.md) · [结构化基线](config/robot_baseline.json)
- [未验收实机配置](config/robot_calibration.local.json) · [配置生成器说明](firmware/robot/README.md)
- [历史串口演示](docs/archive/Wide_Slow_Demo_V2.md)与`firmware/archive/joint_demo_v2`仅为历史证据，已退出当前运行入口。

当前A点击开始/暂停、B点击夹爪；完整四元数固定参考角度跟随，Roll控制底座，Pitch以1:1协同双臂。虚拟与robot夹爪统一不可续期5000ms保护，停止/失效/断链更早请求打开。Web区分虚拟、无动力软件目标与实体最近成功提交的软件目标；未安装位置传感器，measured始终null。

2026-11-26当前实体观察配置为底座±45°、双臂20～60°、5°死区、三轴15°/s、加速度参数30°/s²。用户明确接受速度/加速度手感，要求扩大范围并反转实体底座方向，故实体采用独立`coordinated-angle-v4`、关节符号[-1,-1]；传感器坐标及旧虚拟/无动力v3方向保留。观察授权与逐轴验收分别记录，未测项目不记为通过，不代表完整工作区或遥操作响应性能通过。

用户提供独立5V/6A动力及人工断电手段。正常通电保持PWM无需持续托扶；首次使能、计划断电或释放承重PWM前须提醒并等待托住确认。按单轴低速小范围、整臂空载、夹爪顺序验证，软件检查不代替现场动作验收。
