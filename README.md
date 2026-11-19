# MotionArm：真实IMU与无动力执行端

当前阶段为 [ESP-NOW执行端无动力接入](docs/17_No_Power_Executor.md)。**舵机动力保持关闭**，只给ESP32和PCA9685逻辑侧供电；robot固件没有可解除的输出锁或有效PWM输出路径。本阶段不进行实体运动或带载抓放。

主链路：MPU6050 → wearable ESP32 → ESP-NOW v2 → robot本地控制与保护 → 软件输出目标 → 锁定的PCA9685。USB只用于开发、管理、诊断和Web遥测。

- [启动与无动力检查](docs/Local_Quickstart.md)
- [完整控制语义和阶段边界](docs/17_No_Power_Executor.md)
- [无线及USB协议](docs/03_Communication_Protocol.md)
- [虚拟控制台及既有实测记录](docs/16_Virtual_Console.md)
- [机械零姿态、尺寸与标定依据](docs/Robot_Baseline.md) · [结构化基线](config/robot_baseline.json)
- [未验收实机配置](config/robot_calibration.local.json) · [配置生成器说明](firmware/robot/README.md)
- [历史串口演示](docs/archive/Wide_Slow_Demo_V2.md)与`firmware/archive/joint_demo_v2`仅为历史证据，已退出当前运行入口。

当前A点击开始/暂停、B点击夹爪；完整四元数固定参考角度跟随，Roll控制底座，Pitch以1:1协同双臂。虚拟与robot夹爪统一不可续期5000ms保护，停止/失效/断链更早请求打开。Web区分虚拟和实体无动力软件目标，未安装位置传感器，measured始终null。

机械臂无驱动保持时会下垂。后续有动力阶段必须另行明确授权，在机械支撑、供电、断电手段和标定条件落实后，按单轴低速小范围、整臂空载、夹爪顺序验证。当前软件、通信或逻辑目标通过都不表示这些项目通过。
