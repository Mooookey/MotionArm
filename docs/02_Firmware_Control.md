# 当前固件控制

当前权威说明为 [ESP-NOW无动力执行端](17_No_Power_Executor.md)，无线字节协议见 [通信协议](03_Communication_Protocol.md)。腕端保留100Hz姿态采样和USB att:3，新增25Hz ESP-NOW v2。执行端本地管理固定参考、A点击跟随、B点击夹爪、约束与5000ms独立保护。

当前robot固件仅有无动力目标计算与PCA FULL_OFF输出锁，不能通过管理命令或标定配置解锁。有效输入超时250ms、管理心跳超时300ms均停止/清参考/请求打开，恢复后必须显式初始化。

按住A、mode切换和速度积分属于 [旧控制设计](archive/Firmware_Control_Pre_V2.md)，不进入当前链路。旧串口抓放源见 `firmware/archive/joint_demo_v2`，不属于本阶段构建或运行入口。
