# robot：仅无动力执行端

本轮固件运行ESP-NOW v2与USB v2，执行端本地管理完整四元数固定参考、A点击跟随、B点击夹爪、限位/速度/加速度和5000ms不可续期保护。详细协议见 [通信协议](../../docs/03_Communication_Protocol.md)，操作见 [无动力执行端](../../docs/17_No_Power_Executor.md)。

**舵机动力必须关闭**。当前代码只配置和回读PCA9685 FULL_OFF，没有有效脉冲输出函数或解锁命令；即使意外接通动力，也不能由本固件接受运动目标并输出PWM。FULL_OFF仅为本轮启动锁，不是有动力阶段的松爪策略。

USB115200、LF、每行≤512字节。PC先hello；仅支持init、stop和heartbeat。新握手/重连不初始化，管理心跳300ms过期本地停止、清参考、开爪；无线有效输入250ms过期同样处理。软件目标与实物位置区分，measured始终null；MAC层发送成功、I²C成功和串口ACK均不表示到位。

## 标定仍锁定

ArmProfile.h仍为空，不运行生成器为本机解锁。local/example配置schema2的安全标志保持false，未知值仍null。home_deg/home_us是映射锚点，startup_deg是独立启动角；保留竖直1610/1630/1570µs依据，不能用约45°1030/1050替换锚点。无动力默认0/45/45只属于软件联调请求，不确认机械初态。

生成器继续核对实际供电/断电/重力支撑、四轴行程/速度/加速度、工位区间与打开可达性；新策略要求max_closed_hold_ms=5000，不表示热安全认证。省略startup_deg、旧schema、缺少任何实测条件均拒绝生成；配置哈希覆盖启动与策略字段。后续有动力阶段另行验证，不以本轮通过代替。

```powershell
.\tools\build-local.ps1 -Action Build -Sketch robot -ArduinoData C:\Users\21211\AppData\Local\Arduino15
```

常规构建入口仅wearable/robot；历史joint_demo_v2位于firmware/archive，不烧录、不运行。原按住A、mode、速度积分和旧实体PC_MANUAL/REPLAY不进入当前固件。
