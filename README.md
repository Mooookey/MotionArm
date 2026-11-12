# Robot Teleoperation MVP / 机械臂遥操作项目实施文档包

## 阅读入口

1. **[`Plan.md`](Plan.md)**：目标、边界、实物修正、总体架构、M0–M9 里程碑、功能接口与验收。
2. [`docs/01_Hardware_Safety.md`](docs/01_Hardware_Safety.md)：两 ESP32 + MPU6050 + PCA9685 的实物接线、安全供电、夹爪风险、上电和四路标定。
3. [`docs/02_Firmware_Control.md`](docs/02_Firmware_Control.md)：手腕端姿态/按钮/ESP-NOW，机器人端 50Hz 目标、斜坡、owner 与安全状态机。
4. [`docs/03_Communication_Protocol.md`](docs/03_Communication_Protocol.md)：24 字节 ESP-NOW、Serial NDJSON、ACK/lease/session、FastAPI REST 与 Vue WS 数据结构。
5. [`docs/04_Backend_Data_Replay.md`](docs/04_Backend_Data_Replay.md)：FastAPI、PySerial、SQLite、时间线、录制状态机、安全预检与回放。
6. [`docs/05_Frontend_Vue3.md`](docs/05_Frontend_Vue3.md)：Vue 3/TS/Vite/Pinia/ECharts 的控制台、录制/回放页面与断线规则。
7. [`docs/06_Test_Validation.md`](docs/06_Test_Validation.md)：分层单测、模拟/通电故障测试、MVP 验收和实验报告模板。
8. [`docs/07_Execution_Tracker.md`](docs/07_Execution_Tracker.md)：团队任务矩阵、依赖、交付条件与英文路径构建建议。
9. [`docs/08_References_Decisions.md`](docs/08_References_Decisions.md)：与原资料冲突的口径、更正原因、开源项目的使用边界。
10. [`docs/09_Runbook_Troubleshooting.md`](docs/09_Runbook_Troubleshooting.md)：现场演示 SOP、紧急停止与故障排查。

## 安全配置示例

- [`config/robot_calibration.example.json`](config/robot_calibration.example.json)：**全部真实行程参数初始为 `null`，`allow_arm=false`，故意不可直接驱动机械臂**；需按实物逐路校准、校验并保存新配置。
- [`config/system.example.yaml`](config/system.example.yaml)：建议通信频率、超时与服务端本地部署设置；不是实测设备参数。

## 必须遵守的范围

- 实物四个 PWM 控制通道：**CH14 底座、CH15 大臂、CH13 小臂、CH12 夹爪**，不是上游五轴 CH0–4。
- 前端**只采用 Vue 3**，不引入 React、摄像头视觉闭环或自然语言控制。
- 采用手腕 IMU 姿态 → 关节增量速度 → 安全 PWM；**无需在 MVP 引入 XYZ 逆运动学**。
- 普通三线舵机不回传外部关节轴角；网页与数据都只能称“指令/软件目标”，不能称“实体位置实测闭环”。
- 机械臂承重关节在失去力矩后会下垂。配置未实测、机械无支撑、电源未确认或无法物理断电时**不得启用运动**。

## 衔接原有仓库

保留原 `AGENTS.md` 和实际工作区的工具链。建议仅将本包的 `Plan.md`、`docs/`、`config/` 放入原仓库；如果原目录已有同名文件，先用版本控制对比，**不要覆盖**已有实测校准/源代码。文档使用英文路径而中文内容，符合已有工具链限制。
