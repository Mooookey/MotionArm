# 08｜需求冲突记录、技术选型决议与开源复用边界

> 核查日期：2026-11-12。对仓库内容/许可证的表述限于公开可访问页面；后续若直接拷贝源码，仍需在实际使用的 commit 下逐文件核实许可和依赖。

## 1. 三份输入资料的优先级

| 文档 | 权限 | 保留的内容 | 不采用或需更正的内容 |
| --- | --- | --- | --- |
| 用户 `Plan(1).md` | **最高：MVP 范围** | IMU 姿态控制、模式切换、夹爪、固定环境录制回放、组件化、可不做 IK | 文字中误写“五路”与 `[j0..j4]`，按硬件实际更正 |
| 用户 `AGENTS(1).md` | **最高：实际硬件/环境** | ESP32 D0WDQ6、CH340、Arduino 3.3.12、MPU6050 0x68、CH12–CH15、按钮 25/26、下垂/夹爪限制、英文路径 | “第二块 ESP32”未能单靠照片证明就是手腕板，需核验角色 |
| 用户 `previous_context(1).md` | 参考 | ESP-NOW、两板分工、FastAPI、Vue3、流式记录/回放、deadman、超时与安全状态机 | 不接受摄像头/视觉闭环/自然语言/高级任务、不强制 5 轴、不强制 IK；不采用 React TSX/Recharts |

## 2. 核心架构决议（ADR）

### ADR-001：只做传感器参与的交互闭环，不伪造关节编码器闭环

**决定：** 用 MPU6050 测量手腕 Roll/Pitch，robot 用最新姿态更新关节**命令目标**，向后端回传 `q_requested/q_applied_target`。外部 `q_measured_deg=null`。实验报告使用“基于 IMU 的遥操作和软件限位/指令反馈”，不写“机器人关节位置实时实测闭环”。

**理由：** 当前 996 三线舵机只收到 PWM，未记录输出实际轴角的外部传感器；舵机内部控制与本系统能测量实体机械角度是两回事。此区分能避免记录/回放误判成功。

### ADR-002：四个通道，不是五轴演示底座

**决定：** `q[4] = [base, shoulder, elbow, gripper]`，映射 `[14,15,13,12]`。对外称“3 个空间姿态关节 + 夹爪执行器，共四路舵机”，避免“4DOF”把夹爪开/闭误认为独立定位自由度。

**理由：** AGENTS 明示四路接线；照搬 GitHub 五轴例程会导致错控甚至撞机。

### ADR-003：Arduino 最小驱动优先于完整 ESP-IDF 迁移

**决定：** 使用现有 Arduino Core 3.3.12、Adafruit MPU6050 + Adafruit_PWMServoDriver、ESP-NOW；借鉴开源项目模块分层与安全控制思想，不强行引入其整个 ESP-IDF Web 服务、NVS 学习模式、五轴 IK 代码。

**理由：** 现有离线开发环境已具备，MVP 的核心固件逻辑不大，而大工程适配成本和控制权冲突风险高；开源库直接复用可减少 I²C 底层错误。唯一需要新补的 PWM 驱动库要在第一阶段安装/离线缓存。

### ADR-004：关节空间控制代替 MVP XYZ 逆运动学

**决定：** Roll → 底座角速度；Pitch → 在 SHOULDER/ELBOW 模式下对应关节角速度；机器人端按独立硬限位/速率限制积分。可选在已测工作区用经验耦合表让末端径向移动更直观，但不作为验收前置条件。

**理由：** 只做固定工位示教回放，不需要先解出一般 3D 抓取位姿；未知连杆尺寸、舵机零位和负载误差会显著增加 IK 调试负担。输入的操纵方式不能保证真实物理末端直线移动，应如实说明。

### ADR-005：录像/回放一词只描述软件目标时间轴

**决定：** 示教录制是 `robot` 的已应用四路目标 + 时间戳 + 事件；回放按时间轴重建目标并按安全限制执行；物理抓放结果由人工判断。原始记录不可覆盖，改插值/减速生成新版本。

**理由：** 无关节编码器、无抓取检测/视觉，不可能提供完全一致的真实末端路径保证。回放的可靠性来自固定初始状态、慢速、软限幅和反复实测。

### ADR-006：控制权在机器人，调度权在 FastAPI

**决定：** `WEARABLE`（无线）与 `PC_MANUAL/REPLAY`（串口）经 robot 唯一 owner+lease 仲裁；FastAPI 独占 COM 串口。Vue 不与 ESP32 直接通信、不做 MCU 级保护。优先级：安全 STOP/FAULT > 已授权控制者 > 其他请求。

**理由：** 避免网页滑块、体感、回放同时生效。对通信失联、进程崩溃保持有限响应能力。

### ADR-007：Vue 3 + TS + Vite + Pinia + ECharts

**决定：** 前端单文件组件 `.vue`，REST/WS 数据源；图表 ECharts。旧上下文里 React 的 `.tsx` 与 Recharts 不接受。Three.js 目标模型仅可选，且标为非真实测量。

**理由：** 与明确技术栈一致，避免双前端代码基和额外集成负担。

### ADR-008：传感器失效后不允许继续积分；不能自动掉落或 Home

**决定：** 超时、IMU invalid、boot_id 变化、lease 丢失进入 HOLD/SAFE_BOOT；稳态目标停止，尽可能保持当前支撑 PWM；真实危险用实体断电并人工托臂。夹爪达到闭合超时只报警/阻止继续任务，不在空中自动松开。

**理由：** 承重臂无驱动时下垂；以“禁能电机”作为通用故障安全动作，反而可能立即造成物理事故。

## 3. 外部项目的可复用范围及许可证

| 参考项目/资料 | 可用部分 | 不应直接挪用 | 许可状态与注意事项 |
| --- | --- | --- | --- |
| [Linyings ESP32_WROOM_IDF_Rebot_wifi](https://github.com/Linyings/ESP32_WROOM_IDF_Rebot_wifi) | PCA9685/servo controller 的模块划分、轨迹和权限设计思路 | 五轴 CH0–4、假设的 Home/IK、完整 ESP-IDF Wi-Fi Web 服务 | README 声称 MIT，但 2026-11-12 核查 `main/LICENSE` URL 为 404；**在未取得可核对的许可文本前不直接复制源码** |
| [seihwan-Jung Wireless-IMU-Teleoperation-Robotic-Arm](https://github.com/seihwan-Jung/Wireless-IMU-Teleoperation-Robotic-Arm) | 双 ESP32 ESP-NOW、EMA/deadband/slew、事件帧分层思路 | 原项目用双 MPU6050 + 手指电位器 + 五舵机的具体映射 | 未核实到授权复制的 LICENSE；仅参考公开原理、自己实现适配代码 |
| [Aidan-Gildea robot-arm-3dof](https://github.com/Aidan-Gildea/robot-arm-3dof) | Python 串口/运动学作为 P1 参考；将来若补 XYZ 计划可复用合法代码 | 原五路舵机、非本装置连杆长度、角度偏置与直接 Home 行为 | GitHub `main/LICENSE` 可访问，MIT License（2026）；直接修改时须保留许可证与版权声明 |
| [Adafruit PCA9685](https://learn.adafruit.com/16-channel-pwm-servo-driver) | 正式 PWM 库、VCC/V+、I²C、OE、电气校准规则 | “所有舵机 0–180°”假设、把 VCC 当伺服供电 | 安装库并遵循其仓库 LICENSE、版本兼容性要求 |
| [Espressif ESP-NOW](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html) | 官方 packet/ack/seq/channel 语义 | “send success 即已经执行” | 芯片厂文档，不是可直接移植的完整应用 |
| [Vue 3 官方文档](https://vuejs.org/guide/quick-start.html) | Vite+TypeScript+Pinia/Router 创建流程 | React TSX/Recharts | 官方文档，构建依赖版本按当前工具链确认 |

**总体策略：** 借用成熟官方库、协议和能合法引用的代码；最小化**不相容的完整上游工程**整合。开源项目的 README 声明不能代替实际 `LICENSE` 文件的许可确认；不能用“GitHub 可以公开访问”推断允许复制全部源码。

## 4. 原 Plan / 历史建议的逐项口径统一

| 冲突或不清晰处 | 原描述 | 本次统一口径 |
| --- | --- | --- |
| 5 路 vs 4 路 | “五路舵机工作 / `[j0..j4]`” | 固定四路 `[base, shoulder, elbow, gripper]` |
| 通道映射 | 旧开源 CH0–CH4 | 实物 CH14/15/13/12，绝不自动重排 |
| 控制模式 | “前后倾斜前进后退”以及“可用关节模式” | 明确 SHOULDER/ELBOW 的关节速度，平面伸缩语义只是目标体验，待实验验证 |
| 复位 | “回中立停止新增目标，或复位” | **回中立停止新增增量**；Home 需要显式人工确认/独立动作 |
| 传感器闭环 | “基于传感器闭环 MVP” | 仅手腕 IMU 对输入姿态有传感器观测；舵机轴角不可观测 |
| 轨迹来源 | 手腕倾角或页面滑块推算 | 机器人已应用 PWM 逻辑目标，确保与底层限幅一致 |
| 按钮归属 | “第二块 ESP32 GPIO25/26” | 需要现场确认归属；目标角色是 wearable 同板接按钮/IMU |
| 机械臂自主 | “示教后无人实时操作自动复现” | 固定位置/手工放回积木，按预检记录回放；无人持续遥控≠无人值守 |
| Web 实现 | React/TSX 或 Recharts 混入旧方案 | Vue 3 `.vue`、Pinia、ECharts、FastAPI 唯一后端 |
| 前端视频 | 相机预览/物体识别 | 完全移除，甚至不包含占位摄像头组件 |
| 开机恢复 | 自动 Home 或自动续播 | SAFE_BOOT + 人工初态确认，不能自动续播 |
| 夹爪超时 | 机械限制夹紧时间 | 超时报警/停流程，不在空中自动开爪造成坠物 |

## 5. 不确定事项（保留为待现场决策，不凭照片捏造）

- 当前哪块 ESP32 同时接 MPU6050 与双按钮，另一块是否稳定通过 CH340 连 PC。
- PCA9685 的实际 I²C 地址、OE 有无上拉/是否已连接；是否提供能够控制输出默认断开的条件。
- 4 台舵机是否确为原厂 MG996R、实际安全电压和峰值电流、现有 DC 供电适配器额定值、端子可承载电流。
- 四个舵机角度零位、方向、脉宽极限、转速/加速度、重力负载能力、夹爪夹紧安全驻留时间。
- 从固定 A 点到 B 点可否在 3 个位姿关节（无腕俯仰独立关节）的情况下稳定抓起：若困难，只能选矮泡沫物体、约束夹爪接近方式或调整台面工位，**不自动引入第五轴**。
- 是否已有 `Adafruit_PWMServoDriver` 库的离线副本、前端 Node/npm/后端 pip 可用版本与网络能力。
- 考核是否明确要求 IMU 200Hz 原始采样：如果有，先保证腕端 200Hz **采样**，无线控制仍可 25–50Hz 下采样；记录路径的数据完整性与课程采样要求分开处理。

这些事项在文档中均以 `TODO`、`null` 或“建议值”表达。完成实际测量后更新 `AGENTS.md`、校准 JSON、测试报告，并提高校准版本，才可宣称项目达成预期。

## 6. 本次外部资料来源（直接链接）

- Espressif ESP-NOW（官方）：https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html
- Arduino-ESP32 ESP-NOW（官方）：https://docs.espressif.com/projects/arduino-esp32/en/latest/api/espnow.html
- Adafruit PCA9685 管脚与电源：https://learn.adafruit.com/16-channel-pwm-servo-driver/pinouts
- Adafruit PWM 库使用与舵机校准：https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all
- TowerPro MG996R 官方标称规格（需确认实物）：https://towerpro.com.tw/product/mg996r/
- Vue 3 官方：https://vuejs.org/guide/quick-start.html
- Vue 3 + TS：https://vuejs.org/guide/typescript/overview
- Pinia 官方：https://pinia.vuejs.org/introduction.html
- Aidan-Gildea 仓库 MIT 文本：https://github.com/Aidan-Gildea/robot-arm-3dof/blob/main/LICENSE
