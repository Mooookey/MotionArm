# 07｜分工、任务依赖、实现顺序与交付追踪

> 本文是团队执行清单，不是外部服务的任务管理器。工期是估算工程量，用于排序，不构成已经实施或能保证完成日期的承诺。全部源码/构建路径使用英文。硬件烧录/串口访问必须由**一个操作者独占**，避免多个进程同时连 COM、重复烧录或同时控制机构。

## 1. 工作包拆解与依赖

| ID | 优先级 | 任务与建议负责人角色 | 前置依赖 | 关键产出 | 关闭条件 |
| --- | --- | --- | --- | --- | --- |
| HW-01 | P0 | 确认两板角色/接线：硬件负责人 | 无 | 角色表、MAC/COM、实物接线照片 | 两块板与模块归属准确 |
| HW-02 | P0 | 电源容量/极性/共地/开关：硬件负责人 | HW-01 | 安全电气检查表 | 可物理断伺服动力、有支撑 |
| HW-03 | P0 | 机械底座固定/空间边界：硬件负责人 | HW-02 | 桌面工位示意、重力支架 | 人员与线缆不进运动区 |
| FW-R01 | P0 | robot I²C 与 PCA9685 驱动：固件负责人 | HW-02 | 可识别 0x40 并选择 CH12–15 的测试固件 | 单通道可控、无错映射 |
| FW-R02 | P0 | 四关节脉宽/方向/软件限位标定：硬件+固件 | FW-R01,HW-03 | `robot_calibration.json` 和 hash | 四路小步通过、未知值无法 ARM |
| FW-W01 | P0 | wearable MPU6050 探测/静态校零：固件负责人 | HW-01 | `imu` 驱动及有效性标志 | 0x68 读取正确，异常标 invalid |
| FW-W02 | P0 | 姿态滤波/neutral/双按钮：固件负责人 | FW-W01 | Roll/Pitch + mode + deadman + gripper event | 静止/短长按测试 |
| FW-W03 | P0 | 两板 ESP-NOW 传输：固件负责人 | FW-W02,FW-R01 | 24B v1 帧/CRC/boot/seq | 丢包/重复包不重复触发 |
| FW-R03 | P0 | robot owner/状态机/看门狗：固件负责人 | FW-W03,FW-R02 | TELEOP/READY/REPLAY/HOLD/FAULT | 切换互斥、250ms 初值超时有效 |
| FW-R04 | P0 | 50Hz 关节增量+限速+夹爪保护：固件负责人 | FW-R03 | 关节控制器与测试 | deadman/限幅/爪驻留通过 |
| PR-01 | P0 | Serial NDJSON v1 双端协议：协议+固件 | FW-R03 | HELLO/ACK/STATE/COMMAND | 编码/版本/错包测试 |
| BE-01 | P0 | PySerial 唯一连接器/设备识别：后端负责人 | PR-01 | serial_bridge | 不写死 COM，重连无误动作 |
| BE-02 | P0 | 命令代理/lease/状态缓存：后端负责人 | BE-01 | command_broker + telemetry_cache | 异常命令/双端抢权被拒 |
| BE-03 | P0 | FastAPI 基础端点与 WS：后端负责人 | BE-02 | status/control/ws | Mock 接口/断线测试通过 |
| UI-01 | P0 | Vue3 TS Vite 基础/Pinia/Router：前端负责人 | 03 协议定义 | 初始化页面、typed client | npm 构建/单测通过 |
| UI-02 | P0 | 状态条/IMU/4路目标与事件图：前端负责人 | UI-01,BE-03 | Dashboard | Offline/stale/目标与实测标注 |
| BE-04 | P0 | SQLite schema/写入/导出：后端负责人 | BE-03 | schema migration、repository | 数据 roundtrip 与异常归档 |
| BE-05 | P0 | Recording 状态机/事件关联：后端负责人 | BE-04,FW-R04 | recording service | 开始/结束/中断可追踪 |
| UI-03 | P0 | Teaching 交互与轨迹列表：前端负责人 | UI-02,BE-05 | /teaching 与列表 | 录制中断/保存有正确提示 |
| BE-06 | P0 | Replay preflight/插值/lease：后端负责人 | BE-05,FW-R04 | replay service | 错轨迹不能执行，STOP 可抢占 |
| UI-04 | P0 | Replay 预检/确认/时间轴：前端负责人 | UI-03,BE-06 | /replay | 失败预检不能启动 |
| QA-01 | P0 | 整体固定工位轻载测试：测试/硬件 | HW-03,UI-04,BE-06 | 10轮记录、问题清单 | 达到 A01 或如实记录边界 |
| QA-02 | P0 | 掉线/重启/双操作者/异常角等故障测试：测试/固件 | QA-01 前先做空载安全版本 | 验收与故障注入报告 | 所有 P0 异常行为可解释 |
| UI-05 | P1 | 轨迹比较、二维/三维**指令目标**示意：前端 | UI-04 | 可视化增强 | 不延迟 P0，明确非实测 |
| BE-07 | P1 | 原始 IMU 记录/派生轨迹分析：后端 | BE-05 | CSV/分析脚本 | 数据来源/时间基准明确 |

## 2. 关键路径（不要同时开工所有模块）

```text
HW-01 → HW-02 → HW-03 → FW-R01 → FW-R02 → FW-R03 → FW-R04
                              ↑                ↑
                  FW-W01 → FW-W02 → FW-W03
                                                    ↓
                                               PR-01 → BE-01 → BE-02 → BE-03
                                                                        ↓
                                                         BE-04 → BE-05 → BE-06
                                                             ↓         ↓
                                             UI-01 → UI-02 → UI-03 → UI-04
                                                                    ↓
                                                               QA-01 / QA-02
```

M0–M1 必须由实物串口操作者主导；UI-01 可在硬件标定期间用 Mock JSON 平行开发，但不得提前认为真实串口格式已定。BE-04 SQLite 可与 UI-01 平行实现，前提是协议 v1 和字段语义冻结。

## 3. 建议迭代节奏（按 Gate，而不是固定日历）

| 迭代 | 主要工作 | 粗略人力投入（人天，非承诺） | 演示目标 |
| --- | --- | --- | --- |
| I0 | 确认接线、电源、支撑、校准表 | 1–3 | 四路电路安全，I²C 可探测 |
| I1 | 单伺服 + IMU + 按键诊断 | 2–4 | 每路独立小步转动、姿态稳定 |
| I2 | ESP-NOW + robot 控制状态机 | 3–6 | deadman 体感遥操作可安全停止 |
| I3 | USB 协议 + 后端 + Vue 状态台 | 3–6 | 姿态/关节/异常状态实时可视 |
| I4 | SQLite recording + replay preflight | 3–6 | 固定初态记录和空载复现 |
| I5 | 轻载抓放、调速/夹爪与故障演练 | 3–7 | A/B 固定搬运 10 轮和完整报告 |

上述粗估约 15–32 人天，可多人交叉开展但硬件阶段不允许并行操控同一设备。强制先过上一 Gate：如果 M1 因供电抖动失败，减少前端功能不会解决根因。

## 4. 每个任务的 DoD（Definition of Done）

可将以下模板复制到 issue/PR：

```markdown
## TASK-ID
- [ ] 代码或标定文件在英文路径的预期目录中，未绕过现有结构
- [ ] 所有新增参数声明单位与校准来源，禁止通用 0–180° 伪限位
- [ ] 单元测试/Mock 通过（若为软模块）
- [ ] 固件用 ESP32 Core 3.3.12 实际编译（若涉及固件）
- [ ] 接线与测试前置安全检查通过（若需通电）
- [ ] 协议版本/配置哈希变化已反映在 docs 与 API
- [ ] 故障/断连/超时行为已测试
- [ ] 提供可重复的验证操作与结果文件
- [ ] 代码 review：没有其他控制者同时拥有 PWM 权限
```

接口变更要同时更新：`docs/03_Communication_Protocol.md` → firmware 编解码 → backend schemas → Vue TypeScript types → 协议兼容测试。不能先在某端“临时加一字段”却不更新另一端。

## 5. Windows 工作站与离线环境操作路径

AGENTS 所述工具链/脚本是**已记录的现有环境**，应尽量保留，不在未经实机验证时假设本地 Windows 的路径都仍然存在。

```powershell
# 只在已有工作区的实际位置执行
.\tools\esp32.ps1 -Action Check

# 分别对实际存在的 sketch 构建；以下是拟建英文目录示例
.\tools\esp32.ps1 -Action Build -Sketch E:\Code\robot_teleop_mvp\firmware\wearable_esp32
.\tools\esp32.ps1 -Action Build -Sketch E:\Code\robot_teleop_mvp\firmware\robot_esp32

# 上传必须根据 HELLO / MAC 与 Windows 设备管理器确认正确 COM；不预设 COM3
.\tools\esp32.ps1 -Action Upload -Sketch E:\Code\robot_teleop_mvp\firmware\robot_esp32 -Port COMx
```

示例 `COMx` 是占位符，不应直接执行；用实际端口替换且同一时刻**只能有一个进程打开该 COM**。原工作区命令 `-Sketch E:\Code\emb-exam\YourSketch` 仍可用作检查脚本语义，不能推断未来工程也位于该目录。

**必须补充的固件库：** `Adafruit_PWMServoDriver`（或团队已验证的等价 PCA9685 库）。`AGENTS.md` 列出的五个 Adafruit 库里没有明确它；离线状态应先核对已缓存库，避免现场才发现不可编译。再建立 `backend/.venv` 与前端 Node/npm 依赖锁文件，固定项目特定版本。

## 6. 多人协作边界

**硬件/固件负责人：** 唯一执行烧录、读 UART、机械臂通电、功率测试。负责硬件标定、ESP-NOW/实时控制、故障与限位。任何合并修改 PWM 映射的 PR 均需其 review。

**后端负责人：** 独占运行中串口的 PySerial 连接；提供 FakeSerial，使其他开发者不用接设备也能测试 API、记录和回放。对校准版本、轨迹完整性、lease 负责。

**前端负责人：** 仅对接 API/WS Mock；通过 Pinia 展示设备与命令状态，不得写自行猜测的舵机串口调用。控制禁用规则与后端状态机对应。

**测试/演示负责人：** 固定工位、重复抓放与证据、每个故障用例“安全注入”记录；判定完成与否并安排现场监护。团队人数不足时一人可兼任多角色，但不能同时以多个程序访问实物串口。

## 7. 依赖风险与减配原则

- 如果 I²C/供电不稳定：暂停所有上层功能，以 H01/H02 为阻断问题处理，不通过算法掩盖电源重启。
- 如果 996 夹爪过热：换更软/更小的泡沫块、减少夹取时间、增大机械安全余量，必要时限制演示为不夹物的空载轨迹；绝不无限增加闭合脉宽。
- 如果姿态长期漂：只做 Roll/Pitch + 中立重校 + 死区，禁用 Yaw/手部空间位移，不引入视觉补偿。
- 如果 IK 难调：**MVP 本来就不依赖 IK**，继续使用两种关节模式完成固定 A/B 录制与复现。
- 如果三维模型难做：先完成文字状态、滑块、IMU/目标曲线、回放任务，Three.js 是 P1。
- 如果 10 次成功率低于 8：记录机械失败原因，优先调供电/夹爪/初态/速度，再优化 UI；无反馈时绝不许报告“完全闭环定位精度”。
