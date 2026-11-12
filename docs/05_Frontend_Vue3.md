# 05｜Vue 3 前端设计：实时遥操作监控、示教与回放

> **唯一前端框架：Vue 3**。使用 TypeScript、Vite、Composition API、Pinia（共享状态）、Vue Router（页面路由）、Apache ECharts（曲线）；不采用 React、Next.js、Recharts，也不保留旧方案的 TSX/React 文件结构。屏幕不含摄像头画面、物体识别或自然语言任务输入。

## 1. 页面目标与交互原则

PC 界面不仅要发命令，更应该让操作员清楚知道**哪些数据是传感器测到的，哪些只是控制器期望值**：

- `Roll/Pitch`：MPU6050 解算后的佩戴姿态（可视为手腕传感器估计值）。
- `q_requested_deg`：控制者希望机器人执行的关节目标。
- `q_applied_target_deg`：机器人固件经限位/速率限制后**已经用于 PWM 输出**的指令目标，不是实际关节角。
- `q_measured_deg`：本硬件没有外部关节反馈，必须显示“未配备/不可用”，不得用目标值冒充。
- `state/owner/fault/age`：设备状态与控制权限，决定按钮能否操作。
- `recording/replay`：正在执行的工作流，必须有明确起止与失败信息。

用户操作顺序遵循：**读状态 → 明确准备 → 允许控制 → 演示 → 放弃控制/停止 → 查看数据**。绝不在页面加载/刷新、WebSocket 重连或组件挂载时自动 ARM、HOME 或 REPLAY。

## 2. 推荐页面布局与功能

### 2.1 总览 `/` 或 `/dashboard`

```text
┌─────────────────────────────────────────────────────────────────────────┐
│ Mini Teleop MVP       机器人: ONLINE  手腕: ONLINE  状态: TELEOP        │
│ Owner: WEARABLE       校准: c1       软 STOP  [提示：硬件断电开关位置]   │
├─────────────────────────────┬───────────────────────────────────────────┤
│ IMU Roll/Pitch             │ 关节软件目标（非实体实测）                  │
│ Roll: -06.1°              │ 底座 BASE      requested/out             │
│ Pitch: +12.4°             │ 大臂 SHOULDER   requested/out             │
│ 模式 SHOULDER/ELBOW        │ 小臂 ELBOW      requested/out             │
│ DEADMAN [按住/释放]       │ 夹爪 GRIPPER    open/close/holding         │
│ 最近有效姿态 32ms         │ 真实轴角：未配备位置反馈                   │
├─────────────────────────────┴───────────────────────────────────────────┤
│ ECharts：Roll / Pitch / 四关节软件目标（可切曲线、暂停图表）           │
├─────────────────────────────────────────────────────────────────────────┤
│ 记录：IDLE   [开始录制]   [结束并保存]                                │
│ 回放：IDLE   [选择录制]   [预检]  [开始回放]  [停止]                  │
├─────────────────────────────────────────────────────────────────────────┤
│ 事件：连接/断线、标定、模式变化、夹爪、停止原因、限位命中              │
└─────────────────────────────────────────────────────────────────────────┘
```

默认需要大尺寸状态条和显眼的“STOP（软件停止）”，并且在旁明确提示“物理急停靠断开伺服电源”；不要通过颜色暗示软件 STOP 可以阻止重力下垂。

### 2.2 手动调试 `/manual`

- 默认只读显示四个关节的**软件目标**；若要操作滑块，先点击“请求 PC_MANUAL 控制权”，弹出检查确认（桌面净空、承重支撑、无回放、已标定）。
- 后端返回 lease 成功前滑块只能预览，不得发有效命令；若 owner 为 WEARABLE/REPLAY，页面显示冲突并禁止接管。
- 每次只能发送**完整四元组目标**，不得单独发送一个没有完整背景的 `[shoulder]` 数值。输入范围来自后端当前标定，不能前端默认 0–180°。
- **按住移动**优于自由拖动：拖动期间以低频（如 10–20Hz）更新，鼠标/触控松开即释放运动意图；不能让页面滞后时积压一串目标。最终的角度限幅和停止策略都由 `robot` 固件执行。
- Home 按钮如保留，必须标为“移动到已标定 Home（需路径检查）”，二次确认，且没有确定安全路径时默认不可用。

### 2.3 示教页面 `/teaching`

示教只在 WEARABLE/TELEOP 期间进行：

1. 页面显示设备在线、IMU valid、`owner=WEARABLE`、当前校准版本、固定工作区 `workcell_id`。
2. 输入录制名称（例如 `foam_block_A_to_B_01`）与实验备注，点击“开始录制”。
3. 显示运行计时、接收有效采样数、最近帧年龄、夹爪事件、掉帧/故障统计；采样异常实时提示“该记录可能不可回放”。
4. 点击“结束并保存”后等待后端数据库确认，显示录制 ID、持续时间、记录/事件数、完整性和缺帧范围。
5. 录制被中断时显示“已归档但不可自动回放”，保留数据供排障，不能让 UI 用绿色成功标签覆盖实际错误。

### 2.4 回放页面 `/replay`

- 左侧轨迹库：名称、创建时间、运行时长、样本数、事件数、`calibration_id`、`status=completed/invalid`。
- 选择后显示轨迹预览（四路 setpoint 曲线、夹爪事件与时间轴），并明确“这不是机器人实测位置记录”。
- 点击“预检”调 `/api/replays/preflight`：显示机械臂当前软件初态/录制初态之差、版本是否相同、是否有 gap、夹爪最大驻留、速度与安全错误。
- 仅在 preflight 为 OK 且操作员勾选实体工位/泡沫块/机械臂起点/工作区检查后，才能提交 `preflight_id` 启动回放；不能用预检失败时的“仍然继续”选项绕过硬门槛。
- 回放过程中展示进度、当前阶段/事件、已应用目标与计划目标误差，始终保留停止按钮。回放结束状态显示 `命令轨迹执行完成`；再由操作员手动填写物理结果 `成功 / 失败 / 未确认`。

### 2.5 实验与设置 `/experiments`, `/settings`

- 实验页展示次数、人工成功率、失败原因（夹持滑脱、初态误差、连接中断等），能从每次实验查看相关轨迹与 replay_run。
- 设置页只读展示四路通道对应关系、软限位、最大速率、校准哈希、固件版本与设备 MAC，避免普通用户通过网页直接篡改硬件校准。
- 调试/专家校准若后续做独立页面，默认禁用且必须显式解锁、限幅及人工观护，绝不能通过普通设置页面写任意 PWM µs。

## 3. Vue3 推荐源代码布局

```text
frontend/
├── index.html
├── vite.config.ts
├── package.json
├── src/
│   ├── main.ts
│   ├── App.vue
│   ├── router/index.ts
│   ├── types/{telemetry,recording,replay}.ts
│   ├── api/{client,control,recordings,replays}.ts
│   ├── stores/{device,control,recording,replay}.ts
│   ├── composables/{useTelemetrySocket,useRingBuffer}.ts
│   ├── components/
│   │   ├── ConnectionBanner.vue
│   │   ├── SafetyStatus.vue
│   │   ├── ImuGauge.vue
│   │   ├── JointTargets.vue
│   │   ├── TelemetryChart.vue
│   │   ├── TeachRecorder.vue
│   │   ├── ReplayPreflight.vue
│   │   ├── ReplayTimeline.vue
│   │   ├── EventLog.vue
│   │   └── SoftwareStopButton.vue
│   └── views/{DashboardView,ManualView,TeachingView,ReplayView,ExperimentsView}.vue
└── tests/{stores,api,components}/
```

`Vue3 + Vite` 官方脚手架直接提供 TypeScript 与 Pinia/Router 选项，代码使用单文件 `.vue` 组件及 `<script setup lang="ts">`。ECharts 可直接在 Vue 组件中 `echarts.init(dom)` 并在 `onBeforeUnmount` 时 `dispose()`；若加 `vue-echarts`，需固定版本并确认 Vue 3 兼容。**不要混用 Recharts**（React 图表库）。

## 4. 数据源与 Store 类型契约

```ts
export type ControlState =
  | 'SAFE_BOOT' | 'READY' | 'TELEOP' | 'PC_MANUAL'
  | 'RECORDING_TELEOP' | 'REPLAY' | 'HOLD' | 'FAULT' | 'ESTOP_LATCHED'

export type ControlOwner = 'NONE' | 'WEARABLE' | 'PC_MANUAL' | 'REPLAY'

export interface JointSnapshot {
  names: readonly ['base', 'shoulder', 'elbow', 'gripper']
  requested_deg: [number, number, number, number]
  applied_target_deg: [number, number, number, number]
  measured_deg: null  // 外部关节位置传感器未配置，不可用数组伪造
}

export interface DeviceSnapshot {
  robotOnline: boolean
  wearableOnline: boolean
  lastPacketAgeMs: number | null
  controlState: ControlState
  controlOwner: ControlOwner
  faultCode: string | null
  imuValid: boolean
  deadman: boolean
  rollDeg: number | null
  pitchDeg: number | null
  joints: JointSnapshot | null
  calibrationId: string | null
}
```

`deviceStore` 只通过 `/api/status` 初始拉取 + `/ws/telemetry` 更新。`controlStore` 管理后端返回的 owner/lease/操作请求，不得本地把“已发出请求”直接推断成“已取得控制权”；`recordingStore` 与 `replayStore` 保存各流程状态，任何网络故障先冻结按钮并查服务端真实状态。

## 5. WebSocket、图表和弱网行为

### 5.1 连接管理

```text
page enters:
  GET /api/status
  connect /ws/telemetry
on message:
  validate schema, reject clearly stale snapshots
  write Pinia deviceStore
  append to bounded time-series buffers
on close/error:
  mark stale/disconnected
  disable move/record/replay start
  show hardware STOP notice
  reconnect only for READ-ONLY status (exponential backoff)
  NEVER re-send last motion POST or resume replay automatically
on page leaves:
  dispose chart, cancel subscriptions and request timers
```

网页是否连接不应该改变已经被 `robot` 允许的 WEARABLE 体感控制；仅影响 PC 控制入口。浏览器离开/崩溃期间后端继续记录的过程由服务端状态决定，Vue 重新连接后必须先读取 server snapshot，不假定旧页面缓存是真实状态。

### 5.2 曲线性能

推荐显示 30–60 秒窗口，可切到 5 分钟，数据按 `server_time_ms` 排序但不要假设它代表机器人执行时刻。一般 10–25 Hz WS 推送已足够可视化，曲线保留环形缓存（例如 25Hz×300s≈7500 点/通道），图表更新频率 10Hz 或更低，丢弃旧图表帧不影响底层记录；**前端缓存不得承担示教数据持久化**。

一张 Roll/Pitch 曲线、一张四关节 setpoint 曲线和可选的按钮/事件时间轴已经足够。若使用 2D/3D 示意图，形状需与真实机械臂结构相近，**未测量连杆尺寸时只显示示意，不计算假精度末端 XYZ**。

## 6. 操作按钮使能矩阵（前端 UX，不代替后端校验）

| 状态 | 手动滑块 | 开始录制 | 停止录制 | 预检回放 | 启动回放 | 软件 STOP |
| --- | --- | --- | --- | --- | --- | --- |
| `SAFE_BOOT` | ✕ | ✕ | ✕ | 查看报告可 | ✕ | 可请求 |
| `READY` | 获取 PC_MANUAL 后 | ✕ | ✕ | ✓ | 经 preflight / 人工确认 | ✓ |
| `TELEOP` | ✕ | ✓ | ✕ | 只读 | ✕ | ✓ |
| `RECORDING_TELEOP` | ✕ | ✕ | ✓ | ✕ | ✕ | ✓ |
| `PC_MANUAL` | ✓（有租约） | ✕ | ✕ | ✕ | ✕ | ✓ |
| `REPLAY` | ✕ | ✕ | ✕ | ✕ | ✕ | ✓ |
| `HOLD/FAULT/ESTOP` | ✕ | ✕ | 已异常结束 | 查看报告可 | ✕ | ✓（若线路可用） |

控件置灰时写清拒绝原因。特别注意：**软件 STOP 不等于实体断电**，如果串口已掉线，按钮点击可能无法送达机器人，因此必须同时突出物理开关位置和人工托臂提示。

## 7. 推荐工作站开发命令

在**英文路径**创建工程，例如 `E:\Code\robot_teleop_mvp`，避免之前中文路径造成构建/链接失败的问题。以下是官方 Vue 3 脚手架命令，执行前确认本机 Node 版本满足官方要求并能获取 npm 包（离线环境需要已缓存/提前下载依赖）。

```powershell
cd E:\Code\robot_teleop_mvp
npm create vue@latest
# project name: frontend
# TypeScript: Yes; Router: Yes; Pinia: Yes; Vitest: Yes; ESLint: Yes
cd frontend
npm install
npm install echarts
npm run dev
npm run build
```

开发期可用 `vite.config.ts` 将 `/api` 和 `/ws` 代理到本地 `127.0.0.1:8000`，部署时将静态资源交由 FastAPI 或一个本地前端服务提供，**无需增加 ESP32 自带 Web 服务器**。

## 8. 前端测试验收

- 使用录制好的模拟遥测 JSON，所有页面在没有硬件时可展示；检测实时值更新、红黄告警、无命令时不会意外产生请求。
- 模拟 WebSocket 断开，动作用控件立即禁用；网络恢复后只恢复可视化订阅，不自动恢复 REPLAY。
- 两浏览器窗口同时操作：服务端只授予一个 lease，第二页明确显示 409 冲突；前端不能覆盖后端最终决定。
- 校准不匹配、preflight 失败、物体没放回 A 点（人工未勾选）时回放启动按钮不可用。
- 夹爪只在 TELEOP/PC_MANUAL 的正确控制条件下可请求；长按模式与短按夹爪事件状态被正确显示。
- UI 提示 `applied_target_deg = 机器人输出软件目标`，`measured_deg = 未配备`，不出现“实际角度已达目标”的虚假显示。
- `npm run build`、TypeScript 检查、单元测试通过，图表组件反复切换无持续内存增长。

## 9. 官方参考

- Vue 3 官方快速开始及脚手架：https://vuejs.org/guide/quick-start.html
- Vue 3 与 TypeScript：https://vuejs.org/guide/typescript/overview
- Pinia：https://pinia.vuejs.org/introduction.html
- ECharts：https://echarts.apache.org/en/index.html
