# 真实 IMU 与虚拟机械臂控制台 · Attitude V3

虚拟执行端链路：MPU6050 → 腕端 ESP32 → USB → FastAPI → 虚拟控制器 → Vue / Three.js。输入来自真实 IMU，执行端为虚拟执行端；三维姿态与末端位置均为软件目标／估算预览，`measured=null`。当前界面没有模拟输入入口，也不会在断线时自动切换为模拟数据。

## 安装与启动

固定安装：芯片正面向下、放在手背上，传感器 X 向右、Y 从手背指向手肘、Z 向下。手腕坐标为指尖／右侧／下方，即传感器 −Y／+X／+Z。按本次佩戴实测，底座关节映射符号固定为 +1，Pitch 对大小臂的关节符号固定为 −1；控制坐标变换与关节映射符号是两层不同的约定。右倾使底座俯视顺时针，抬手使大小臂一起抬起，不提供安装或方向配置。

使用 Python 3.10+；本机使用 `.venv` 或 `py -3.13`，不要使用旧 Anaconda Python。

```powershell
cd E:\Code\MotionArm
$env:MOTIONARM_SOURCE = 'wearable'
.\.venv\Scripts\python.exe -m uvicorn app.main:app --app-dir backend --host 127.0.0.1 --port 8000
```

另一个终端：

```powershell
cd E:\Code\MotionArm\frontend
npm run dev
```

打开 <http://127.0.0.1:5173>，选择实际腕端端口。端口号不代表板卡身份；后端只读串口，不发送运动命令。Arduino 串口监视器与后端不能同时占用端口。旧 `mock/serial` 后端仍保留，当前页面会拒绝其遥测，不将模拟输入标为真实 IMU。

## 腕端固件与唯一 USB 协议

`firmware/wearable` 保留以下 USB att:3 诊断，同时运行 ESP-NOW v2；实体无动力接入见 [当前执行端说明](17_No_Power_Executor.md)。MPU6050 地址及 WHO_AM_I 为 0x68，SDA21/SCL22，100kHz；A=GPIO25，B=GPIO26，低电平消抖按下边沿计数。

```powershell
.\tools\build-local.ps1 -Action Build -Sketch wearable -ArduinoData C:\Users\21211\AppData\Local\Arduino15
```

新后端必须配套 att:3 固件。构建不等于已经烧录；确认腕端身份并关闭串口占用后，才使用实际端口执行 `-Action Upload -Sketch wearable -Port COMx`。

100Hz 采样，松开按钮静置约 2.5 秒完成 250 样本陀螺零偏校准；115200 8N1、25Hz、每行含换行最多 384 字节，整帧非阻塞发送。唯一字段为：

`v,att,role,boot,seq,ms,age_ms,who,valid,cal,q,gyro_dps,accel_g,a,b,ac,bc,norm_g,temp_c,errors,rej,gr,rc`

- `v=1, att=3, role=wearable`。拒绝缺失／旧姿态版本、未知或缺少字段，不接受旧 `roll/pitch/mode/short/long`。
- `q=[w,x,y,z]`：Hamilton 四元数，传感器到滤波世界坐标的姿态，六位小数。后端要求范数与 1 的误差≤0.01，接纳后归一化。
- `gyro_dps=[X,Y,Z]`：传感器原轴、扣除零偏后的 °/s，两位小数；`accel_g=[X,Y,Z]`：传感器原轴未经缩放修饰的 g，三位小数。
- `norm_g` 与 `temp_c` 保留原始诊断；`gr=2000` 表示陀螺量程±2000°/s。`errors` 为读取错误计数，`rc` 为拒绝样本计数。
- `rej`：0正常，1总线读取失败，2采样间隔超时，3加速度失效，4陀螺量程饱和，5解算失效。
- `a/b` 为电平；`ac/bc` 为 uint32 消抖按下计数，按住不重复、释放不触发、开机已按住不产生虚假点击。

实际序列化器与 C++ 测试共享 `WearableTelemetry.h`；最大字段宽度帧为 383 字节，25Hz 为 9575 bytes/s，低于 115200 8N1 的 11520 bytes/s。MPU6050 没有磁力计，航向仍可能漂移，也不测量手部平移。

## 初始化与固定参考跟随

1. 连接腕端，等待输入有效。双按钮释放，保持舒适手势约 0.5 秒。后端要求连续窗口覆盖≥480ms、相邻样本≤100ms、三轴角速度模长<5°/s、窗口内最大三维姿态距离≤3°；仅绕 Z 转动也不能通过。
2. 点击唯一入口“启用并回初始”。以符号对齐后的四元数均值建立本次会话参考，虚拟目标设为 0°/45°/45°、夹爪打开，等待 A 开始。尚未初始化时相对角显示“—”。
3. A 点击开始／暂停，松开不暂停；B 点击切换虚拟夹爪，暂停时也可操作 B。首次闭合起最多5000ms，包含闭合推进且不可续期；到期强制打开，打开完整并收到新鲜释放后才接受下一次闭合点击。没有模式切换或长按语义。
4. 按 `inverse(q_reference) ⊗ q_current` 计算相对姿态，再将 `[w,x,y,z]` 转成手腕坐标 `[w,−y,x,z]`，取 SO(3) 对数映射的 X/Y 分量作为 Roll/Pitch。Z 仅参与诊断与三维保护，不直接控制底座。
5. 默认 5° 软死区后按角度跟随；Roll 控制底座，Pitch 以 1:1 协同控制大小臂。固定倾斜时模型到目标后停稳，输出受速度和加速度约束。
6. 参考在暂停、触限、A/B 点击时保持不变。超限硬截住，退回有效角区后继续；回到启用手势请求回初始。相对总转角≥175°时保持转轴输出，回到参考≤90°后解除歧义。
7. 软件停止、输入无效／过期或重启会禁用控制并清除会话参考，不自动恢复；需要重新初始化。夹爪立即请求打开，打开推进独立于姿态启用状态。

后端统一提供 `can_enable` 与阻塞原因，前端不另建稳定判定。跟随、录制、历史回放期间禁止重新初始化。

配置仅包含 `deadzone_deg`、`limits_deg`、`max_velocity_deg_s`、`max_acceleration_deg_s2`，死区为 `(0,20]°`。默认限位底座±45°、大小臂0–60°，速度20/15/15°/s，加速度60°/s²。仅停止后可修改，均不代表实体安全标定。文件为 `data/virtual-config.json`，可用 `MOTIONARM_VIRTUAL_CONFIG` 指定；已知旧字段首次读取时移除并保存，保留有效运动参数。参考不持久化。

## 模型、诊断与历史

模型沿用 `config/robot_baseline.json`：底板宽90mm、肩轴高90mm、肩肘轴距115mm、肘至安装面65mm、安装面至爪尖90mm。模型Z向上、+X为底座前方；大臂为从竖直向下的角，小臂为相对大臂向下的角，夹爪为独立开合比例。竖直末端约(0,0,360)mm；45°启动末端约(236.3,0,171.3)mm。

折叠诊断区显示当前／参考四元数、手腕三轴相对旋转向量、传感器三轴角速度／加速度和拒绝原因。断线或过期值标为历史接收值，禁止用于控制。诊断展开和状态提示不自动重置相机。

新录制使用 `schema_version=5`、`controller_version=coordinated-angle-v3`，保存真实输入、按钮、映射状态、请求／输出目标、固定安装与参考四元数；实体记录另外保存配置哈希、双机身份、PWM阶段及人工观察。既有schema2/3/4保持原样。JSON 保留完整数据，CSV 展开四元数及三轴诊断，不含旧事件／模式列。最长10分钟，输入失效、断线、重启中断归档。

历史回放独占 REPLAY 控制权，离线也可运行。旧录制保持原数据：几何与角度约定明确的虚拟记录按已保存目标回放，不重新解算旧 IMU；语义不明确的记录只查看／导出。缺口不插值，中断记录不能连续自动回放。历史模式仍标注独立的实时腕端诊断。

## 接口与验证

`GET /api/status` 与 `/ws/telemetry` 使用 `schema_version=5`，包含 `input/imu/buttons/executor/control/joint_state/recording/replay`。`imu.quaternion` 是当前传感器姿态，`control.follow.reference_quaternion` 是会话参考，`delta_rotation_vector_deg` 是手腕轴三维相对旋转向量。`requested/output_target/measured` 保留统一接口，当前实测字段为 null。实体空载输出阶段见 [有动力接口](18_Powered_Empty_Trial.md)，历史只在停止并断开实体管理后虚拟回放。

- `GET /api/input/ports`；`POST /api/input/connect {port}`、`/disconnect`。
- `GET/POST /api/virtual/config`；`POST /api/virtual/enable`、`/stop`、`/reset`。旧 `/prepare` 与 `/neutral` 已移除。
- `GET /api/recordings`、`/{id}`、`/{id}/export?format=csv|json`；`POST /start {name}`、`/stop`。
- `POST /api/replays/preflight {record_id}`、`/start {preflight_id}`、`/stop`。

250ms 无新有效输入禁用控制，1秒无合法帧进入重连；控制循环停顿>100ms暂停跟随。新鲜度基于本机单调时钟，不跨设备相减时钟。

```powershell
$env:PYTHONPATH = 'backend'
.\.venv\Scripts\python.exe -B -m unittest discover -s backend/tests
cd frontend
npm test
npm run build
```

C++ 主机回归位于 `firmware/tests`，覆盖姿态滤波、按钮和实际帧序列化。自动测试使用夹具，不构成产品模拟输入模式。腕端烧录、佩戴方向动态验证和物理拔插必须单独记录；不能替代传感器精度或实体带载抓放验收。

旧版二维旋转向量、可配置安装、按钮模式和现场历史记录见[Att3之前的归档](archive/Virtual_Console_Pre_Att3.md)，不作为当前操作依据。

## 2026-11-19 软件验证

- 后端62项、前端15项回归通过，Vue类型检查和生产构建通过；保留既有Three.js分包体积提示。
- C++姿态、按钮、序列化三组回归通过；ESP32核心3.3.12编译通过，Flash303088字节、RAM23708字节。未烧录，未操作串口。
- 浏览器在独立临时数据库、腕端未连接状态下检查1440×1000与390×844布局，无横向溢出；初始化锁定、安装只读和诊断展开正常，相机适配计数未因诊断展开改变，未发现控制台错误。
- 追加170°→190°跨过175°保护带的回归，防止25Hz采样跳过保护区；连续q/−q符号翻转不误触发保护。
- 本次无真实动态IMU、佩戴方向或物理拔插验收，不涉及实体执行端动作。

## 2026-11-19 腕端烧录与真实输入接通

- 烧录前释放后端串口占用，通过烧录器确认COM3为ESP32-D0WDQ6 rev1.1，MAC `EC:E3:34:EF:81:64`，与仓库腕端身份一致。
- 复用已验证的att:3编译产物（当前源码仅注释不同），115200上传成功，所有写入区段哈希校验通过，已硬复位。
- 上传后独立只读取得125帧att:3，平均间隔40ms、最大315字节；WHO_AM_I=0x68、250/250校准、valid=1、I²C错误0、rej=0。随后关闭探测串口，仅由FastAPI占用COM3。
- FastAPI与Vue已在127.0.0.1:8000／5173启动，网页显示真实输入有效。初始保持未启用、无会话参考，等待佩戴完成后初始化。
- 原始加速度模长约1.21g，保留现有精度诊断提示；本条不代表传感器精度通过。佩戴方向、A/B动态控制及物理拔插验收尚待配合验证。

## 2026-11-19 佩戴方向与双按钮现场验证

- 用户确认佩戴完成并静置后，通过三维稳定门控采集参考；真实A点击开始跟随，松开后运行状态保持。
- 首次右倾实测Roll为负（观察至−34.58°），原底座符号−1使请求角为正；用户确认俯视为逆时针，与预期相反。仅将固定底座关节符号修正为+1，保留−Y／+X／+Z控制坐标变换与Pitch符号−1。
- 独立抬手观察258个状态样本全部有效：Pitch最高约+19.13°，大小臂请求同步从45°降至30.87°；用户确认一起抬起，Pitch方向通过。
- 按钮观察捕获A计数从1增至2并进入暂停；B计数依次1、2、3，夹爪每次切换，暂停期间也可开合。该窗口257个样本全部有效，参考不变。
- 方向修正后，后端重启并重新初始化；未再次烧录腕端。右倾复测269个状态样本全部有效，Roll最低约−29.74°、底座请求最低约−24.74°；用户确认俯视顺时针且回参考正常。参考全程不变，A计数增至4后暂停。
- 增加现场值方向回归，后端63项、前端15项及生产构建通过。结束时前后端继续运行、真实IMU在线、控制已暂停、虚拟夹爪打开。
- 本次确认了右倾、抬手与双按钮交互；物理拔插、绝对姿态精度、航向长时漂移和实体执行端运动不在本次验收结论内。
