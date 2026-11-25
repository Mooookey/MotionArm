# 当前本机启动与执行端入口

保留 [无动力执行端](17_No_Power_Executor.md)，新增 [小范围空载实体联调](18_Powered_Empty_Trial.md)。准备、烧录与检查时舵机动力关闭；首次使能须现场确认托住和对齐。禁止运行归档串口抓放演示。

## 启动

```powershell
cd E:\Code\MotionArm
$env:MOTIONARM_SOURCE = 'wearable'
.\.venv\Scripts\python.exe -m uvicorn app.main:app --app-dir backend --host 127.0.0.1 --port 8000
```

另一个终端：

```powershell
cd E:\Code\MotionArm\frontend
npm run dev -- --host 127.0.0.1
```

打开 http://127.0.0.1:5173 。默认虚拟执行端，选择腕端实际端口连接；初始化后保持暂停。实体模式需明确选择，先停止控制/录制/回放，再选择robot实际端口。连接、刷新和重启不自动初始化；有动力模式按准备、现场确认、PWM保持、单轴检查、初始化的顺序进行。旧`MOTIONARM_SOURCE=serial`仅进入无动力适配，不启用旧实体PC_MANUAL或回放接口。

腕端USB att:3可独立诊断；实体目标只由robot接收ESP-NOW并本地计算，不能用腕端USB计算结果冒充robot接纳。每个串口由一个进程独占，Arduino串口监视器、后端和独立检查脚本不能同时占用。

## 构建与检查

```powershell
.\tools\build-local.ps1 -Action Build -Sketch wearable -ArduinoData C:\Users\21211\AppData\Local\Arduino15
.\tools\build-local.ps1 -Action Build -Sketch robot -ArduinoData C:\Users\21211\AppData\Local\Arduino15
# 显式选择有动力试验镜像；构建不会解锁输出。
.\tools\build-local.ps1 -Action Build -Sketch robot -RobotMode powered -ArduinoData C:\Users\21211\AppData\Local\Arduino15
```

使用英文TEMP构建目录和离线库。默认robot构建仍为永久锁定的无动力镜像；有动力镜像必须显式选择。烧录前核对MAC，复用对应编译产物；COM编号可能变化。无动力robot使用USB v2/115200，有动力robot使用USB v3/230400；版本或模式不匹配须拒绝，不能回退接管。输出仍为FULL_OFF时不表示现场动力电压已测得为零。

新版兼容入口 `tools/esp32.ps1` 同样仅允许当前wearable/robot，归档演示不进入常规构建；构建和日志留在英文TEMP，不写源码目录。上传复用已有产物时使用 `-ReuseBuildDirectory` 指向本次构建返回的目录，仍须匹配模式、源码和板卡身份。

本机Python使用仓库.venv，不使用旧Anaconda。验证命令：

```powershell
$env:PYTHONPATH = 'backend'
.\.venv\Scripts\python.exe -B -m unittest discover -s backend/tests
.\.venv\Scripts\python.exe -B firmware/tests/test_profile.py
.\.venv\Scripts\python.exe -B firmware/tests/test_controller_vectors.py
cd frontend
npm test
npm run build
```

C++测试用本机g++编译到英文TEMP路径，包含实际协议、控制核心和遥测序列化；测试夹具不是产品模拟输入。无动力检查结束须STOP并确认owner=NONE、参考清空、软件夹爪打开、输出锁定；有动力结束先STOP，按现场支撑确认流程释放PWM及断动力。未完成按钮、佩戴或物理拔插时如实标为待测。
