# 当前本机启动与无动力入口

操作边界及验收步骤见 [无动力执行端](17_No_Power_Executor.md)。舵机动力关闭，禁止运行归档串口抓放演示。

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

打开 http://127.0.0.1:5173 。默认虚拟执行端，选择腕端实际端口连接；初始化后保持暂停。切换到“实体执行端·无动力联调”时先停止控制/录制/回放，再选择robot实际端口。连接、刷新和重启不自动初始化。旧`MOTIONARM_SOURCE=serial`现在也进入新版无动力适配，不启用旧实体PC_MANUAL或回放接口。

腕端USB att:3可独立诊断；实体目标只由robot接收ESP-NOW并本地计算，不能用腕端USB计算结果冒充robot接纳。每个串口由一个进程独占，Arduino串口监视器、后端和独立检查脚本不能同时占用。

## 构建与检查

```powershell
.\tools\build-local.ps1 -Action Build -Sketch wearable -ArduinoData C:\Users\21211\AppData\Local\Arduino15
.\tools\build-local.ps1 -Action Build -Sketch robot -ArduinoData C:\Users\21211\AppData\Local\Arduino15
```

使用英文TEMP构建目录和离线库。烧录前核对MAC，复用对应编译产物；COM编号可能变化。新robot必须以USB v2握手报告robot_no_power、正确MAC和output_locked=true；旧固件应显示不兼容，不回退到旧控制。

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

C++测试用本机g++编译到英文TEMP路径，包含实际协议、控制核心和遥测序列化；测试夹具不是产品模拟输入。现场测试完成后STOP并确认owner=NONE、参考清空、软件夹爪打开、输出仍锁定。未完成按钮、佩戴或物理拔插时如实标为待测。
