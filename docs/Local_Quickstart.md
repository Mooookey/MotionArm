# 当前本机入口

## 真实 IMU 三维虚拟控制台（默认）

使用 Python 3.10+（本机 `py -3.13`），按 [真实 IMU 与虚拟执行端说明](16_Virtual_Console.md) 启动 FastAPI 和 Vue。默认 `MOTIONARM_SOURCE=wearable`；网页选择实际腕端端口，连接成功后手动启用。A 固定 GPIO25、B 固定 GPIO26，按下状态分别显示。腕端固件以 USB 输出，不运行 ESP-NOW。录制历史可离线三维回放，不接入实体执行端。

下方 mock/robot 命令为保留的旧后端入口，新三维控制台不以它们作为实时输入。

## 机械臂串口演示

打开 [joint_demo_v2.ino](../firmware/joint_demo_v2/joint_demo_v2.ino)，板型ESP32 Dev Module、执行端最近COM7，115200、换行。托稳两臂、关闭外置5V再上传。恢复供电并准备好45°待机姿态后，逐条发送 `DEMO ARM`、`DEMO APPROACH`、`DEMO DRY`、`DEMO GRIP`。完整含义见 [当前演示](15_Wide_Slow_Demo_V2.md)。首次ARM脉宽立即应用，不保证从未知姿态慢速到位。

从仓库根目录只编译（不打开串口）：

```powershell
.\tools\build-local.ps1 -Action Build -Sketch joint_demo_v2
.\tools\build-local.ps1 -Action Build -Sketch wearable
.\tools\build-local.ps1 -Action Build -Sketch robot
```

脚本使用英文临时路径和仓库自带库，构建产物不写入源码目录。原版tools/esp32.ps1为历史环境入口，保留原文件，但当前机器优先使用build-local.ps1。

## 网页软件模拟

在根目录创建环境并启动后端：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r .\backend\requirements.txt
$env:MOTIONARM_SOURCE = 'mock'
.\.venv\Scripts\python.exe -m uvicorn app.main:app --app-dir backend --host 127.0.0.1 --port 8000
```

另一个终端进入frontend。若已有package-lock.json用npm ci，否则先npm install，再npm run dev；浏览器打开 http://127.0.0.1:5173 。此模式仅供旧后端接口使用；新的三维控制台使用上述 wearable 模式。

## 正式robot串口只读

必须运行firmware/robot，不能把文本命令演示当NDJSON设备。关闭Arduino串口监视器，后端环境设置如下，再运行相同uvicorn命令：

```powershell
$env:MOTIONARM_SOURCE = 'serial'
$env:MOTIONARM_SERIAL_PORT = 'COM7'
$env:MOTIONARM_SERIAL_BAUD = '115200'
$env:MOTIONARM_CALIBRATION = (Resolve-Path .\config\robot_calibration.local.json).Path
Remove-Item Env:MOTIONARM_ENABLE_MOTION -ErrorAction SilentlyContinue
```

当前正式profile为空且本地配置未验证，因此实体路径保持只读。新的 wearable 模式可独立读取实际腕端端口；不能在后端占用端口时同时运行 tools/read-esp32.ps1。历史无线集成参考 [交接](Wearable_Handoff.md)。
