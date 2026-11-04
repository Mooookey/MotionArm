# ESP32 六轴 IMU 实验

200Hz MPU6050 采集、ESP32 校准/低通/Mahony 姿态/短时位移、USB 二进制传输与离线 Web 上位机。

在 PowerShell 中使用英文工作区：

```powershell
Set-Location E:\Code\emb-exam
.\tools\esp32.ps1 -Action Check
.\tools\esp32.ps1 -Action Upload -Sketch E:\Code\emb-exam\firmware\ImuExam -Port COM3
.\tools\serve.ps1
```

每次重新确认端口。浏览器打开 http://localhost:8000，使用 Edge/Chrome，点击“连接设备”；上传速率为115200，数据通信为460800。连接后保持静置约3秒完成校准，再开始采集/记录。

- 详细操作：`docs/QUICKSTART.md`
- 六项任务、公式、验收证据和边界：`docs/ACCEPTANCE.md`
- 二进制协议：`PROTOCOL.md`
- 固件与数学说明：`firmware/ImuExam/`
- 无依赖网页：`web/`；`?demo=1` 为明确标记的模拟模式，不可用于验收。
- 实测数据与报告：`results/`。网页可导入 `recording.json` 回放，CSV保留ESP32采样时间戳。

自动采集（关闭网页串口和IDE监视器，模块保持静止）：

```powershell
.\tools\capture.ps1 -Port COM3 -Seconds 305 -OutDir E:\Code\emb-exam\results\static -Recalibrate
node tools/decode-capture.mjs E:\Code\emb-exam\results\static
node tools/build-report.mjs E:\Code\emb-exam\results\static\recording.json
.\tools\check.ps1
```

六轴 IMU 的 yaw 是相对方向，位移为有静止约束的短时积分估计。轴对齐与动态响应验收需要实际转动模块；视频需同时展示实体模块和网页。
