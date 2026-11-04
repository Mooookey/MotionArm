# 一页操作与验收流程

## 启动

1. 在 `E:\Code\emb-exam` 打开 PowerShell，连接 ESP32，确认实际串口；关闭 Arduino 串口监视器和其他占用端口的程序。
2. 检查、编译与上传（把 COM3 替换成当前实际端口）：

   ```powershell
   .\tools\esp32.ps1 -Action Check
   .\tools\esp32.ps1 -Action Build -Sketch E:\Code\emb-exam\firmware\ImuExam
   .\tools\esp32.ps1 -Action Upload -Sketch E:\Code\emb-exam\firmware\ImuExam -Port COM3
   ```

3. 在第二个 PowerShell 启动本机 HTTP 服务：

   ```powershell
   .\tools\serve.ps1
   ```

4. 在 Edge/Chrome 打开 [本地网页](http://127.0.0.1:8000)，点击连接并选择 CH340 对应端口。运行通信速率为 460800，与上传速率 115200 不同。界面模拟模式只能检查显示，不能代替真实测试。

## 采集静态数据

1. 把模块水平固定、+Z 向上，静置直至校准完成。必要时在未记录状态点击重新校准。
   校准将保存三轴陀螺零偏和单姿态重力模长比例 κ；仅 processed 加速度应用 κ。它不是完整三轴/六面标定，同姿态归一结果不构成精度证明。
2. 点击开始采集/记录，连续保持静止至少 5 分钟。前 60 秒用于误差和标准差；整段用于漂移、丢包和采样稳定性。
3. 点击停止，等待设备 END 摘要。确认记录未显示失读、缺样或完整性失败；异常时保留数据并排查重测。
4. 下载 CSV 与 JSON，导出指标图。JSON 必须包含设备采样配置、校准值和结束计数。检查实际积分时长与完整性，不把目标频率当成实测频率。

也可关闭网页串口后，由命令行独占采集。略多于 300 秒可避免 START 响应和主机计时开销使设备时间窗口不足：

```powershell
.\tools\capture.ps1 -Port COM3 -Seconds 305 -OutDir E:\Code\emb-exam\results\static_305s -Recalibrate
node .\tools\decode-capture.mjs E:\Code\emb-exam\results\static_305s
node .\tools\build-report.mjs E:\Code\emb-exam\results\static_305s\recording.json --kind static
```

解码生成 `data.csv`、`recording.json`、`metrics.json`、`metadata.json`；网页可导入 `recording.json` 回放和查看指标。静态测试前最好用水平仪确认姿态；仅近似水平时，X/Y 均值含重力投影，不能把全部误差归因于传感器零偏。

报告生成器另生成 `report.html` 和 `figures` 目录中的六张独立 SVG，浏览器打开即可查看、打印或保存。静态录制的第六张仅为原始/处理曲线参考，不能替代动态动作响应验收；动态记录应使用 `--kind dynamic` 并补充实际动作说明。

## 动态动作与视频

1. 新建动态记录，做一次缓慢转动和一次较快转动，导出原始/处理对比曲线，说明延迟及峰值变化。
2. 在未记录状态位移归零，演示“静止→短距离移动→静止”，查看三轴相对位移。此值会漂移，不用于长期定位。
3. 视频同时拍到实体模块和屏幕：水平初始姿态、绕 +X 正转约 90°、回初始后绕 +Y、回初始后绕 +Z；再组合转动至少 30 秒。

## 提交前

按 [验收说明](ACCEPTANCE.md) 填入真实数值和计算过程，附六类指标图、原始/处理 CSV、配置/计数 JSON、轴向视频及完整源码。保留算法来源和许可说明，模拟演示结果不要混入实测材料。
