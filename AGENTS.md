## 工作区与资料

- 使用英文工作区，保持工程和构建路径全英文，已遇到中文路径导致链接失败。

## 技术栈

- 后端 Python FastAPI
- 前端 Vue.js

## 硬件与接线

- ESP32-D0WDQ6 rev1.1，双核240MHz，Flash 4MB；Arduino板型 `ESP32 Dev Module`，FQBN `esp32:esp32:esp32`
- USB串口为CH340
- IMU名称：MPU6050姿态传感器_No34；I²C地址 `0x68`，WHO_AM_I=`0x68`
  - 接线：VCC→3V3，GND→GND，SDA→GPIO21，SCL→GPIO22；诊断总线100kHz

- PCA9685
- 996数字舵机
- 4自由度机械臂
- PAC9685：CH12连接夹爪舵机，CH13连接小臂，CH14连接底座，CH15连接大臂
- ESP32：D21连接 PAC9685 SDA，D22连接SCL
- 按钮：正极、负极、OUT等3级，低电平触发
- 第二块ESP32：GPIO25，GPIO26分别连接按钮OUT极
- 使用面包盘负极渠连接了ESP32的GND和两个按钮端的GND，正极渠连接了ESP32的3V3和按钮的正极
- MPU6050的SDA连ESP32 GPIO21、SCL连GPIO22

## 机械约束

- 夹爪不得长时间夹持或持续夹紧，需避免堵转发热
- 机械臂在无驱动保持时会因重力下垂
- 承重关节在短时输出结束后释放可能下落

## 软件与离线依赖

- Arduino IDE 2.3.10：`D:\ESP32\app\Arduino IDE`。
- CLI 1.5.1：`D:\ESP32\app\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe`。
- ESP32核心3.3.12及工具：`%LOCALAPPDATA%\Arduino15\packages`。
- 离线库：`%LOCALAPPDATA%\Arduino15\libraries`；标准用户库目录 `C:\Users\21211\Documents\Arduino\libraries` 也已补齐五个Adafruit库：MPU6050、SSD1306、GFX、BusIO、Unified Sensor。

## 自动化入口

在工作区执行：

```powershell
.\tools\esp32.ps1 -Action Check
.\tools\esp32.ps1 -Action Build -Sketch E:\Code\emb-exam\YourSketch
.\tools\esp32.ps1 -Action Upload -Sketch E:\Code\emb-exam\YourSketch -Port COM3
```

## 协作建议

1. 使用中等粒度的子Agent任务派发，节省上下文；硬件串口操作由一个Agent独占，避免并发上传或读串口
2. 优先复用已安装库及有依据的开源组件，核对API、单位和硬件接线后再整合
3. 不建议在工作区保留临时文件，包括但不限于运行时日志、脚本等
