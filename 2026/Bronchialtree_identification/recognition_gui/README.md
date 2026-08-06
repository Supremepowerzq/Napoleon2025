# 支气管部位识别 PyQt 控制台

本界面是 `Bronchialtree_identification` 内的独立运行入口。它使用 PyQt5 把支气管位置图和内窥镜画面放在同一窗口中，不修改 Napoleon 的电机、限位、手柄和 AutoNav 控制实现。

## 主界面

- 左侧：支气管模型、确认位置、已通过路径和候选位置。
- 右侧：无检测框、无类别文字、无置信度的干净内窥镜画面。
- 顶部：当前位置、F/B/S、手柄/机器人模式和内窥镜连接状态。
- 底部：目标检测、录像、手动、停止、归零、设为零位和断电。
- 菜单栏：自主巡检、单条现场采集路径 AutoNav、导航暂停和停止。

左侧定位点已按 `assets/bronchi.png` 的气道中心线重新校准；拓扑连线会经过图中实际隆突和叶支气管分叉，不再用跨过组织区域的直线连接。

目标检测关闭时摄像头和录像保持运行，但不执行 YOLO 推理；左侧全部部位熄灭并显示“目标检测未开启”。开启后先显示“等待识别”，同一确认部位需要连续 4 次推理达到至少 0.60 置信度后才开始点亮；低置信度、漏检或部位变化都会重新计数。内部状态机仍保留 TR 作为拓扑起点，但默认 TR 和单帧 TR 假阳性不会直接显示。

录像默认保存到：

```text
Bronchialtree_identification/recordings/
```

## 启动

在 `G:\zq\Napoleon2025\2026` 下：

```powershell
C:\Users\lenovo\.conda\envs\Napoleon_2026\python.exe -m Bronchialtree_identification.recognition_gui.launcher
```

也可继续使用部署入口：

```powershell
C:\Users\lenovo\.conda\envs\Napoleon_2026\python.exe -m Bronchialtree_identification.motor_linked_detection.run_napoleon
```

## 无硬件检查

```powershell
C:\Users\lenovo\.conda\envs\Napoleon_2026\python.exe -m Bronchialtree_identification.recognition_gui.launcher --check-only
C:\Users\lenovo\.conda\envs\Napoleon_2026\python.exe -m Bronchialtree_identification.recognition_gui.launcher --ui-smoke-test
C:\Users\lenovo\.conda\envs\Napoleon_2026\python.exe -m unittest Bronchialtree_identification.motor_linked_detection.test_logic Bronchialtree_identification.recognition_gui.test_gui_logic -v
```

## 安全边界

- `video_worker.py` 只拥有摄像头、YOLO 和录像，不访问串口或下发电机命令。
- `robot_runtime.py` 加载现有 AutoNav 主程序，通过既有 UI 命令队列执行按钮命令。
- 电机反馈适配器只读取已有缓存并发布状态，不创建第二套串口。
- 巡检默认使用 `AutoNavdatasets` 中最近一次 RMB 单文件路径，不做路径平均。
- TR 菜单项调用原主程序的独立三轴返回原点逻辑，不当作 AutoNav 数据文件。
- Layer 2/3 在新界面的路径导航中默认关闭。
