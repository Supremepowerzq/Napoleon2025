# 电机联动支气管检测

本目录是 Napoleon 运行工程内的支气管检测部署模块。它在现有主程序中复用同一个串口、同一组电机对象和同一个摄像头，不创建第二个硬件连接。

## 运行架构

1. `main2026-Xhandwriting-AC-AutoNav(0710)-Bronchus.py` 仍是唯一的电机和串口拥有者；PyQt 界面只通过其既有命令队列调用控制功能。
2. `NapoleonMotorFeedbackAdapter` 只读取主程序已有的 M0/M2 缓存值和手柄输入，发布只读快照，不下发任何电机命令。弯曲状态优先使用 M2 实际保持角度，而不是只看会自动回中的手柄位置。
3. `predict_2026_528_zck_motor_yolo.py` 提供已部署的 PyTorch YOLO、ROI 和去畸变能力；`recognition_gui/video_worker.py` 只替换显示循环，将无检测框、无置信度的干净画面嵌入 PyQt。
4. `recognition_gui/` 是部位识别专用界面：左侧点亮支气管位置，右侧显示内窥镜，主界面提供检测、录像、手动、停止、归零、设零与断电；巡检和 AutoNav 放在菜单栏。
5. `SafeAnatomyStateMachine` 用 M0 的 `forward/backward/idle` 替代原脚本中的人工方向变量。

## 状态切换规则

- 普通部位只使用 M0 前进、后退和停止关系，不读取上下操作。
- 只有 `RMB -> RULB/BI` 使用 `RmbBranchGate`：
- `RULB`：常规前进过程中按 6 个检测周期累计至少 3 次，并且需要有弯曲证据；上下方向取绝对值，因此向上和向下均有效。弯曲证据优先取设零后 M2 的实际保持角度（绝对值至少 5°）；手柄发生过 M2 弯曲操作后会锁存弯曲状态，手柄回中不会清除，只有 M2 回到零位附近并稳定 5 个检测周期才释放。M2 达到强弯曲区（绝对值至少 100°）时，一次 RULB 视觉命中即可在前进过程中直接确认。若前进期间已经出现 RULB 黄色候选，停车后进入 3 秒补充确认窗口；若未经历停止检测周期就立即回退，强弯曲条件下也会先提交 RULB，再按 RULB→RMB→TR 分级回退。M1 整体旋转不参与门控，只会通过改变镜头方向影响模型画面。停车窗口不会累计 BI，也不能在静止状态凭空建立 RULB 候选。
  - `BI`：8 个检测周期中至少命中 6 次、连续 2 个周期不再看到 RMB，并且 M0 从进入 RMB 起继续前进至少 45 度后才确认。
  - RMB 与 BI 同时容易出现时，BI 先保持为软候选；RULB 的稀疏证据优先完成分叉判定。
- M0 后退时，所有部位只允许回到拓扑父节点。
- 手动模式下只有 M0 出现真实速度反馈才发布 `forward/backward`；扳机命令刚下发但电机尚未运动时保持 `idle`，防止提前回退到父节点。
- 电机反馈无效时强制按 `idle` 处理，不允许状态前跳。
- 电机快照超过 0.5 秒未更新时视为失联，同样强制降级为 `idle`。

所有阈值集中在 `config.py` 的 `StateMachineConfig`，现场调整不需要改状态机代码。

## 文件说明

- `config.py`：标签别名、支气管拓扑和现场阈值。
- `napoleon_adapter.py`：Napoleon 缓存电机/手柄信号适配器，不访问串口。
- `xinput_controller.py`：轻量版专用 Xbox 轮询适配器，绕过按钮可用但轴值不更新的问题；不修改原手柄接口。
- `motor_bus.py`：主控制线程到视频线程的只读电机快照。
- `safe_state_machine.py`：普通前后拓扑状态机。
- `rmb_branch_gate.py`：仅用于 RMB、RULB、BI 的特殊门控。
- `coordinator.py`：视频检测与电机快照的组合入口。
- `detection_bus.py`：向 UI 或日志提供最新确认状态。
- `run_napoleon.py`：资源预检和唯一主程序启动入口，不复制硬件控制代码。
- `test_logic.py`：不连接相机、电机即可执行的回归测试。
- `../recognition_gui/`：PyQt 主界面、视频帧总线、录像、支气管点亮和机器人运行适配器。
- `../run_recognition_gui.py`：PyQt 专用直接启动脚本。

同级的 `2026/predict_2026_528_zck_motor_yolo.py` 只由独立识别入口加载。其他原主程序和原 `predict_2026_528_zck.py` 的完整视觉调用关系保持不变。

PyQt 内窥镜画面不绘制检测框、类别或置信度。全部原始检测仍送入门控状态机，最终 `Confirmed` 与候选位置只在左侧支气管模型上显示。`F/B/S`、手柄、摄像头和机器人模式在顶部状态栏显示。

## 模型资源

默认从以下目录读取，避免运行时依赖 `G:\\zck` 或 `G:\\wqx`：

```text
G:\zq\Napoleon2025\model_data\bronchus_yolo\best.pt
G:\zq\Napoleon2025\model_data\bronchus_yolo\undistort_maps_zhiqiguan_roi.npz
```

## 使用与验证

在 `G:\\zq\\Napoleon2025\\2026` 下执行纯逻辑测试：

```powershell
conda activate Napoleon_2026
python -m unittest Bronchialtree_identification.motor_linked_detection.test_logic -v
```

先执行不接硬件的部署预检：

```powershell
conda activate Napoleon_2026
python -m Bronchialtree_identification.motor_linked_detection.run_napoleon --check-only
```

确认后由同一个入口启动独立轻量版本：

```powershell
python -m Bronchialtree_identification.motor_linked_detection.run_napoleon
```

也可以直接启动 PyQt 入口：

```powershell
python -m Bronchialtree_identification.recognition_gui.launcher
```

无硬件离屏界面检查：

```powershell
python -m Bronchialtree_identification.recognition_gui.launcher --ui-smoke-test
```

首次上机建议抬起机器人或断开传动负载，仅检查：手柄前推显示 `forward`、后拉显示 `backward`、松开显示 `idle`，方向确认后再进行低速体模测试。纯逻辑测试不能代替串口、电机限位和急停的现场检查。
