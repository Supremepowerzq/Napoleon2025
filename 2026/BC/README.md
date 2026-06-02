# BC — 支气管镜行为克隆自主介入模块

基于模仿学习（Behavior Cloning）的支气管镜机器人自主介入系统。  
专家手动操作数据 → GRU策略网络训练 → AI自主巡检各主要支气管。

---

## 目录

- [功能概述](#功能概述)
- [文件结构](#文件结构)
- [快速开始](#快速开始)
- [详细操作说明](#详细操作说明)
  - [Step 1 安装依赖](#step-1-安装依赖)
  - [Step 2 录制专家演示](#step-2-录制专家演示)
  - [Step 3 训练模型](#step-3-训练模型)
  - [Step 4 验证推理](#step-4-验证推理)
  - [Step 5 接入主程序](#step-5-接入主程序)
- [观测空间设计](#观测空间设计)
- [动作空间设计](#动作空间设计)
- [支气管路径标签](#支气管路径标签)
- [模型架构](#模型架构)
- [数据格式](#数据格式)
- [主程序集成](#主程序集成)
- [训练建议](#训练建议)
- [常见问题](#常见问题)

---

## 功能概述

| 能力 | 说明 |
|---|---|
| 自主导航 | 从入口出发，按指定路径介入各主要支气管分支 |
| 岔口识别 | 识别当前视野中的岔口，自动选择正确分支进入 |
| 结石/黏液清理 | 检测到目标时自动切换清理子策略，清理后恢复导航 |
| 多路径支持 | 一个模型覆盖左主支气管、右主支气管等9条路径 |
| 实时推理 | GRU推理延迟 < 2ms（4090），可在20~60Hz控制循环中运行 |

**工作流程**

```
专家手动介入（录制中）
        ↓
  ActionRecorder + 视觉特征同步保存 → expert_demos/*.h5
        ↓
  BronchusDataset（滑动窗口采样，T=8帧）
        ↓
  BronchusPolicy 训练（BC Loss = MSE导航 + CE子任务）
        ↓
  checkpoints/bc_best.pth
        ↓
  主程序 AI_AUTO 状态 → BCRunner.step() → 电机指令
```

---

## 文件结构

```
2026/BC/
├── README.md            ← 本文档
├── 1.py                 ← 命令行入口（训练/验证/查看演示）
├── model.py             ← 策略网络定义（BronchusPolicy，OBS_DIM=20）
├── data_collector.py    ← 专家演示录制器（含速度+dt+视觉特征）
├── dataset.py           ← PyTorch Dataset & DataLoader
├── train.py             ← BC 训练主循环
├── inference.py         ← 实时推理接口（接入主程序）
├── expert_demos/        ← 录制的演示数据（自动创建）
│   ├── 20260527_143022_LMB_1800frames.h5   ← 左主支气管
│   ├── 20260527_151033_RMB_1650frames.h5   ← 右主支气管
│   └── ...
└── checkpoints/         ← 训练权重（自动创建）
    ├── bc_best.pth      ← 最佳验证损失权重
    └── bc_epoch150.pth  ← 定期快照
```

---

## 快速开始

```powershell
# 进入 BC 目录
cd g:\zq\Napoleon2025\2026\BC

# 0. 检查依赖
python bc_main.py install_check

# 1. 在主程序 UI 中录制专家演示（见下方 Step 2）

# 2. 训练
python bc_main.py train --epochs 150

# 3. 验证推理
python bc_main.py test --path LMB
```

---

## 详细操作说明

### Step 1 安装依赖

```powershell
pip install h5py tensorboard
```

其余依赖（torch、numpy）在 Napoleon2025 主环境中已安装。

验证：
```powershell
python bc_main.py install_check
```

预期输出：
```
  ✓ PyTorch
  ✓ HDF5
  ✓ TensorBoard
  ✓ NumPy
  ✓ CUDA: GPU=NVIDIA GeForce RTX 4090
```

---

### Step 2 录制专家演示

**录制方式：直接在主程序 UI 操作**（推荐，无需另开终端）

录制流程：
1. 正常启动主程序 `main2026-Xhandwriting-AC-Auto(5.19depthtest)-Bronchus.py`
2. 在 UI 的「动作录制与回放」区域，**选择当前要录制的支气管路径**（下拉菜单，见下方主程序集成说明）
3. 点击「**开始录制**」→ 专家用手柄按正常临床流程介入
4. 遇到结石/黏液时正常操作清理，系统自动标记为清理帧
5. 完成后点击「**停止录制**」→ 自动保存到 `BC/expert_demos/`

文件命名规则（自动生成，全英文）：
```
{时间戳}_{YOLO代码}_{帧数}frames.h5
例如：20260527_143022_LMB_1800frames.h5
```

**需要在主程序中添加的路径选择 UI（见「主程序集成」章节）**

**路径标签（YOLO 代码）**

| YOLO 代码 | 中文名 | 建议录制次数 |
|---|---|---|
| EXP | 自主探索（不指定目标） | 10次 |
| TR  | 气管 | 5次 |
| LMB | 左主支气管 | 10次 |
| RMB | 右主支气管 | 10次 |
| LUB | 左上叶 | 5次 |
| LLB | 左下叶 | 5次 |
| RUL | 右上叶 | 5次 |
| BI  | 中间支气管 | 5次 |
| RML | 右中叶 | 5次 |
| RLL | 右下叶 | 5次 |

**录制质量要求**
- 每次录制时长建议 30~120 秒
- 操作要连贯，**避免中途长时间暂停**（dt 突变会影响速度特征质量）
- 如遇结石/黏液目标，正常操作清理后继续前进（自动标记为清理帧）
- 同一路径从**不同初始姿态**出发（手柄轻微偏转后再归零，覆盖扰动场景）

**查看已录制数据**
```powershell
python bc_main.py list
```

---

### Step 3 训练模型

```powershell
python bc_main.py train --epochs 150 --batch_size 64
```

**监控训练（另开终端）**
```powershell
tensorboard --logdir BC/runs/
# 浏览器打开 http://localhost:6006
```

**关键指标说明**

| 指标 | 含义 | 健康值 |
|---|---|---|
| `train/nav` | 导航动作 MSE 损失 | < 0.05 趋势下降 |
| `val/nav` | 验证集导航损失 | < 0.08，不回升 |
| `val/task_acc` | 子任务分类准确率 | > 0.85 |
| `lr` | 学习率 | OneCycleLR 自动调整 |

**断点续训**
```powershell
python bc_main.py train --resume BC/checkpoints/bc_epoch100.pth --epochs 200
```

**训练时间参考（RTX 4090）**

| 数据量 | Epochs | 预估时间 |
|---|---|---|
| 5000 帧 | 150 | ~30 分钟 |
| 20000 帧 | 150 | ~90 分钟 |
| 50000 帧 | 200 | ~3 小时 |

---

### Step 4 验证推理

```powershell
python bc_main.py test --path 2
```

输出示例（随机观测仿真）：
```
模拟推理 20 步  目标: left_main
 步     m0       m1       m2      任务          置信
─────────────────────────────────────────────────
   1  +0.000   +0.021   -0.015  navigate      0.923
   2  +0.043   +0.018   -0.012  navigate      0.911
   8  +0.000   +0.000   +0.000  clear_stone   0.876
```

---

### Step 5 接入主程序

在 `main2026-Xhandwriting-AC-Auto(5.19depthtest)-Bronchus.py` 中添加：

**① 文件顶部导入**
```python
from BC.inference import BCRunner
from BC.model import BRONCHUS_PATHS
```

**② 初始化（在 MotorGroup2025 之后）**
```python
bc_runner: Optional[BCRunner] = None
```

**③ 添加 AI_AUTO 状态处理函数**
```python
def _loop_ai_auto(self):
    global bc_runner
    if bc_runner is None:
        try:
            bc_runner = BCRunner(
                model_path="BC/checkpoints/bc_best.pth",
                goal_id=0,               # 0=自主探索
                action_scale_factor=0.3  # 先用小值，验证安全后调大到0.5~1.0
            )
            self.voice_window.push_log("AI自主巡检模式已就绪")
        except Exception as e:
            self.voice_window.push_log(f"BC模型加载失败: {e}")
            self._on_command("manual")
            return

    decision = bc_runner.step(self.motor_group)

    self.motor_group.set_motor_position(0, decision.new_m0, max_speed=25.0)
    self.motor_group.set_motor_position(1, decision.new_m1, max_speed=15.0)
    self.motor_group.set_motor_position(2, decision.new_m2, max_speed=15.0)

    self.voice_window.update_state(
        "AI自主巡检",
        f"[{decision.goal_id}]{BRONCHUS_PATHS.get(decision.goal_id,'?')} "
        f"| {decision.task_name} | step={decision.step_count}"
    )
```

**④ 视觉线程注入特征（在 predict_wqx-Bronchus.py 末尾添加）**
```python
# 导入（文件顶部）
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "BC"))
from data_collector import update_visual_features

# 每帧处理结束后调用（在 apply_motor_command 附近）
update_visual_features(
    stone_cx        = cx_norm,          # 结石质心 x，归一化到 [-1,1]
    stone_cy        = cy_norm,          # 结石质心 y
    stone_area      = area_ratio,       # 像素面积比 [0,1]
    stone_detected  = bool(has_stone),
    bifur_cx        = bfx,             # 岔口质心 x
    bifur_cy        = bfy,
    bifur_area      = bf_area,
    depth_center_mm = z_center,
    depth_mean_mm   = z_mean,
    depth_max_mm    = z_max,
    path_left_mm    = path_l,          # DepthPathFinder 左路径深度
    path_center_mm  = path_c,
    path_right_mm   = path_r,
)
```

**⑤ 语音命令（可选）**
```python
STANDARD_VOICE_COMMANDS["自主巡检"] = "ai_auto"
STANDARD_VOICE_COMMANDS["开始自动"] = "ai_auto"
```

---

## 观测空间设计

每帧观测向量共 **20 维**，按如下顺序排列：

| 索引 | 字段名 | 归一化 | 来源 | 作用 |
|---|---|---|---|---|
| 0 | stone_cx | ÷1.0 | HSV 分割质心 x（已归一化） | 结石位置 |
| 1 | stone_cy | ÷1.0 | HSV 分割质心 y | 结石位置 |
| 2 | stone_area | ÷1.0 | 结石像素面积比 [0,1] | 结石大小/距离 |
| 3 | stone_detected | ÷1.0 | 0/1 布尔值 | 触发清理子策略 |
| 4 | bifur_cx | ÷1.0 | 岔口质心 x | 岔口方向 |
| 5 | bifur_cy | ÷1.0 | 岔口质心 y | 岔口方向 |
| 6 | bifur_area | ÷1.0 | 岔口面积比 | 岔口大小/距离 |
| 7 | depth_center_mm | ÷300 | 图像中心深度 | 前方障碍物感知 |
| 8 | depth_mean_mm | ÷300 | 管腔平均深度 | 腔道整体情况 |
| 9 | depth_max_mm | ÷300 | 最深点深度 | 可通行距离 |
| 10 | path_left_mm | ÷300 | 左侧路径深度 | 左路可通行性 |
| 11 | path_center_mm | ÷300 | 中间路径深度 | 中路可通行性 |
| 12 | path_right_mm | ÷300 | 右侧路径深度 | 右路可通行性 |
| 13 | m0_angle | ÷900 | M0 前进电机角度 | 当前位置 |
| 14 | m1_angle | ÷170 | M1 左右电机角度 | 当前位置 |
| 15 | m2_angle | ÷500 | M2 上下电机角度 | 当前位置 |
| 16 | m0_vel | ÷90 | M0 速度 °/s | **当前运动状态，防止抖动** |
| 17 | m1_vel | ÷30 | M1 速度 °/s | **当前运动状态** |
| 18 | m2_vel | ÷50 | M2 速度 °/s | **当前运动状态** |
| 19 | dt | ÷0.05 | 帧间隔 s（20Hz=1.0） | **处理帧率波动/操作停顿** |

> **为什么要加速度和 dt？**
> - 只有位置时，GRU 无法区分"电机正在高速转动"和"静止"，容易在转向到位后继续输出转向指令导致过冲。速度信号让模型知道当前动量，学会在适当时机减速。
> - dt 处理帧间隔不均匀的情况：手柄操作停顿、录制暂停恢复、推理时帧率波动，都会让动作幅度失真，dt 归一化修正这个误差。

---

## 动作空间设计

输出 **3 维**归一化增量，通过 `ACTION_SCALE` 反归一化为实际角度变化：

| 维度 | 含义 | 缩放因子 | 单位 |
|---|---|---|---|
| 0 | Δm0 | ×5.0 | °/帧（前进/后退） |
| 1 | Δm1 | ×3.0 | °/帧（左右转向） |
| 2 | Δm2 | ×3.0 | °/帧（上下转向） |

推理时还叠加：
- **动作平滑**：EMA 滤波（α=0.4）
- **单步限幅**：Δm0 ≤ 8°，Δm1/Δm2 ≤ 5°
- **全局缩放**：`action_scale_factor`（默认 0.3，调稳后增大）

---

## 支气管路径标签

与 YOLO 部位识别状态机的代码保持一致，内部用整数索引供 `nn.Embedding` 使用，
对外（文件名、UI显示、命令行参数）使用 YOLO 字母代码：

| 整数索引 | YOLO 代码 | 中文名 | 备注 |
|---|---|---|---|
| 0 | EXP | 自主探索 | 不指定目标，随 DepthPathFinder 探索 |
| 1 | TR  | 气管 | trachea |
| 2 | LMB | 左主支气管 | left main bronchus |
| 3 | RMB | 右主支气管 | right main bronchus |
| 4 | LUB | 左上叶 | left upper lobe bronchus |
| 5 | LLB | 左下叶 | left lower lobe bronchus |
| 6 | RUL | 右上叶 | right upper lobe |
| 7 | BI  | 中间支气管 | bronchus intermedius |
| 8 | RML | 右中叶 | right middle lobe |
| 9 | RLL | 右下叶 | right lower lobe |

```python
# model.py 中的双向映射
YOLO_CODE_TO_IDX = {"EXP":0, "TR":1, "LMB":2, "RMB":3, "LUB":4,
                    "LLB":5, "RUL":6, "BI":7, "RML":8, "RLL":9}

# 使用方式（YOLO代码 → Embedding整数索引）
from BC.model import path_idx
idx = path_idx("LMB")   # → 2
idx = path_idx(2)        # → 2（直接传整数也支持）
```

---

## 模型架构

```
输入: obs_seq [B, T=8, OBS_DIM=16] + goal_id [B]
                    │
        ┌───────────┴──────────────┐
        │       ObsEncoder          │
        │   Linear(16→128)          │
        │   LayerNorm + ELU         │
        │   Linear(128→128)         │
        │   LayerNorm + ELU         │
        └───────────┬──────────────┘
                    │ [B, T, 128]
        ┌───────────┴──────────────┐
        │  GoalEmbedding(9→16)     │
        │  expand → [B, T, 16]     │
        └───────────┬──────────────┘
                    │
              concat [B, T, 144]
                    │
        ┌───────────┴──────────────┐
        │      GRU(144→256, 2层)    │
        │      dropout=0.1          │
        └───────────┬──────────────┘
                    │ last hidden [B, 256]
            ┌───────┴────────┐
            │                │
     ActionHead          TaskHead
   Linear(256→128)    Linear(256→64)
   ELU                ELU
   Linear(128→3)      Linear(64→2)
   Tanh               (logits)
            │                │
      action [-1,1]³    task_probs [2]
```

**总参数量：约 0.5M**（适合实时推理，4090 推理 < 1ms/帧）

---

## 数据格式

每条演示保存为 HDF5 文件（`expert_demos/*.h5`）：

```
episode.h5
├── attrs
│   ├── session_name : str
│   ├── path_label   : int       目标支气管标签
│   ├── path_name    : str
│   ├── n_frames     : int
│   ├── duration_s   : float
│   └── obs_fields   : str       逗号分隔的字段名
├── timestamps    [N]    float32  相对时间(s)
├── motor_angles  [N, 3] float32  原始角度(°)
├── actions       [N, 3] float32  角度增量(°)，加载时自动归一化
├── task_labels   [N]    int8     0=导航 1=清理
└── obs           [N,16] float32  归一化观测向量
```

---

## 主程序集成

`inference.py` 末尾的 `AI_AUTO_INTEGRATION_SNIPPET` 字符串包含完整集成代码片段，运行以下命令直接打印：

```powershell
python -c "from BC.inference import AI_AUTO_INTEGRATION_SNIPPET; print(AI_AUTO_INTEGRATION_SNIPPET)"
```

---

## 训练建议

**数据量目标**

| 阶段 | 目标帧数 | 预期效果 |
|---|---|---|
| 最低可用 | 5,000 帧（~4分钟） | 基本方向正确，精度低 |
| 推荐基线 | 20,000 帧（~17分钟） | 可用于离体实验 |
| 动物实验前 | 50,000 帧（~40分钟） | 稳定介入主要气道 |

**数据多样性**
- 每条路径至少从 **3种不同初始姿态** 出发
- 包含结石清理场景（约占总帧数 20%）
- 包含错误恢复（操作失误后人工纠正）

**超参数调整建议**

| 场景 | 建议调整 |
|---|---|
| 验证损失不下降 | 降低 lr 到 1e-4，增大 patience |
| 过拟合（val >> train） | 增加 augment=True，减小 seq_len |
| 子任务准确率低 | 增大 task_weight 到 0.5 |
| 推理时动作抖动 | 降低 action_scale_factor，增大 EMA alpha |

**DAgger 迭代（进阶）**

BC 训练后，将模型部署到真机进行 DAgger 迭代：
1. 策略自动运行，专家用手柄随时纠正
2. 纠正动作自动被 `data_collector` 记录
3. 合并新旧数据重新训练
4. 重复 2~3 轮后策略泛化性大幅提升

```powershell
# DAgger 数据合并重训（新数据放在 dagger_demos/ 目录）
python bc_main.py train --resume checkpoints/bc_best.pth --epochs 50
```

---

## 常见问题

**Q: `h5py` 安装失败？**  
`pip install h5py --only-binary :all:` 或用 conda：`conda install h5py`

**Q: 训练时提示 "训练集为空"？**  
检查 `expert_demos/` 目录下是否有 `.h5` 文件，运行 `python bc_main.py list` 确认。

**Q: 推理时电机几乎不动？**  
`action_scale_factor` 太小，逐步调大：`runner.set_scale(0.5)` → `0.8` → `1.0`

**Q: 子任务一直输出 clear_stone？**  
视觉特征 `stone_detected` 一直为 True，检查 `predict_wqx-Bronchus.py` 中的 HSV 阈值是否过于宽松。

**Q: 验证损失震荡不收敛？**  
演示数据中同一路径的操作风格不一致（不同人录制或每次操作差异很大），建议统一操作规范后重新录制。

**Q: 想切换巡检目标（例如从左主改右主）？**  
```python
bc_runner.set_goal(3)   # 切换到右主支气管，自动 reset GRU 隐状态
```

---

*Napoleon2025 BC Module — ZQ 2026-05*
