# Napoleon2025 — AI-Auto 自主介入模式开发指南

> 适用版本：`main2026-Xhandwriting-AC-Auto(5.19depthtest)-Bronchus.py`  
> 目标：新增 **Mode-4 AI-Auto**，基于多模态大模型+深度强化学习实现支气管镜自主介入导航

---

## 目录

1. [技术路线评估](#1-技术路线评估)
2. [系统架构设计](#2-系统架构设计)
3. [数据采集方案（第一步）](#3-数据采集方案第一步)
4. [仿真环境搭建](#4-仿真环境搭建)
5. [模型训练流程](#5-模型训练流程)
6. [AI-Auto 模式集成](#6-ai-auto-模式集成)
7. [一周上手计划](#7-一周上手计划)
8. [环境安装指令](#8-环境安装指令)
9. [目录结构规划](#9-目录结构规划)
10. [常见问题](#10-常见问题)

---

## 1 技术路线评估

### 你的想法可行性分析

你的核心思路：**录制专家操作数据 → 大模型训练 → 推理时根据图像+时间信息输出电机指令**

这是一条被学术界和工业界验证过的技术路线，具体对应：

| 你的描述 | 学术名称 | 可行性 |
|---|---|---|
| 录制专家的手动操作 | Expert Demonstration / Teleoperation Data | 已有 ActionRecorder 支撑，直接可用 |
| 利用大模型训练 | Behavior Cloning (BC) + Imitation Learning | 可行，是 DRL 的暖启动 |
| 根据图像+时间信息输出电机状态 | Multimodal Temporal Policy | 需要扩展观测空间（见第3节） |
| AI-Auto 控制模式 | Closed-loop Autonomous Navigation Policy | 在仿真验证后可接入真机 |

### 关键问题：当前 ActionRecorder 的缺陷

现有 `ActionRecorder.py` **只录制电机角度+时间戳**，缺少视觉特征，这是必须补齐的：

```python
# 现在的数据格式（不够用）
{
    "timestamp": 1.23,
    "motor_0": -45.2,
    "motor_1": 12.8,
    "motor_2": -230.0
}

# 需要扩展为（才能训练视觉策略）
{
    "timestamp": 1.23,
    "motor_0": -45.2, "motor_1": 12.8, "motor_2": -230.0,
    "visual": {
        "bifurcation_cx": 0.48,    # 岔口质心 x（归一化）
        "bifurcation_cy": 0.52,    # 岔口质心 y（归一化）
        "depth_mean_mm": 45.3,     # 深度均值
        "depth_max_mm": 89.1,      # 最深深度
        "depth_std_mm": 12.4,      # 深度标准差
        "path_direction": 0,       # -1=左 0=直 1=右
        "segmentation_area": 0.31, # 有效气道面积比
        "has_bifurcation": True    # 是否检测到岔口
    }
}
```

### 推荐技术路线

```
专家操作录制（含视觉特征）
        ↓
  Behavior Cloning 预训练
  (让策略学会模仿人的动作)
        ↓
  MuJoCo/PyBullet 仿真环境
  (基于支气管STL模型构建)
        ↓
  PPO 强化学习精调
  (在仿真中超越专家水平)
        ↓
  AI-Auto 模式集成到真机
  (sim2real迁移)
```

---

## 2 系统架构设计

### 2.1 观测空间（Policy Input）

策略网络的输入分三部分：

```
观测 o_t = [视觉特征 z_t | 电机状态 s_t | 历史序列]

视觉特征 z_t (由视觉编码器提取):
  - 来源：predict_wqx-Bronchus.py 的 UNet 分割结果 + Depth-Anything-V2 深度图
  - 维度：256-dim 特征向量 (由冻结的 DINOv2-Small 编码器提取)

电机状态 s_t:
  - [m0_angle, m1_angle, m2_angle]  (当前角度，归一化到[-1,1])
  - [m0_speed, m1_speed, m2_speed]  (估算速度)

历史序列:
  - 最近 T=8 帧的 (z_t, s_t) 拼接，送入 GRU 编码时序信息
```

### 2.2 动作空间（Policy Output）

策略网络的输出：目标电机增量（每帧的角度变化量）

```
动作 a_t = [Δm0, Δm1, Δm2]

Δm0: [-5, +5] 度/帧  前进/后退
Δm1: [-3, +3] 度/帧  左右偏转
Δm2: [-3, +3] 度/帧  上下偏转

最终电机指令 = clamp(当前角度 + Δ, 电机限位)
```

### 2.3 策略网络架构

```
          ┌─────────────────────────────────────┐
          │           PolicyNet                  │
          │                                      │
图像帧 ──→│ DINOv2-Small Encoder (冻结)          │
          │        ↓ 256-dim                     │
电机状态 →│ State Embedding (Linear 6→64)        │
          │        ↓                             │
          │ [视觉+状态] → GRU(256, 2层)          │
          │ (处理时序信息，T=8帧窗口)             │
          │        ↓ 256-dim                     │
          │ Action Head (MLP 256→128→3)           │
          │        ↓ tanh激活                    │
          │   [Δm0, Δm1, Δm2]                   │
          └─────────────────────────────────────┘
```

### 2.4 训练阶段

| 阶段 | 方法 | 数据来源 | 目标 |
|---|---|---|---|
| Phase 1 | Behavior Cloning | 专家演示录制文件 | 快速获得可用基线 |
| Phase 2 | PPO 强化学习 | MuJoCo仿真环境 | 超越专家，适应新场景 |
| Phase 3 | Sim2Real 迁移 | 离体猪肺 → 动物实验 | 真机部署 |

---

## 3 数据采集方案（第一步）

### 3.1 扩展 ActionRecorder

需要在 `ActionRecorder.py` 的 `record_action()` 方法中添加视觉特征钩子：

**新增文件**：`2026/ai_auto/expert_recorder.py`

```python
"""
专家演示数据录制器（扩展版）
在原 ActionRecorder 基础上添加视觉特征保存
"""

import json, time, threading, os
from dataclasses import dataclass, asdict
from typing import Optional, Dict, List, Callable
import numpy as np


@dataclass
class ExpertFrame:
    timestamp: float
    motor_0: float   # 度
    motor_1: float
    motor_2: float
    # 视觉特征（由外部回调注入）
    bifurcation_cx: float = 0.5   # 归一化坐标
    bifurcation_cy: float = 0.5
    depth_mean_mm: float = 0.0
    depth_max_mm: float = 0.0
    depth_std_mm: float = 0.0
    path_direction: int = 0       # -1/0/1
    segmentation_area: float = 0.0
    has_bifurcation: bool = False


class ExpertDemoRecorder:
    """
    专家演示录制器
    使用方式：
        recorder = ExpertDemoRecorder(motor_group, visual_feature_fn)
        recorder.start()
        ... 专家手动操作 ...
        path = recorder.stop()  # 返回保存路径
    """

    def __init__(
        self,
        motor_group,
        visual_feature_fn: Optional[Callable[[], Dict]] = None,
        save_dir: str = "expert_demos",
        record_hz: float = 20.0,
    ):
        self.motor_group = motor_group
        self.visual_feature_fn = visual_feature_fn  # 视觉模块的回调
        self.save_dir = save_dir
        self.record_hz = record_hz
        self.frames: List[ExpertFrame] = []
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._start_time: float = 0.0
        os.makedirs(save_dir, exist_ok=True)

    def start(self) -> bool:
        if self._running:
            return False
        self.frames = []
        self._start_time = time.time()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"[ExpertRecorder] 开始录制，目标频率 {self.record_hz}Hz")
        return True

    def stop(self) -> Optional[str]:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        return self._save()

    def _loop(self):
        interval = 1.0 / self.record_hz
        while self._running:
            t0 = time.perf_counter()
            self._record_frame()
            elapsed = time.perf_counter() - t0
            sleep_t = interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def _record_frame(self):
        try:
            angles = self.motor_group.get_cached_angles()
            ts = time.time() - self._start_time

            frame = ExpertFrame(
                timestamp=ts,
                motor_0=float(angles[0]),
                motor_1=float(angles[1]),
                motor_2=float(angles[2]),
            )

            if self.visual_feature_fn:
                feat = self.visual_feature_fn()
                frame.bifurcation_cx = float(feat.get("cx", 0.5))
                frame.bifurcation_cy = float(feat.get("cy", 0.5))
                frame.depth_mean_mm = float(feat.get("depth_mean", 0.0))
                frame.depth_max_mm = float(feat.get("depth_max", 0.0))
                frame.depth_std_mm = float(feat.get("depth_std", 0.0))
                frame.path_direction = int(feat.get("direction", 0))
                frame.segmentation_area = float(feat.get("seg_area", 0.0))
                frame.has_bifurcation = bool(feat.get("has_bifurcation", False))

            self.frames.append(frame)
        except Exception as e:
            print(f"[ExpertRecorder] 记录帧失败: {e}")

    def _save(self) -> Optional[str]:
        if not self.frames:
            return None
        from datetime import datetime
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.save_dir, f"expert_{ts}.json")
        data = {
            "version": "1.0",
            "created_at": ts,
            "frame_count": len(self.frames),
            "duration": self.frames[-1].timestamp if self.frames else 0.0,
            "record_hz": self.record_hz,
            "frames": [asdict(f) for f in self.frames],
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        print(f"[ExpertRecorder] 已保存 {len(self.frames)} 帧 → {path}")
        return path
```

### 3.2 录制操作步骤

1. 将内镜插入支气管模型（或离体猪肺）
2. 确认摄像头和电机均已连接
3. 在主程序 UI 中切换到**手动模式**
4. 点击「开始录制」→ **专家按照临床规范缓慢操作**（每次操作3-5分钟）
5. 点击「停止录制」→ 数据自动保存到 `expert_demos/`

**建议录制场景**：
- 直段前进（20条）
- 左岔口转向（20条）
- 右岔口转向（20条）
- 遭遇黏液+清理（10条）
- 退出回撤（10条）

**目标数据量**：至少 80 条演示，总时长 > 4 小时

---

## 4 仿真环境搭建

### 4.1 选择：MuJoCo vs PyBullet

| 特性 | MuJoCo | PyBullet |
|---|---|---|
| 渲染质量 | 高，支持 offscreen | 中等 |
| 物理精度 | 高（适合柔性体） | 中等 |
| STL 导入 | 需转换为 MJCF/OBJ | 直接支持 URDF/OBJ |
| DRL 生态 | 与 stable-baselines3 无缝 | 同上 |
| 安装难度 | 需要许可证（MuJoCo 2.1+ 免费） | pip 直接安装 |
| **推荐** | 首选 | 备选 |

**建议优先用 PyBullet 快速启动**（安装简单），等仿真跑通后再迁移到 MuJoCo。

### 4.2 PyBullet 支气管仿真环境

**新增文件**：`2026/ai_auto/bronchus_env.py`

```python
"""
支气管仿真环境 - Gymnasium 接口
基于 PyBullet + STL 支气管模型
"""

import gymnasium as gym
import numpy as np
import pybullet as p
import pybullet_data
from typing import Tuple, Dict, Any, Optional


class BronchusEnv(gym.Env):
    """
    支气管镜仿真环境
    
    观测空间：视觉特征(8) + 电机状态(6) + 历史GRU隐状态(256) → 由策略网络处理
    动作空间：[Δm0, Δm1, Δm2] 归一化到 [-1, 1]
    """

    metadata = {"render_modes": ["rgb_array", "human"]}

    # 动作缩放（映射到度）
    ACTION_SCALE = np.array([5.0, 3.0, 3.0])

    # 电机限位（度）
    MOTOR_LIMITS = [(-900.0, 0.0), (-170.0, 170.0), (-500.0, 500.0)]

    def __init__(
        self,
        stl_path: str,
        render_mode: str = "rgb_array",
        max_steps: int = 500,
        target_depth_mm: float = 200.0,
    ):
        super().__init__()
        self.stl_path = stl_path
        self.render_mode = render_mode
        self.max_steps = max_steps
        self.target_depth_mm = target_depth_mm

        # 观测空间：[视觉特征8维 + 电机6维]
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(14,), dtype=np.float32
        )

        # 动作空间：归一化增量
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(3,), dtype=np.float32
        )

        self._physics_client = None
        self._bronchus_id = None
        self._scope_id = None
        self._motor_angles = np.zeros(3)
        self._step_count = 0
        self._total_depth = 0.0

    def reset(self, seed=None, options=None) -> Tuple[np.ndarray, Dict]:
        super().reset(seed=seed)

        if self._physics_client is None:
            if self.render_mode == "human":
                self._physics_client = p.connect(p.GUI)
            else:
                self._physics_client = p.connect(p.DIRECT)

            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0, 0, -9.81)

        p.resetSimulation(self._physics_client)

        # 加载支气管 STL 模型
        # 注意：PyBullet 需要将 STL 转换为 OBJ 或通过 URDF 引用
        # 转换命令：trimesh --help 或使用 open3d
        collision_shape = p.createCollisionShape(
            p.GEOM_MESH,
            fileName=self.stl_path,
            physicsClientId=self._physics_client,
        )
        visual_shape = p.createVisualShape(
            p.GEOM_MESH,
            fileName=self.stl_path,
            physicsClientId=self._physics_client,
        )
        self._bronchus_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            physicsClientId=self._physics_client,
        )

        # 初始化内镜位置（支气管入口处）
        self._motor_angles = np.zeros(3)
        self._step_count = 0
        self._total_depth = 0.0

        obs = self._get_obs()
        return obs, {}

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        self._step_count += 1

        # 应用动作（角度增量）
        delta = action * self.ACTION_SCALE
        new_angles = self._motor_angles + delta

        # 限位
        for i, (lo, hi) in enumerate(self.MOTOR_LIMITS):
            new_angles[i] = np.clip(new_angles[i], lo, hi)

        self._motor_angles = new_angles

        # 物理仿真步进
        p.stepSimulation(self._physics_client)

        obs = self._get_obs()
        reward = self._compute_reward()
        terminated = self._check_termination()
        truncated = self._step_count >= self.max_steps

        info = {
            "step": self._step_count,
            "depth_mm": self._total_depth,
            "motor_angles": self._motor_angles.copy(),
        }
        return obs, reward, terminated, truncated, info

    def _get_obs(self) -> np.ndarray:
        """构建观测向量"""
        # 这里返回简化的特征向量
        # 真实部署时替换为来自视觉模块的特征
        visual_features = np.zeros(8, dtype=np.float32)  # 占位符

        motor_norm = np.array([
            self._motor_angles[0] / 900.0,       # M0 归一化
            self._motor_angles[1] / 170.0,        # M1 归一化
            self._motor_angles[2] / 500.0,        # M2 归一化
            0.0, 0.0, 0.0,                         # 速度估算（占位）
        ], dtype=np.float32)

        return np.concatenate([visual_features, motor_norm])

    def _compute_reward(self) -> float:
        """
        奖励函数设计：
        + 深度进展（前进奖励）
        - 气道壁碰撞
        + 到达岔口时的路径选择正确性
        """
        forward_reward = max(0, -self._motor_angles[0]) / 900.0 * 2.0
        collision_penalty = self._check_collision() * (-5.0)
        time_penalty = -0.01  # 每步轻微惩罚，鼓励快速到达
        return forward_reward + collision_penalty + time_penalty

    def _check_collision(self) -> bool:
        """检测与支气管壁的碰撞"""
        if self._scope_id is None or self._bronchus_id is None:
            return False
        contacts = p.getContactPoints(
            self._scope_id, self._bronchus_id,
            physicsClientId=self._physics_client
        )
        return len(contacts) > 0

    def _check_termination(self) -> bool:
        """检查终止条件"""
        if self._check_collision():
            return True
        if self._total_depth >= self.target_depth_mm:
            return True
        return False

    def render(self):
        if self.render_mode == "rgb_array":
            width, height = 640, 480
            view_mat = p.computeViewMatrix([0, 0, 0.1], [0, 0, -1], [0, 1, 0])
            proj_mat = p.computeProjectionMatrixFOV(60, width/height, 0.01, 100)
            _, _, rgb, _, _ = p.getCameraImage(
                width, height, view_mat, proj_mat,
                physicsClientId=self._physics_client
            )
            return np.array(rgb, dtype=np.uint8)[:, :, :3]

    def close(self):
        if self._physics_client is not None:
            p.disconnect(self._physics_client)
            self._physics_client = None
```

### 4.3 STL 模型转换

```bash
# 安装 trimesh（STL → OBJ 转换）
pip install trimesh

# 转换脚本
python -c "
import trimesh
mesh = trimesh.load('bronchus_model.stl')
mesh.export('bronchus_model.obj')
print('转换完成')
"
```

---

## 5 模型训练流程

### 5.1 Phase 1：Behavior Cloning

**新增文件**：`2026/ai_auto/train_bc.py`

```python
"""
Behavior Cloning 训练脚本
输入：expert_demos/*.json
输出：bc_policy.pth
"""

import json, os, glob
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from pathlib import Path


class ExpertDataset(Dataset):
    """专家演示数据集，使用滑动窗口生成时序样本"""

    def __init__(self, demo_dir: str, seq_len: int = 8):
        self.seq_len = seq_len
        self.samples = []
        self._load(demo_dir)

    def _load(self, demo_dir: str):
        for fpath in glob.glob(os.path.join(demo_dir, "expert_*.json")):
            with open(fpath, "r") as f:
                data = json.load(f)
            frames = data["frames"]
            for i in range(self.seq_len, len(frames) - 1):
                window = frames[i - self.seq_len: i]
                target = frames[i]
                # 构建观测序列
                obs_seq = []
                for fr in window:
                    v = fr.get("visual", {})
                    obs = [
                        fr["motor_0"] / 900.0,
                        fr["motor_1"] / 170.0,
                        fr["motor_2"] / 500.0,
                        v.get("bifurcation_cx", 0.5),
                        v.get("bifurcation_cy", 0.5),
                        v.get("depth_mean_mm", 0.0) / 300.0,
                        v.get("depth_max_mm", 0.0) / 300.0,
                        v.get("path_direction", 0) / 1.0,
                        float(v.get("has_bifurcation", False)),
                        v.get("segmentation_area", 0.0),
                    ]
                    obs_seq.append(obs)

                # 目标动作（相对上一帧的角度变化）
                prev = frames[i - 1]
                action = [
                    (target["motor_0"] - prev["motor_0"]) / 5.0,
                    (target["motor_1"] - prev["motor_1"]) / 3.0,
                    (target["motor_2"] - prev["motor_2"]) / 3.0,
                ]

                self.samples.append((
                    torch.tensor(obs_seq, dtype=torch.float32),   # [T, obs_dim]
                    torch.tensor(action, dtype=torch.float32),     # [3]
                ))

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        return self.samples[idx]


class BCPolicy(nn.Module):
    """
    Behavior Cloning 策略网络
    GRU 时序编码 + MLP 动作头
    """

    def __init__(self, obs_dim: int = 10, hidden_dim: int = 256, action_dim: int = 3):
        super().__init__()
        self.gru = nn.GRU(
            input_size=obs_dim,
            hidden_size=hidden_dim,
            num_layers=2,
            batch_first=True,
            dropout=0.1,
        )
        self.action_head = nn.Sequential(
            nn.Linear(hidden_dim, 128),
            nn.ReLU(),
            nn.Linear(128, action_dim),
            nn.Tanh(),  # 输出在 [-1, 1]
        )

    def forward(self, obs_seq: torch.Tensor) -> torch.Tensor:
        """
        obs_seq: [B, T, obs_dim]
        returns: [B, action_dim]
        """
        gru_out, _ = self.gru(obs_seq)
        last_hidden = gru_out[:, -1, :]  # 取最后时刻的隐状态
        return self.action_head(last_hidden)


def train_bc(
    demo_dir: str = "expert_demos",
    save_path: str = "bc_policy.pth",
    epochs: int = 100,
    batch_size: int = 64,
    lr: float = 1e-3,
):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"使用设备: {device}")

    dataset = ExpertDataset(demo_dir)
    print(f"加载 {len(dataset)} 个训练样本")

    train_size = int(0.9 * len(dataset))
    val_size = len(dataset) - train_size
    train_ds, val_ds = torch.utils.data.random_split(dataset, [train_size, val_size])

    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=batch_size)

    model = BCPolicy().to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
    criterion = nn.MSELoss()

    best_val_loss = float("inf")
    for epoch in range(epochs):
        model.train()
        train_losses = []
        for obs_seq, actions in train_loader:
            obs_seq, actions = obs_seq.to(device), actions.to(device)
            pred = model(obs_seq)
            loss = criterion(pred, actions)
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()
            train_losses.append(loss.item())

        model.eval()
        val_losses = []
        with torch.no_grad():
            for obs_seq, actions in val_loader:
                obs_seq, actions = obs_seq.to(device), actions.to(device)
                pred = model(obs_seq)
                val_losses.append(criterion(pred, actions).item())

        train_loss = np.mean(train_losses)
        val_loss = np.mean(val_losses)
        scheduler.step()

        if (epoch + 1) % 10 == 0:
            print(f"Epoch {epoch+1}/{epochs} | Train: {train_loss:.4f} | Val: {val_loss:.4f}")

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save(model.state_dict(), save_path)

    print(f"训练完成，最佳验证损失: {best_val_loss:.4f}，模型已保存到 {save_path}")
    return model


if __name__ == "__main__":
    train_bc()
```

### 5.2 Phase 2：PPO 强化学习

```bash
# 安装 stable-baselines3
pip install stable-baselines3[extra] gymnasium

# 训练脚本调用示例
python 2026/ai_auto/train_ppo.py \
    --stl_path bronchus_model.obj \
    --pretrained bc_policy.pth \
    --total_steps 1000000
```

**新增文件**：`2026/ai_auto/train_ppo.py`

```python
"""
PPO 强化学习训练（基于 BC 预训练策略暖启动）
"""
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback
from bronchus_env import BronchusEnv
import argparse


def train_ppo(stl_path: str, pretrained_path: str = None, total_steps: int = 1_000_000):
    env = BronchusEnv(stl_path=stl_path)
    eval_env = BronchusEnv(stl_path=stl_path)

    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        tensorboard_log="./ppo_logs/",
    )

    # 如果有 BC 预训练权重，加载到 policy 网络
    # （需要手动迁移权重或使用 SB3 的 policy 自定义接口）
    # 简化版：直接用 PPO 从零训练

    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path="./ppo_best/",
        log_path="./ppo_logs/",
        eval_freq=10000,
        n_eval_episodes=5,
        deterministic=True,
    )

    model.learn(total_timesteps=total_steps, callback=eval_callback)
    model.save("ppo_bronchus_final")
    print("PPO 训练完成")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--stl_path", required=True)
    parser.add_argument("--pretrained", default=None)
    parser.add_argument("--total_steps", type=int, default=1_000_000)
    args = parser.parse_args()
    train_ppo(args.stl_path, args.pretrained, args.total_steps)
```

---

## 6 AI-Auto 模式集成

在主程序 `main2026-Xhandwriting-AC-Auto(5.19depthtest)-Bronchus.py` 中新增 AI-Auto 模式。

**新增文件**：`2026/ai_auto/ai_policy_runner.py`

```python
"""
AI-Auto 策略推理器
在真机控制循环中调用
"""

import torch
import numpy as np
from collections import deque
from typing import Dict, Optional
from train_bc import BCPolicy   # 导入模型定义


class AIPolicyRunner:
    """
    AI策略推理器
    实时根据视觉特征+电机状态输出电机增量指令
    """

    def __init__(self, model_path: str, seq_len: int = 8, device: str = "auto"):
        if device == "auto":
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        else:
            self.device = torch.device(device)

        self.model = BCPolicy().to(self.device)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.eval()

        self.seq_len = seq_len
        self.obs_buffer: deque = deque(maxlen=seq_len)

        # 使用零帧初始化 buffer（冷启动）
        for _ in range(seq_len):
            self.obs_buffer.append(np.zeros(10, dtype=np.float32))

        print(f"[AIPolicyRunner] 模型加载完成，设备: {self.device}")

    def _build_obs(self, motor_angles: tuple, visual_features: Dict) -> np.ndarray:
        """构建单帧观测向量"""
        m0, m1, m2 = motor_angles
        v = visual_features
        return np.array([
            m0 / 900.0,
            m1 / 170.0,
            m2 / 500.0,
            v.get("cx", 0.5),
            v.get("cy", 0.5),
            v.get("depth_mean", 0.0) / 300.0,
            v.get("depth_max", 0.0) / 300.0,
            v.get("direction", 0.0),
            float(v.get("has_bifurcation", False)),
            v.get("seg_area", 0.0),
        ], dtype=np.float32)

    def predict(self, motor_angles: tuple, visual_features: Dict) -> np.ndarray:
        """
        推理一步
        
        Returns:
            action: [Δm0_deg, Δm1_deg, Δm2_deg]（真实角度变化量，已反归一化）
        """
        obs = self._build_obs(motor_angles, visual_features)
        self.obs_buffer.append(obs)

        obs_seq = torch.tensor(
            np.stack(list(self.obs_buffer)), dtype=torch.float32
        ).unsqueeze(0).to(self.device)

        with torch.no_grad():
            action_norm = self.model(obs_seq)[0].cpu().numpy()  # [-1, 1]

        # 反归一化到真实角度增量
        action_deg = action_norm * np.array([5.0, 3.0, 3.0])
        return action_deg

    def reset(self):
        """重置序列缓存（切换模式时调用）"""
        for _ in range(self.seq_len):
            self.obs_buffer.append(np.zeros(10, dtype=np.float32))
```

### 在主状态机中添加 AI-Auto 状态

在主程序文件的状态机中添加如下逻辑（在 `vision` 状态之后）：

```python
# 在主循环的 AI-Auto 模式处理函数中（约在现有 vision 模式之后新增）

def _loop_ai_auto(self):
    """AI-Auto 自主介入模式"""
    if not hasattr(self, '_ai_runner') or self._ai_runner is None:
        try:
            from ai_auto.ai_policy_runner import AIPolicyRunner
            self._ai_runner = AIPolicyRunner(
                model_path="2026/ai_auto/bc_policy.pth"
            )
            self._ai_runner.reset()
            self.voice_window.push_log("AI-Auto 模式已就绪")
        except Exception as e:
            self.voice_window.push_log(f"AI策略加载失败: {e}")
            self._on_command("manual")
            return

    # 获取当前电机状态
    motor_angles = self.motor_group.get_cached_angles()

    # 获取视觉特征（从视觉线程共享变量读取）
    visual_features = self._get_visual_features_for_ai()

    # 推理
    action = self._ai_runner.predict(motor_angles, visual_features)

    # 应用动作（带限位保护）
    new_m0 = motor_angles[0] + action[0]
    new_m1 = motor_angles[1] + action[1]
    new_m2 = motor_angles[2] + action[2]

    # 使用已有的 set_motor_position 接口执行
    self.motor_group.set_motor_position(0, new_m0, max_speed=30.0)
    self.motor_group.set_motor_position(1, new_m1, max_speed=20.0)
    self.motor_group.set_motor_position(2, new_m2, max_speed=20.0)

def _get_visual_features_for_ai(self) -> dict:
    """从当前视觉结果中提取AI策略所需特征"""
    # 读取 config 中由视觉线程写入的特征
    # 实际实现时替换为你的视觉模块共享变量
    return {
        "cx": 0.5, "cy": 0.5,
        "depth_mean": 50.0, "depth_max": 100.0,
        "direction": 0, "has_bifurcation": False, "seg_area": 0.3,
    }
```

---

## 7 一周上手计划

### Day 1（周一）— 环境搭建 + 数据分析

**上午（2h）**：安装依赖

```bash
pip install gymnasium stable-baselines3[extra] pybullet
pip install torch torchvision  # 已有则跳过
pip install trimesh open3d      # STL处理
pip install tensorboard          # 训练监控
```

**下午（3h）**：分析现有录制数据

```bash
# 查看现有录制文件
python -c "
import json, glob
for f in glob.glob('recorded_actions/*.json'):
    d = json.load(open(f))
    print(f, len(d['actions']), 'frames, duration:', d['duration'], 's')
"
```

**目标**：理解现有数据格式，规划视觉特征扩展方案

---

### Day 2（周二）— 扩展数据采集 + 首批录制

**上午（2h）**：
- 创建 `2026/ai_auto/` 目录
- 实现 `expert_recorder.py`（上文代码）
- 接入视觉特征回调

**下午（3h）**：
- 用现有视觉模块 + ExpertDemoRecorder 进行首批专家录制
- 录制 10-20 条直段前进 + 岔口转向数据
- 验证数据格式正确（用 Python 打开 JSON 检查）

---

### Day 3（周三）— Behavior Cloning 训练

**上午（2h）**：
- 实现 `train_bc.py`（上文代码）
- 数据预处理：检查数据质量，过滤异常帧

**下午（3h）**：
- 运行 BC 训练（100 epochs，CPU 也能跑，约 30 分钟）

```bash
cd 2026/ai_auto
python train_bc.py
```

- 监控训练曲线（loss 下降即为正常）
- 保存 `bc_policy.pth`

---

### Day 4（周四）— 仿真环境搭建

**上午（3h）**：
- 安装 PyBullet
- 导入支气管 STL 模型并转换为 OBJ

```bash
pip install pybullet pybullet_data
# 转换 STL
python -c "
import trimesh
mesh = trimesh.load('bronchus_model.stl')
mesh.export('bronchus_model.obj')
"
```

**下午（2h）**：
- 实现 `bronchus_env.py` 基础版（上文代码）
- 测试环境是否能正常 reset/step

```python
from bronchus_env import BronchusEnv
env = BronchusEnv("bronchus_model.obj", render_mode="human")
obs, _ = env.reset()
for _ in range(100):
    action = env.action_space.sample()
    obs, reward, done, _, _ = env.step(action)
    if done:
        obs, _ = env.reset()
```

---

### Day 5（周五）— PPO 训练启动

**全天（4h）**：

```bash
# 启动 PPO 训练（后台运行）
python 2026/ai_auto/train_ppo.py \
    --stl_path bronchus_model.obj \
    --total_steps 500000

# 启动 TensorBoard 监控
tensorboard --logdir ./ppo_logs/
```

- 观察 reward 曲线是否上升
- 调整奖励函数（如碰撞惩罚权重）

---

### Day 6（周六）— AI-Auto 模式集成

**上午（3h）**：
- 实现 `ai_policy_runner.py`
- 在主程序中添加 AI-Auto 状态（修改 state machine）
- 绑定 UI 按钮

**下午（2h）**：
- 离线测试：仅加载模型，打印推理结果，不连接电机
- 检查推理速度（目标 > 20Hz）

---

### Day 7（周日）— 集成测试 + 文档

**上午（2h）**：
- 连接真机（仅开低速限制），测试 AI-Auto 基础响应
- 观察电机动作是否与人工操作方向一致

**下午（2h）**：
- 更新 README
- 整理录制数据，规划下周录制计划

---

## 8 环境安装指令

```bash
# 基础依赖（在 Napoleon2025 环境中运行）
pip install gymnasium
pip install stable-baselines3[extra]
pip install pybullet
pip install pybullet_data
pip install trimesh
pip install tensorboard

# 验证安装
python -c "import gymnasium, stable_baselines3, pybullet; print('环境OK')"

# 如需 MuJoCo（可选，先用 PyBullet）
pip install mujoco
pip install gymnasium[mujoco]
```

---

## 9 目录结构规划

```
2026/
├── ai_auto/                     ← 新建
│   ├── expert_recorder.py       ← 扩展版专家录制器
│   ├── bronchus_env.py          ← PyBullet 仿真环境
│   ├── train_bc.py              ← BC 训练脚本
│   ├── train_ppo.py             ← PPO 训练脚本
│   ├── ai_policy_runner.py      ← 推理接口（供主程序调用）
│   ├── bc_policy.pth            ← BC 训练权重（训练后生成）
│   └── ppo_best/                ← PPO 最佳权重（训练后生成）
├── expert_demos/                ← 新建（专家演示数据）
│   ├── expert_20260527_143022.json
│   └── ...
├── ActionRecorder.py            ← 原有（保持不变）
├── main2026-RL-P0.py            ← 原有（深度导航基线）
├── main2026-Xhandwriting-AC-Auto(5.19depthtest)-Bronchus.py  ← 主程序（添加AI-Auto）
└── predict_wqx-Bronchus.py      ← 原有（视觉模块）
```

---

## 10 常见问题

**Q: STL 文件无法在 PyBullet 中加载？**  
A: PyBullet 偏好 OBJ 格式，使用 trimesh 转换：`trimesh.load("x.stl").export("x.obj")`

**Q: BC 训练后策略输出全是 0？**  
A: 检查数据集是否有效，确认专家演示有足够的动作变化量（threshold > 0.1度）

**Q: 推理速度不够（< 20Hz）？**  
A: 切换到 GPU (`device="cuda"`)，或减小 seq_len 到 4；GRU 推理本身很快

**Q: 仿真中机器人碰撞率过高？**  
A: 增加碰撞惩罚权重，或添加 curriculum learning（先在大支气管训练）

**Q: 如何从仿真迁移到真机（Sim2Real）？**  
A: 建议使用 Domain Randomization：在仿真中对视觉特征加噪声，训练时随机化管道直径，提高策略鲁棒性

---

*文档版本：2026-05-26 | 作者：ZQ*
