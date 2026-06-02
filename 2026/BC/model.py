# -*- coding: utf-8 -*-
"""
BC Policy Network — BronchusPolicy
====================================
目标条件化 GRU 策略网络，用于支气管主干自主介入。

输入（每帧 OBS_DIM 维结构化特征 × T 帧历史 + 目标条件）
输出（电机增量动作 3 维 + 子任务分类 2 类）

Author: ZQ  Date: 2026-05
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from dataclasses import dataclass
from typing import Optional, Tuple

# ──────────────────────────────────────────────────────────────
# 常量
# ──────────────────────────────────────────────────────────────

# 结构化观测向量各字段：(名称, 归一化除数, 说明)
# 前16维：视觉+位置；后4维：速度+时间步长
OBS_FIELDS = [
    # ── 视觉特征（来自 UNet/HSV + DepthAnythingV2）─────────────
    ("stone_cx",         1.0,    "结石质心 x，已归一化到 [-1,1]"),
    ("stone_cy",         1.0,    "结石质心 y，已归一化到 [-1,1]"),
    ("stone_area",       1.0,    "结石像素面积占比 [0,1]"),
    ("stone_detected",   1.0,    "是否检测到结石 0/1"),
    ("bifur_cx",         1.0,    "岔口质心 x 归一化"),
    ("bifur_cy",         1.0,    "岔口质心 y 归一化"),
    ("bifur_area",       1.0,    "岔口面积占比 [0,1]"),
    ("depth_center_mm",  300.0,  "图像中心深度 mm → /300"),
    ("depth_mean_mm",    300.0,  "管腔平均深度 mm → /300"),
    ("depth_max_mm",     300.0,  "最深点深度 mm → /300"),
    ("path_left_mm",     300.0,  "左路径深度 mm → /300"),
    ("path_center_mm",   300.0,  "中路径深度 mm → /300"),
    ("path_right_mm",    300.0,  "右路径深度 mm → /300"),
    # ── 电机位置 ──────────────────────────────────────────────
    ("m0_angle",         900.0,  "M0 前进角度 → /900"),
    ("m1_angle",         170.0,  "M1 左右角度 → /170"),
    ("m2_angle",         500.0,  "M2 上下角度 → /500"),
    # ── 电机速度（°/s，由相邻帧角度差/dt 估算）───────────────
    # 速度让模型知道当前运动状态，避免 GRU 需要自行推算动量
    ("m0_vel",           90.0,   "M0 速度 °/s → /90（最大速度约 90°/s）"),
    ("m1_vel",           30.0,   "M1 速度 °/s → /30"),
    ("m2_vel",           50.0,   "M2 速度 °/s → /50"),
    # ── 时间步长（帧间隔）────────────────────────────────────
    # 录制暂停、手柄操作停顿时 dt 会变大；推理时也可能帧率波动
    # 归一化：dt / 0.05（20Hz 对应 1.0；10Hz 对应 2.0）
    ("dt",               0.05,   "帧间隔 s → /0.05（20Hz=1.0）"),
]
OBS_DIM = len(OBS_FIELDS)  # 20

# 动作缩放（将 tanh 输出 [-1,1] 映射到实际角度增量 °/帧）
ACTION_SCALE = torch.tensor([5.0, 3.0, 3.0])  # [Δm0, Δm1, Δm2]

# ── 支气管路径标签 ─────────────────────────────────────────────
# 与 YOLO 部位识别状态机保持一致（字母代码），同时维护整数索引供 Embedding 使用
BRONCHUS_PATHS = {
    0: ("EXP", "自主探索"),      # 不指定目标，自主探索
    1: ("TR",  "气管"),
    2: ("LMB", "左主支气管"),
    3: ("RMB", "右主支气管"),
    4: ("LUB", "左上叶"),
    5: ("LLB", "左下叶"),
    6: ("RUL", "右上叶"),
    7: ("BI",  "中间支气管"),
    8: ("RML", "右中叶"),
    9: ("RLL", "右下叶"),
}
NUM_GOALS = len(BRONCHUS_PATHS)  # 10

# YOLO 代码 → 整数索引（供 nn.Embedding 使用）
YOLO_CODE_TO_IDX = {v[0]: k for k, v in BRONCHUS_PATHS.items()}
# 整数索引 → YOLO 代码
IDX_TO_YOLO = {k: v[0] for k, v in BRONCHUS_PATHS.items()}
# 整数索引 → 中文名
IDX_TO_NAME = {k: v[1] for k, v in BRONCHUS_PATHS.items()}


def path_idx(code_or_idx) -> int:
    """将 YOLO 代码（'LMB'）或整数索引转为 Embedding 整数索引。"""
    if isinstance(code_or_idx, str):
        return YOLO_CODE_TO_IDX.get(code_or_idx.upper(), 0)
    return int(code_or_idx)

# 子任务标签
TASK_NAVIGATE = 0   # 导航前进
TASK_CLEAR     = 1  # 清理结石/黏液


# ──────────────────────────────────────────────────────────────
# 观测向量构建工具
# ──────────────────────────────────────────────────────────────

def build_obs_vector(features: dict) -> torch.Tensor:
    """
    将视觉处理结果字典打包为标准化观测向量。

    Parameters
    ----------
    features : dict
        键名对应 OBS_FIELDS 中的名称，值为 float。
        缺失字段自动填 0。

    Returns
    -------
    torch.Tensor  shape (OBS_DIM,)  float32
    """
    vec = []
    for name, scale, _ in OBS_FIELDS:
        raw = float(features.get(name, 0.0))
        vec.append(raw / scale)
    return torch.tensor(vec, dtype=torch.float32)


def obs_from_config_and_motors(
    stone_cx: float, stone_cy: float, stone_area: float, stone_detected: bool,
    bifur_cx: float, bifur_cy: float, bifur_area: float,
    depth_center_mm: float, depth_mean_mm: float, depth_max_mm: float,
    path_depths: tuple,       # (left, center, right) mm
    motor_angles: tuple,      # (m0, m1, m2) °
    motor_velocities: tuple = (0.0, 0.0, 0.0),  # (v0, v1, v2) °/s
    dt: float = 0.05,         # 帧间隔 s（20Hz → 0.05）
) -> torch.Tensor:
    """快捷构建函数，直接从现有系统变量构建 20 维观测向量。"""
    feat = {
        "stone_cx": stone_cx,
        "stone_cy": stone_cy,
        "stone_area": stone_area,
        "stone_detected": float(stone_detected),
        "bifur_cx": bifur_cx,
        "bifur_cy": bifur_cy,
        "bifur_area": bifur_area,
        "depth_center_mm": depth_center_mm,
        "depth_mean_mm": depth_mean_mm,
        "depth_max_mm": depth_max_mm,
        "path_left_mm": path_depths[0],
        "path_center_mm": path_depths[1],
        "path_right_mm": path_depths[2],
        "m0_angle": motor_angles[0],
        "m1_angle": motor_angles[1],
        "m2_angle": motor_angles[2],
        "m0_vel": motor_velocities[0],
        "m1_vel": motor_velocities[1],
        "m2_vel": motor_velocities[2],
        "dt": dt,
    }
    return build_obs_vector(feat)


# ──────────────────────────────────────────────────────────────
# 网络模块
# ──────────────────────────────────────────────────────────────

class ObsEncoder(nn.Module):
    """
    将单帧结构化观测向量编码为隐向量。
    OBS_DIM → hidden_dim
    """
    def __init__(self, obs_dim: int = OBS_DIM, hidden_dim: int = 128):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 128),
            nn.LayerNorm(128),
            nn.ELU(),
            nn.Linear(128, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.ELU(),
        )

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        """obs: [..., OBS_DIM] → [..., hidden_dim]"""
        return self.net(obs)


class BronchusPolicy(nn.Module):
    """
    目标条件化 GRU 策略网络
    ─────────────────────────
    输入:
      obs_seq  [B, T, OBS_DIM]  — T 帧观测历史
      goal_id  [B]              — 目标支气管标签（整数）
    输出:
      action   [B, 3]           — 归一化电机增量 tanh∈[-1,1]
      task_logits [B, 2]        — 子任务分类 logits（导航/清理）
    """
    def __init__(
        self,
        obs_dim:    int = OBS_DIM,
        goal_dim:   int = 16,
        enc_dim:    int = 128,
        gru_hidden: int = 256,
        gru_layers: int = 2,
        action_dim: int = 3,
        num_goals:  int = NUM_GOALS,
    ):
        super().__init__()
        self.obs_dim    = obs_dim
        self.gru_hidden = gru_hidden
        self.gru_layers = gru_layers

        # 观测编码器
        self.obs_encoder = ObsEncoder(obs_dim, enc_dim)

        # 目标嵌入
        self.goal_embed = nn.Embedding(num_goals, goal_dim)

        # GRU 时序编码
        gru_in = enc_dim + goal_dim
        self.gru = nn.GRU(
            input_size=gru_in,
            hidden_size=gru_hidden,
            num_layers=gru_layers,
            batch_first=True,
            dropout=0.1 if gru_layers > 1 else 0.0,
        )

        # 动作头（导航）
        self.action_head = nn.Sequential(
            nn.Linear(gru_hidden, 128),
            nn.ELU(),
            nn.Linear(128, action_dim),
            nn.Tanh(),
        )

        # 子任务分类头（导航 vs 清理结石）
        self.task_head = nn.Sequential(
            nn.Linear(gru_hidden, 64),
            nn.ELU(),
            nn.Linear(64, 2),
        )

        self._init_weights()

    def _init_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.orthogonal_(m.weight, gain=0.5)
                if m.bias is not None:
                    nn.init.zeros_(m.bias)
            elif isinstance(m, nn.GRU):
                for name, param in m.named_parameters():
                    if "weight" in name:
                        nn.init.orthogonal_(param)
                    elif "bias" in name:
                        nn.init.zeros_(param)

    def forward(
        self,
        obs_seq:  torch.Tensor,               # [B, T, OBS_DIM]
        goal_id:  torch.Tensor,               # [B]
        hidden:   Optional[torch.Tensor] = None,  # GRU 隐状态
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Returns
        -------
        action      [B, 3]          tanh ∈ [-1, 1]
        task_logits [B, 2]
        new_hidden  [num_layers, B, gru_hidden]
        """
        B, T, _ = obs_seq.shape

        # 编码观测序列
        obs_enc = self.obs_encoder(obs_seq)   # [B, T, enc_dim]

        # 目标嵌入扩展到时序维度
        goal = self.goal_embed(goal_id)        # [B, goal_dim]
        goal_seq = goal.unsqueeze(1).expand(-1, T, -1)  # [B, T, goal_dim]

        # 拼接后送 GRU
        gru_in = torch.cat([obs_enc, goal_seq], dim=-1)   # [B, T, enc_dim+goal_dim]
        gru_out, new_hidden = self.gru(gru_in, hidden)    # [B, T, gru_hidden]

        # 取最后时刻隐状态
        last = gru_out[:, -1, :]   # [B, gru_hidden]

        action      = self.action_head(last)    # [B, 3]
        task_logits = self.task_head(last)       # [B, 2]

        return action, task_logits, new_hidden

    def get_initial_hidden(self, batch_size: int, device: torch.device) -> torch.Tensor:
        return torch.zeros(self.gru_layers, batch_size, self.gru_hidden, device=device)


# ──────────────────────────────────────────────────────────────
# 模型工厂 & 工具
# ──────────────────────────────────────────────────────────────

def create_policy(device: str = "auto") -> BronchusPolicy:
    """创建标准策略网络并移至指定设备。"""
    if device == "auto":
        device = "cuda" if torch.cuda.is_available() else "cpu"
    return BronchusPolicy().to(device)


def save_checkpoint(model: BronchusPolicy, path: str, epoch: int, val_loss: float):
    import os
    os.makedirs(os.path.dirname(path), exist_ok=True)
    torch.save({
        "epoch": epoch,
        "val_loss": val_loss,
        "model_state": model.state_dict(),
        "obs_fields": OBS_FIELDS,
        "action_scale": ACTION_SCALE.tolist(),
        "bronchus_paths": BRONCHUS_PATHS,
    }, path)
    print(f"[Checkpoint] 已保存 epoch={epoch}, val_loss={val_loss:.4f} → {path}")


def load_checkpoint(path: str, device: str = "auto") -> Tuple[BronchusPolicy, dict]:
    if device == "auto":
        device = "cuda" if torch.cuda.is_available() else "cpu"
    ckpt = torch.load(path, map_location=device)
    model = BronchusPolicy().to(device)
    model.load_state_dict(ckpt["model_state"])
    model.eval()
    print(f"[Checkpoint] 已加载 epoch={ckpt['epoch']}, val_loss={ckpt['val_loss']:.4f}")
    return model, ckpt


def model_summary(model: BronchusPolicy):
    total = sum(p.numel() for p in model.parameters())
    trainable = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"BronchusPolicy: {total/1e6:.2f}M 参数，{trainable/1e6:.2f}M 可训练")
    print(f"  OBS_DIM={OBS_DIM}  NUM_GOALS={NUM_GOALS}")
    print(f"  ACTION_SCALE={ACTION_SCALE.tolist()}")


if __name__ == "__main__":
    model = create_policy("cpu")
    model_summary(model)

    # 验证前向推理
    B, T = 2, 8
    obs  = torch.randn(B, T, OBS_DIM)
    goal = torch.randint(0, NUM_GOALS, (B,))
    act, task_logits, hidden = model(obs, goal)
    print(f"action:      {act.shape}  {act.min().item():.3f} ~ {act.max().item():.3f}")
    print(f"task_logits: {task_logits.shape}")
    print(f"hidden:      {hidden.shape}")
