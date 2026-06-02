# -*- coding: utf-8 -*-
"""
BronchusDataset — BC 训练数据集
=================================
从 HDF5/npz 专家演示文件构建 PyTorch Dataset。

每个样本:
  obs_seq   [T, OBS_DIM]  — T 帧历史观测
  action    [3]           — 目标帧动作增量（归一化到 [-1,1]）
  task_label int          — 子任务标签（0=导航, 1=清理）
  goal_id   int           — 目标支气管标签

Author: ZQ  Date: 2026-05
"""

import os
import glob
import numpy as np
import torch
from torch.utils.data import Dataset, DataLoader, WeightedRandomSampler
from typing import List, Tuple, Optional

try:
    import h5py
    HDF5_AVAILABLE = True
except ImportError:
    HDF5_AVAILABLE = False

from model import OBS_FIELDS, ACTION_SCALE, NUM_GOALS


# ──────────────────────────────────────────────────────────────
# 单个 Episode 缓存
# ──────────────────────────────────────────────────────────────

class Episode:
    """内存中的单条演示轨迹。"""
    def __init__(self, obs: np.ndarray, actions: np.ndarray,
                 task_labels: np.ndarray, path_label: int, session: str = ""):
        """
        obs         : [N, OBS_DIM]  已归一化（按 OBS_FIELDS 的 scale）
        actions     : [N, 3]        原始角度增量（°）
        task_labels : [N]           0=导航 1=清理
        path_label  : int
        """
        assert len(obs) == len(actions) == len(task_labels), "长度不一致"
        self.obs         = obs.astype(np.float32)
        self.actions     = actions.astype(np.float32)
        self.task_labels = task_labels.astype(np.int64)
        self.path_label  = int(path_label)
        self.session     = session
        self.n_frames    = len(obs)

    def normalize_actions(self):
        """将动作归一化到 [-1, 1]，除以 ACTION_SCALE。"""
        scale = ACTION_SCALE.numpy()  # [5, 3, 3]
        self.actions = np.clip(self.actions / scale[None, :], -1.0, 1.0)
        return self

    def __len__(self):
        return self.n_frames

    def __repr__(self):
        return (f"Episode(session={self.session}, path={self.path_label}, "
                f"n_frames={self.n_frames})")


def load_episode_hdf5(fpath: str) -> Optional[Episode]:
    """从 HDF5 文件加载一条演示轨迹。"""
    if not HDF5_AVAILABLE:
        return None
    try:
        with h5py.File(fpath, "r") as hf:
            obs         = hf["obs"][:]           # [N, OBS_DIM]
            actions     = hf["actions"][:]       # [N, 3]
            task_labels = hf["task_labels"][:]   # [N]
            path_label  = int(hf.attrs.get("path_label", 0))
            session     = str(hf.attrs.get("session_name", fpath))
        ep = Episode(obs, actions, task_labels, path_label, session)
        ep.normalize_actions()
        return ep
    except Exception as e:
        print(f"[Dataset] 加载失败 {fpath}: {e}")
        return None


def load_episode_npz(fpath: str) -> Optional[Episode]:
    """从 npz 文件加载一条演示轨迹。"""
    try:
        d = np.load(fpath)
        obs         = d["obs"]
        actions     = d["actions"]
        task_labels = d["task_labels"]
        path_label  = int(d["path_label"][0])
        ep = Episode(obs, actions, task_labels, path_label, fpath)
        ep.normalize_actions()
        return ep
    except Exception as e:
        print(f"[Dataset] 加载失败 {fpath}: {e}")
        return None


def load_episode_json(fpath: str) -> Optional[Episode]:
    """
    从 JSON 文件加载一条演示轨迹（data_collector 默认格式）。
    与 ActionRecorder 的 JSON 格式兼容，可直接在 VS Code 中查看。
    """
    import json as _json
    try:
        with open(fpath, "r", encoding="utf-8") as fp:
            data = _json.load(fp)

        frames     = data.get("frames", [])
        path_label = int(data.get("path_label", 0))
        session    = data.get("session_name", fpath)

        if not frames:
            print(f"[Dataset] 空文件: {fpath}")
            return None

        # obs_fields 定义了观测向量的字段顺序
        obs_fields_names = data.get("obs_fields", [f[0] for f in OBS_FIELDS])
        n = len(frames)

        obs         = np.zeros((n, len(obs_fields_names)), dtype=np.float32)
        actions     = np.zeros((n, 3), dtype=np.float32)
        task_labels = np.zeros(n, dtype=np.int64)

        # 归一化因子（与 OBS_FIELDS 保持一致）
        scale_map = {name: scale for name, scale, _ in OBS_FIELDS}

        for i, fr in enumerate(frames):
            # 观测向量：按 obs_fields_names 顺序，用各自 scale 归一化
            for j, fname in enumerate(obs_fields_names):
                raw   = float(fr.get(fname, 0.0))
                scale = scale_map.get(fname, 1.0)
                obs[i, j] = raw / scale

            actions[i, 0]  = float(fr.get("delta_m0", 0.0))
            actions[i, 1]  = float(fr.get("delta_m1", 0.0))
            actions[i, 2]  = float(fr.get("delta_m2", 0.0))
            task_labels[i] = int(fr.get("task_label", 0))

        ep = Episode(obs, actions, task_labels, path_label, session)
        ep.normalize_actions()
        return ep
    except Exception as e:
        print(f"[Dataset] 加载失败 {fpath}: {e}")
        return None


# ──────────────────────────────────────────────────────────────
# PyTorch Dataset
# ──────────────────────────────────────────────────────────────

class BronchusDataset(Dataset):
    """
    支气管 BC 训练数据集（滑动窗口采样）。

    每个样本使用长度为 seq_len 的历史窗口作为观测，
    预测窗口末帧的动作。

    Parameters
    ----------
    demo_dir  : str   演示文件目录
    seq_len   : int   历史帧数 T（default=8）
    stride    : int   滑动步长（default=1，可设2减少冗余）
    augment   : bool  数据增强（加观测噪声）
    goal_filter : List[int] or None  只加载指定路径标签的演示
    """

    def __init__(
        self,
        demo_dir:    str,
        seq_len:     int   = 8,
        stride:      int   = 1,
        augment:     bool  = False,
        goal_filter: Optional[List[int]] = None,
    ):
        self.seq_len     = seq_len
        self.stride      = stride
        self.augment     = augment
        self.goal_filter = goal_filter

        self.episodes: List[Episode] = []
        self._load_all(demo_dir)

        # 构建样本索引: [(episode_idx, start_frame_idx), ...]
        self._indices: List[Tuple[int, int]] = []
        self._build_indices()

        print(f"[Dataset] 加载 {len(self.episodes)} 条轨迹 → {len(self._indices)} 个训练样本")
        self._print_distribution()

    def _load_all(self, demo_dir: str):
        if not os.path.exists(demo_dir):
            print(f"[Dataset] 目录不存在: {demo_dir}")
            return

        files = sorted(
            glob.glob(os.path.join(demo_dir, "*.json"))   # 默认格式（可直接查看）
            + glob.glob(os.path.join(demo_dir, "*.h5"))   # HDF5 格式
            + glob.glob(os.path.join(demo_dir, "*.npz"))  # npz 格式
        )

        for fpath in files:
            if fpath.endswith(".json"):
                ep = load_episode_json(fpath)
            elif fpath.endswith(".h5"):
                ep = load_episode_hdf5(fpath)
            else:
                ep = load_episode_npz(fpath)
            if ep is None:
                continue
            if self.goal_filter is not None and ep.path_label not in self.goal_filter:
                continue
            self.episodes.append(ep)

    def _build_indices(self):
        for ep_idx, ep in enumerate(self.episodes):
            # 需要至少 seq_len+1 帧（最后一帧作为动作目标）
            for start in range(0, ep.n_frames - self.seq_len, self.stride):
                self._indices.append((ep_idx, start))

    def _print_distribution(self):
        from collections import Counter
        counts = Counter(self.episodes[ep_i].path_label for ep_i, _ in self._indices)
        print("  路径分布:")
        for label, cnt in sorted(counts.items()):
            from model import BRONCHUS_PATHS, IDX_TO_YOLO, IDX_TO_NAME
            code = IDX_TO_YOLO.get(label, "?")
            name = IDX_TO_NAME.get(label, "?")
            print(f"    [{label}] {code:6s} {name:16s}: {cnt} 样本")

    def __len__(self):
        return len(self._indices)

    def __getitem__(self, idx: int):
        ep_idx, start = self._indices[idx]
        ep = self.episodes[ep_idx]

        end = start + self.seq_len
        obs_seq     = ep.obs[start:end].copy()       # [T, OBS_DIM]
        action      = ep.actions[end - 1].copy()     # [3]  最后帧的动作
        task_label  = int(ep.task_labels[end - 1])
        goal_id     = ep.path_label

        # 数据增强：观测向量加高斯噪声
        if self.augment:
            noise = np.random.randn(*obs_seq.shape).astype(np.float32) * 0.02
            obs_seq = np.clip(obs_seq + noise, -2.0, 2.0)

        return (
            torch.from_numpy(obs_seq),           # [T, OBS_DIM]
            torch.from_numpy(action),            # [3]
            torch.tensor(task_label, dtype=torch.long),
            torch.tensor(goal_id,   dtype=torch.long),
        )

    # ── 工具 ──────────────────────────────────────────────────

    def get_class_weights(self) -> torch.Tensor:
        """
        计算路径标签的类别权重（用于 WeightedRandomSampler，
        平衡各条支气管的样本数）。
        """
        labels = np.array([self.episodes[ep_i].path_label for ep_i, _ in self._indices])
        classes, counts = np.unique(labels, return_counts=True)
        weights = np.zeros(len(self._indices), dtype=np.float32)
        for cls, cnt in zip(classes, counts):
            weights[labels == cls] = 1.0 / cnt
        return torch.from_numpy(weights)

    def split(self, val_ratio: float = 0.1):
        """按 episode 分割训练/验证集（不打乱帧内顺序）。"""
        n_val  = max(1, int(len(self.episodes) * val_ratio))
        n_train = len(self.episodes) - n_val

        train_eps = self.episodes[:n_train]
        val_eps   = self.episodes[n_train:]

        train_ds = _SubsetDataset(self, train_eps, self.seq_len, self.stride, augment=self.augment)
        val_ds   = _SubsetDataset(self, val_eps,   self.seq_len, self.stride, augment=False)
        print(f"[Dataset] 分割: train={len(train_ds)} val={len(val_ds)}")
        return train_ds, val_ds


class _SubsetDataset(Dataset):
    """内部用：按 episode 列表重建索引的子集。"""
    def __init__(self, parent: BronchusDataset, eps: List[Episode],
                 seq_len: int, stride: int, augment: bool):
        self.parent   = parent
        self.episodes = eps
        self.seq_len  = seq_len
        self.stride   = stride
        self.augment  = augment
        self._indices = []
        for ep_idx, ep in enumerate(eps):
            for start in range(0, ep.n_frames - seq_len, stride):
                self._indices.append((ep_idx, start))

    def __len__(self):
        return len(self._indices)

    def __getitem__(self, idx: int):
        ep_idx, start = self._indices[idx]
        ep = self.episodes[ep_idx]
        end = start + self.seq_len
        obs_seq    = ep.obs[start:end].copy()
        action     = ep.actions[end - 1].copy()
        task_label = int(ep.task_labels[end - 1])
        goal_id    = ep.path_label

        if self.augment:
            obs_seq = np.clip(obs_seq + np.random.randn(*obs_seq.shape).astype(np.float32) * 0.02, -2.0, 2.0)

        return (
            torch.from_numpy(obs_seq),
            torch.from_numpy(action),
            torch.tensor(task_label, dtype=torch.long),
            torch.tensor(goal_id,   dtype=torch.long),
        )


# ──────────────────────────────────────────────────────────────
# DataLoader 工厂
# ──────────────────────────────────────────────────────────────

def create_dataloaders(
    demo_dir:   str,
    batch_size: int   = 64,
    seq_len:    int   = 8,
    val_ratio:  float = 0.1,
    num_workers: int  = 2,
    balance_paths: bool = True,
) -> Tuple[DataLoader, DataLoader]:
    """
    创建训练/验证 DataLoader。

    Parameters
    ----------
    balance_paths : bool
        True 时用 WeightedRandomSampler 平衡各条支气管样本数
    """
    full_ds = BronchusDataset(demo_dir, seq_len=seq_len, augment=True)
    train_ds, val_ds = full_ds.split(val_ratio)

    if balance_paths and len(train_ds) > 0:
        # 重新计算 train_ds 的权重
        labels = np.array([train_ds.episodes[ep_i].path_label for ep_i, _ in train_ds._indices])
        classes, counts = np.unique(labels, return_counts=True)
        w = np.zeros(len(train_ds), dtype=np.float32)
        for cls, cnt in zip(classes, counts):
            w[labels == cls] = 1.0 / cnt
        sampler = WeightedRandomSampler(torch.from_numpy(w), len(train_ds), replacement=True)
        train_loader = DataLoader(train_ds, batch_size=batch_size,
                                  sampler=sampler, num_workers=num_workers,
                                  pin_memory=True)
    else:
        train_loader = DataLoader(train_ds, batch_size=batch_size,
                                  shuffle=True, num_workers=num_workers,
                                  pin_memory=True)

    val_loader = DataLoader(val_ds, batch_size=batch_size,
                            shuffle=False, num_workers=num_workers,
                            pin_memory=True)
    return train_loader, val_loader


if __name__ == "__main__":
    # 快速验证（需要 expert_demos/ 目录存在）
    demo_dir = os.path.join(os.path.dirname(__file__), "expert_demos")
    if not os.path.exists(demo_dir):
        print(f"创建测试目录: {demo_dir}")
        os.makedirs(demo_dir)
        print("请先用 data_collector.py 录制专家演示数据")
    else:
        ds = BronchusDataset(demo_dir)
        print(f"Dataset 大小: {len(ds)}")
        if len(ds) > 0:
            obs, act, task, goal = ds[0]
            print(f"  obs_seq:  {obs.shape}  dtype={obs.dtype}")
            print(f"  action:   {act.shape}  min={act.min():.3f} max={act.max():.3f}")
            print(f"  task:     {task.item()}  goal: {goal.item()}")
