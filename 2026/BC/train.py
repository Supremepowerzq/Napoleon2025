# -*- coding: utf-8 -*-
"""
BC 训练脚本 — 行为克隆
========================
用法:
    python train.py                          # 默认参数
    python train.py --demo_dir expert_demos --epochs 150 --batch_size 64
    python train.py --resume checkpoints/bc_best.pth  # 断点续训

Author: ZQ  Date: 2026-05
"""

import os
import sys
import argparse
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.amp import GradScaler, autocast
from torch.utils.tensorboard import SummaryWriter

# 确保 BC 目录的上级（2026/）也在 path 中
sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent.parent))

from model import (
    BronchusPolicy, ACTION_SCALE,
    save_checkpoint, load_checkpoint, model_summary,
    TASK_NAVIGATE, TASK_CLEAR,
)
from dataset import create_dataloaders, BronchusDataset


# ──────────────────────────────────────────────────────────────
# 损失函数
# ──────────────────────────────────────────────────────────────

def bc_loss(
    pred_action:  torch.Tensor,   # [B, 3]  tanh 输出
    true_action:  torch.Tensor,   # [B, 3]  归一化真值
    pred_task:    torch.Tensor,   # [B, 2]  logits
    true_task:    torch.Tensor,   # [B]     0/1
    task_weight:  float = 0.3,    # 子任务分类损失权重
) -> tuple:
    """
    组合损失 = MSE(动作) + CE(子任务分类)

    Returns
    -------
    total_loss, nav_loss, task_loss
    """
    # 动作 MSE（对导航帧和清理帧都计算）
    nav_loss  = F.mse_loss(pred_action, true_action)

    # 子任务分类交叉熵
    task_loss = F.cross_entropy(pred_task, true_task)

    total = nav_loss + task_weight * task_loss
    return total, nav_loss, task_loss


# ──────────────────────────────────────────────────────────────
# 单 epoch 训练/验证
# ──────────────────────────────────────────────────────────────

def run_epoch(
    model:      BronchusPolicy,
    loader,
    optimizer:  torch.optim.Optimizer,
    scaler:     GradScaler,
    device:     torch.device,
    train:      bool = True,
    task_weight: float = 0.3,
) -> dict:
    model.train(train)
    totals = dict(total=0.0, nav=0.0, task=0.0, task_acc=0.0, n=0)

    ctx = torch.enable_grad() if train else torch.no_grad()
    with ctx:
        for obs_seq, actions, task_labels, goal_ids in loader:
            obs_seq     = obs_seq.to(device, non_blocking=True)    # [B,T,OBS]
            actions     = actions.to(device, non_blocking=True)    # [B,3]
            task_labels = task_labels.to(device, non_blocking=True) # [B]
            goal_ids    = goal_ids.to(device, non_blocking=True)   # [B]

            with autocast("cuda", enabled=(device.type == "cuda")):
                pred_act, pred_task, _ = model(obs_seq, goal_ids)
                loss, nav_l, task_l = bc_loss(
                    pred_act, actions, pred_task, task_labels, task_weight
                )

            if train:
                scaler.scale(loss).backward()
                scaler.unscale_(optimizer)
                torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
                scaler.step(optimizer)
                scaler.update()
                optimizer.zero_grad(set_to_none=True)

            B = obs_seq.size(0)
            task_acc = (pred_task.argmax(1) == task_labels).float().mean().item()
            totals["total"]    += loss.item()    * B
            totals["nav"]      += nav_l.item()   * B
            totals["task"]     += task_l.item()  * B
            totals["task_acc"] += task_acc       * B
            totals["n"]        += B

    n = totals.pop("n")
    return {k: v / n for k, v in totals.items()}


# ──────────────────────────────────────────────────────────────
# 训练主函数
# ──────────────────────────────────────────────────────────────

def train(args):
    # ── 设备 ───────────────────────────────────────────────
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"设备: {device}")
    if device.type == "cuda":
        print(f"  GPU: {torch.cuda.get_device_name(0)}")
        print(f"  VRAM: {torch.cuda.get_device_properties(0).total_memory/1e9:.1f} GB")

    # ── 数据 ───────────────────────────────────────────────
    demo_dir = os.path.join(os.path.dirname(__file__), args.demo_dir)
    # Windows 上多进程 DataLoader 开销大于收益（数据集小时），用 0 反而更快
    _nw = 0 if (os.name == "nt" and len(os.listdir(demo_dir)) < 50) else min(4, os.cpu_count() or 1)
    train_loader, val_loader = create_dataloaders(
        demo_dir      = demo_dir,
        batch_size    = args.batch_size,
        seq_len       = args.seq_len,
        val_ratio     = 0.1,
        num_workers   = _nw,
        balance_paths = True,
    )
    if len(train_loader.dataset) == 0:
        print("训练集为空！请先录制专家演示数据（运行 python 1.py record）")
        return

    # ── 模型 ───────────────────────────────────────────────
    start_epoch = 0
    if args.resume and os.path.exists(args.resume):
        model, ckpt = load_checkpoint(args.resume, str(device))
        start_epoch = ckpt["epoch"] + 1
        print(f"从 epoch {start_epoch} 继续训练")
    else:
        model = BronchusPolicy().to(device)
    model_summary(model)

    # ── 优化器 ─────────────────────────────────────────────
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=args.lr, weight_decay=1e-4
    )
    # OneCycleLR：快速收敛，不容易过拟合
    total_steps = args.epochs * len(train_loader)
    scheduler = torch.optim.lr_scheduler.OneCycleLR(
        optimizer, max_lr=args.lr,
        total_steps=total_steps,
        pct_start=0.1,          # 前 10% warmup
        anneal_strategy="cos",
    )
    scaler = GradScaler("cuda", enabled=(device.type == "cuda"))

    # ── 日志 ───────────────────────────────────────────────
    ckpt_dir = os.path.join(os.path.dirname(__file__), "checkpoints")
    os.makedirs(ckpt_dir, exist_ok=True)
    log_dir  = os.path.join(os.path.dirname(__file__), "runs",
                             time.strftime("bc_%Y%m%d_%H%M%S"))
    writer   = SummaryWriter(log_dir)
    print(f"TensorBoard: tensorboard --logdir {os.path.abspath(log_dir)}")
    print(f"Checkpoint 保存至: {ckpt_dir}")
    print(f"开始训练: {args.epochs} epochs  batch={args.batch_size}  seq_len={args.seq_len}")
    print("─" * 60)

    # ── 训练循环 ───────────────────────────────────────────
    best_val_loss = float("inf")
    no_improve    = 0

    for epoch in range(start_epoch, args.epochs):
        t0 = time.time()

        train_metrics = run_epoch(model, train_loader, optimizer, scaler, device,
                                   train=True, task_weight=args.task_weight)
        val_metrics   = run_epoch(model, val_loader,   optimizer, scaler, device,
                                   train=False, task_weight=args.task_weight)

        scheduler.step()
        elapsed = time.time() - t0
        lr      = optimizer.param_groups[0]["lr"]

        # TensorBoard
        for k, v in train_metrics.items():
            writer.add_scalar(f"train/{k}", v, epoch)
        for k, v in val_metrics.items():
            writer.add_scalar(f"val/{k}",   v, epoch)
        writer.add_scalar("lr", lr, epoch)

        # 打印
        if (epoch + 1) % 10 == 0 or epoch < 5:
            print(f"Epoch {epoch+1:4d}/{args.epochs} | "
                  f"Train total={train_metrics['total']:.4f} nav={train_metrics['nav']:.4f} "
                  f"task_acc={train_metrics['task_acc']:.3f} | "
                  f"Val total={val_metrics['total']:.4f} nav={val_metrics['nav']:.4f} "
                  f"task_acc={val_metrics['task_acc']:.3f} | "
                  f"lr={lr:.2e}  {elapsed:.1f}s")

        # Checkpoint
        if val_metrics["nav"] < best_val_loss:
            best_val_loss = val_metrics["nav"]
            no_improve    = 0
            save_checkpoint(
                model,
                os.path.join(ckpt_dir, "bc_best.pth"),
                epoch, val_metrics["nav"]
            )
        else:
            no_improve += 1

        # 每 50 epoch 保存一次快照
        if (epoch + 1) % 50 == 0:
            save_checkpoint(
                model,
                os.path.join(ckpt_dir, f"bc_epoch{epoch+1}.pth"),
                epoch, val_metrics["nav"]
            )

        # 早停（防止过拟合）
        if args.patience > 0 and no_improve >= args.patience:
            print(f"早停：验证损失 {args.patience} 个 epoch 未改善")
            break

    writer.close()
    print(f"\n训练完成！最佳验证导航损失: {best_val_loss:.4f}")
    print(f"最佳模型: {os.path.join(ckpt_dir, 'bc_best.pth')}")


# ──────────────────────────────────────────────────────────────
# DAgger 迭代（第二阶段，可选）
# ──────────────────────────────────────────────────────────────

def dagger_collect_and_retrain(
    model_path: str,
    new_demo_dir: str,
    args,
):
    """
    DAgger 流程说明:
    ─────────────────
    1. 加载已训练的 BC 策略
    2. 将策略部署到真实机器人上运行（用 inference.py 的 BCRunner）
    3. 同时启动 BronchusDataCollector 录制（专家随时可以用手柄纠正）
    4. 纠正时的动作 = 专家真实动作（覆盖策略输出）
    5. 结束后调用本函数：合并新旧数据 → 重新训练

    本函数只处理数据合并和重新训练，Step 2-4 需在主程序中完成。
    """
    import shutil

    base_demo_dir = os.path.join(os.path.dirname(__file__), args.demo_dir)
    new_files = [f for f in os.listdir(new_demo_dir) if f.endswith((".h5", ".npz"))]
    print(f"[DAgger] 新录制 {len(new_files)} 条轨迹，合并到训练集...")

    for f in new_files:
        shutil.copy(
            os.path.join(new_demo_dir, f),
            os.path.join(base_demo_dir, f"dagger_{f}")
        )

    print(f"[DAgger] 重新训练...")
    args.resume = model_path
    train(args)


# ──────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="支气管 BC 训练")
    p.add_argument("--demo_dir",    default="expert_demos",  help="演示数据目录")
    p.add_argument("--epochs",      type=int,   default=150)
    p.add_argument("--batch_size",  type=int,   default=64)
    p.add_argument("--seq_len",     type=int,   default=8,    help="GRU 历史帧数")
    p.add_argument("--lr",          type=float, default=3e-4)
    p.add_argument("--task_weight", type=float, default=0.3,  help="子任务损失权重")
    p.add_argument("--patience",    type=int,   default=30,   help="早停轮次 (0=关闭)")
    p.add_argument("--resume",      default="",              help="断点续训 checkpoint 路径")
    return p.parse_args()


if __name__ == "__main__":
    # Windows 多进程必须加此保护
    import multiprocessing
    multiprocessing.freeze_support()

    args = parse_args()
    train(args)
