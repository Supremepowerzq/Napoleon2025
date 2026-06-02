# -*- coding: utf-8 -*-
"""
bc_main.py — BC 模块命令行入口
=================================
用法:
    python bc_main.py install_check          # 检查依赖
    python bc_main.py model_info             # 查看模型架构与观测空间
    python bc_main.py list                   # 列出已录制的演示文件
    python bc_main.py train [--epochs 150]   # 训练 BC 模型
    python bc_main.py test  [--path LMB]     # 离线推理验证

录制方式：在主程序 UI 点「开始录制」，无需在此脚本中操作。

Author: ZQ  Date: 2026-05
"""

import sys
import os
import argparse
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent.parent))

from model import (
    BRONCHUS_PATHS, IDX_TO_YOLO, IDX_TO_NAME, YOLO_CODE_TO_IDX,
    path_idx, model_summary, create_policy, OBS_DIM, OBS_FIELDS,
)
from data_collector import list_demos, update_visual_features


# ──────────────────────────────────────────────────────────────
# list
# ──────────────────────────────────────────────────────────────

def cmd_list(args):
    demo_dir = os.path.join(os.path.dirname(__file__), "expert_demos")
    demos = list_demos(demo_dir)
    if not demos:
        print(f"暂无演示文件: {demo_dir}")
        print("请在主程序 UI 中点「开始录制」采集数据。")
        print("\n支持的路径标签 (YOLO 代码):")
        for idx, (code, name) in BRONCHUS_PATHS.items():
            print(f"  {code:6s}  {name}")
        return

    total = sum(d.get("n_frames", 0) for d in demos)
    print(f"\n{'YOLO':6} {'支气管':14} {'帧数':7} {'时长(s)':8}  文件名")
    print("─" * 72)
    for d in demos:
        pl   = d.get("path_label", 0)
        code = IDX_TO_YOLO.get(pl, d.get("yolo_code", "?"))
        name = IDX_TO_NAME.get(pl, "unknown")
        print(f"  {code:6s}  {name:14s}  {d.get('n_frames',0):5d}   "
              f"{d.get('duration_s',0):6.1f}   {os.path.basename(d['file'])}")
    print("─" * 72)
    print(f"共 {len(demos)} 条  {total} 帧  约 {total/20:.0f}s 有效数据")


# ──────────────────────────────────────────────────────────────
# train
# ──────────────────────────────────────────────────────────────

def cmd_train(args):
    demo_dir = os.path.join(os.path.dirname(__file__), "expert_demos")
    demos = list_demos(demo_dir)
    if not demos:
        print("无演示数据！请先在主程序 UI 录制专家演示。")
        return

    total = sum(d.get("n_frames", 0) for d in demos)
    print(f"找到 {len(demos)} 条轨迹，共 {total} 帧，开始训练...")

    from train import train
    import argparse as ap
    train(ap.Namespace(
        demo_dir    = "expert_demos",
        epochs      = args.epochs,
        batch_size  = args.batch_size,
        seq_len     = 8,
        lr          = 3e-4,
        task_weight = 0.3,
        patience    = 30,
        resume      = args.resume or "",
    ))


# ──────────────────────────────────────────────────────────────
# test（离线推理验证，使用随机仿真观测）
# ──────────────────────────────────────────────────────────────

def cmd_test(args):
    ckpt = os.path.join(os.path.dirname(__file__), "checkpoints", "bc_best.pth")
    if not os.path.exists(ckpt):
        print(f"未找到 checkpoint: {ckpt}")
        print("请先训练: python bc_main.py train")
        return

    # --path 接受 YOLO 代码或整数索引
    goal = path_idx(args.path)
    code = IDX_TO_YOLO.get(goal, "?")
    name = IDX_TO_NAME.get(goal, "?")

    from inference import BCRunner
    import numpy as np

    runner = BCRunner(ckpt, goal_id=goal, action_scale_factor=0.5)
    print(f"\n模拟推理 20 步  目标: [{code}] {name}")
    print(f"{'步':4} {'new_m0':9} {'new_m1':9} {'new_m2':9}  {'任务':12}  {'置信'}")
    print("─" * 58)

    class _MockMotorGroup:
        def get_cached_angles(self): return (0.0, 0.0, 0.0)

    mg = _MockMotorGroup()
    for i in range(20):
        update_visual_features(
            stone_detected = (i % 7 == 0),
            stone_cx       = np.random.uniform(-0.3, 0.3),
            depth_mean_mm  = np.random.uniform(20, 100),
        )
        d = runner.step(mg)
        print(f"{i+1:4d}  {d.new_m0:+8.2f}  {d.new_m1:+8.2f}  {d.new_m2:+8.2f}"
              f"  {d.task_name:12s}  {d.task_prob:.3f}")
        time.sleep(0.05)

    print("\n✓ 推理验证完成")


# ──────────────────────────────────────────────────────────────
# model_info
# ──────────────────────────────────────────────────────────────

def cmd_model_info(args):
    model = create_policy("cpu")
    model_summary(model)

    print("\n支气管路径标签 (YOLO 代码 ↔ 整数索引):")
    print(f"  {'索引':5} {'YOLO':8} {'中文名'}")
    print("  " + "─" * 30)
    for idx, (code, name) in BRONCHUS_PATHS.items():
        print(f"  {idx:5d}  {code:8s}  {name}")

    print(f"\n观测向量 ({OBS_DIM} 维):")
    print(f"  {'索引':5} {'字段名':22} {'÷':8}  {'说明'}")
    print("  " + "─" * 68)
    for i, (fname, scale, desc) in enumerate(OBS_FIELDS):
        print(f"  {i:5d}  {fname:22s}  {scale:6.1f}  {desc}")


# ──────────────────────────────────────────────────────────────
# install_check
# ──────────────────────────────────────────────────────────────

def cmd_install_check(args):
    print("检查 BC 训练依赖...\n")
    all_ok = True
    for mod, desc, pkg in [
        ("torch",       "PyTorch",     "torch"),
        ("h5py",        "HDF5 (h5py)", "h5py"),
        ("tensorboard", "TensorBoard", "tensorboard"),
        ("numpy",       "NumPy",       "numpy"),
    ]:
        try:
            __import__(mod)
            print(f"  ✓ {desc}")
        except ImportError:
            print(f"  ✗ {desc}  →  pip install {pkg}")
            all_ok = False

    import torch
    cuda = torch.cuda.is_available()
    if cuda:
        name = torch.cuda.get_device_name(0)
        vram = torch.cuda.get_device_properties(0).total_memory / 1e9
        print(f"  ✓ CUDA  GPU={name}  VRAM={vram:.1f}GB")
    else:
        print(f"  ○ CUDA 不可用（将使用 CPU，训练速度较慢）")

    print()
    if all_ok:
        print("✓ 所有依赖就绪，可以开始训练。")
    else:
        print("请安装缺失依赖后重新检查:")
        print("  pip install h5py tensorboard")


# ──────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(
        prog="bc_main.py",
        description="支气管 BC 自主介入训练工具",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    sub = p.add_subparsers(dest="cmd")

    sub.add_parser("list",          help="列出已录制的演示文件")
    sub.add_parser("model_info",    help="显示模型架构和观测空间定义")
    sub.add_parser("install_check", help="检查依赖是否安装完整")

    t = sub.add_parser("train", help="训练 BC 模型")
    t.add_argument("--epochs",     type=int,   default=150,           help="训练轮数")
    t.add_argument("--batch_size", type=int,   default=64,            help="批大小")
    t.add_argument("--resume",     default="",                        help="断点续训 checkpoint 路径")

    ts = sub.add_parser("test", help="离线推理验证（随机仿真观测）")
    ts.add_argument("--path", default="EXP",
                    help="目标路径 YOLO 代码，如 LMB / RMB / RUL（默认 EXP）")

    args = p.parse_args()

    if args.cmd is None:
        p.print_help()
        print("\n快速开始:")
        print("  python bc_main.py install_check   # 检查依赖")
        print("  python bc_main.py model_info      # 查看观测空间")
        print("  python bc_main.py list            # 查看已录制数据")
        print("  python bc_main.py train           # 开始训练")
        print("  python bc_main.py test --path LMB # 验证推理")
        return

    {
        "list":          cmd_list,
        "train":         cmd_train,
        "test":          cmd_test,
        "model_info":    cmd_model_info,
        "install_check": cmd_install_check,
    }[args.cmd](args)


if __name__ == "__main__":
    import multiprocessing
    multiprocessing.freeze_support()
    main()
