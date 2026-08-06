# -*- coding: utf-8 -*-
"""
verify_demos.py — 采集质量检查工具
=====================================
用法:
    python verify_demos.py              # 检查所有文件
    python verify_demos.py --last 5     # 只看最新5条
    python verify_demos.py --path LMB   # 只看左主支气管
    python verify_demos.py --plot       # 画出最新一条的电机曲线

Author: ZQ  2026-06
"""

import os, sys, json, argparse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from model import IDX_TO_YOLO, IDX_TO_NAME, BRONCHUS_PATHS, OBS_FIELDS


DEMO_DIR = Path(__file__).parent / "expert_demos"
# 深度特征名（13个视觉特征中，检测它们是否全零来判断感知是否生效）
DEPTH_FIELDS  = ["depth_mean_mm", "depth_max_mm", "depth_center_mm"]
VISUAL_FIELDS = ["bifur_cx", "bifur_area", "obstruction_detected"]


def load_json(fpath):
    with open(fpath, "r", encoding="utf-8") as f:
        return json.load(f)


def check_file(fpath, verbose=True):
    """
    返回 dict 包含诊断信息：
      ok        : bool   — 总体是否合格
      issues    : list   — 问题列表
      n_frames  : int
      duration  : float  (s)
      fps       : float
      path      : str    YOLO 代码
      depth_ok  : bool   — 深度特征是否正常（非全零）
      visual_ok : bool   — 视觉特征是否正常
      action_std: tuple  — (std_m0, std_m1, std_m2)，衡量动作多样性
    """
    issues = []
    try:
        d = load_json(fpath)
    except Exception as e:
        return {"ok": False, "issues": [f"读取失败: {e}"], "file": str(fpath)}

    frames = d.get("frames", [])
    n = len(frames)
    dur = d.get("duration_s", 0.0)
    path_label = d.get("path_label", 0)
    yolo_code = d.get("yolo_code", IDX_TO_YOLO.get(path_label, "?"))
    path_name = d.get("path_name", IDX_TO_NAME.get(path_label, "?"))

    # 1. 帧数检查
    if n < 40:
        issues.append(f"帧数太少: {n} 帧（建议 ≥ 40，约 2 秒）")
    if dur < 1.5:
        issues.append(f"时长太短: {dur:.1f}s（建议 ≥ 2s）")

    # 2. 深度特征检查（是否全零 → 感知未接通）
    depth_ok = False
    if frames:
        import statistics
        d_vals = [fr.get("depth_mean_mm", 0.0) for fr in frames]
        if max(d_vals) > 0.5:
            depth_ok = True
        else:
            issues.append("⚠️  depth_mean_mm 全为 0 → Depth-Anything-V2 特征未写入！检查感知线程是否启动")

    # 3. 视觉特征检查（UNet 是否运行）
    visual_ok = False
    if frames:
        b_vals = [abs(fr.get("bifur_cx", 0.0)) + abs(fr.get("bifur_cy", 0.0))
                  + fr.get("bifur_area", 0.0) for fr in frames]
        if max(b_vals) > 0.01:
            visual_ok = True
        else:
            issues.append("⚠️  bifur_* 全为 0 → UNet 特征未写入，可能是摄像头未连接或感知未启动")

    # 4. 动作多样性检查（避免采集了静止不动的数据）
    import statistics as _st
    dm0 = [fr.get("delta_m0", 0.0) for fr in frames]
    dm1 = [fr.get("delta_m1", 0.0) for fr in frames]
    dm2 = [fr.get("delta_m2", 0.0) for fr in frames]
    std0 = _st.pstdev(dm0) if len(dm0) > 1 else 0.0
    std1 = _st.pstdev(dm1) if len(dm1) > 1 else 0.0
    std2 = _st.pstdev(dm2) if len(dm2) > 1 else 0.0
    if std0 < 0.01 and std1 < 0.01 and std2 < 0.01:
        issues.append("⚠️  三轴动作标准差均 < 0.01°，疑似采集了静止片段（未操作手柄？）")

    # 5. 路径标签检查
    if path_label == 0:
        issues.append("路径标签为 EXP（自主探索），确认是否故意采集探索类演示")

    ok = len(issues) == 0 or (len(issues) == 1 and "EXP" in issues[0])

    result = {
        "ok": ok,
        "file": os.path.basename(fpath),
        "path": yolo_code,
        "path_name": path_name,
        "n_frames": n,
        "duration": dur,
        "fps": round(n / max(dur, 1e-3), 1),
        "depth_ok": depth_ok,
        "visual_ok": visual_ok,
        "action_std": (round(std0, 3), round(std1, 3), round(std2, 3)),
        "issues": issues,
    }

    if verbose:
        status = "✅" if ok else "❌"
        print(f"{status} [{yolo_code:4s}] {path_name:14s}  {n:4d}帧  {dur:5.1f}s  fps={result['fps']:.0f}"
              f"  depth={'OK' if depth_ok else 'MISS':4s}  vis={'OK' if visual_ok else 'MISS':4s}"
              f"  std=({std0:.3f},{std1:.3f},{std2:.3f})  {os.path.basename(fpath)}")
        for iss in issues:
            print(f"   └─ {iss}")

    return result


def cmd_check(args):
    files = sorted(DEMO_DIR.glob("*.json"))
    if not files:
        print(f"暂无演示文件: {DEMO_DIR}")
        print("请先在主程序 UI 中点「开始采集」")
        return

    # 过滤
    if args.path:
        code = args.path.upper()
        files = [f for f in files if code in f.stem.upper()]

    if args.last:
        files = files[-args.last:]

    print(f"\n检查 {len(files)} 条演示文件 (目录: {DEMO_DIR})\n")
    print(f"{'状态':2} {'路径':6} {'目标支气管':14} {'帧数':5} {'时长(s)':7} "
          f"{'fps':4} {'深度':4} {'视觉':4}  文件名")
    print("─" * 95)

    results = [check_file(str(f)) for f in files]

    ok_cnt  = sum(1 for r in results if r.get("ok"))
    bad_cnt = len(results) - ok_cnt

    # 路径分布统计
    from collections import Counter
    dist = Counter(r.get("path", "?") for r in results if r.get("ok"))

    print("─" * 95)
    print(f"\n合格: {ok_cnt}  不合格: {bad_cnt}  总计: {len(results)}")
    print("\n路径分布（合格文件）:")
    target_dist = {
        "TR": 5, "LMB": 20, "RMB": 20,
        "LUB": 8, "LLB": 8, "RUL": 8, "BI": 8, "RML": 8, "RLL": 8, "EXP": 7,
    }
    for code, (_, name) in sorted(BRONCHUS_PATHS.items(), key=lambda x: x[0]):
        c = dist.get(BRONCHUS_PATHS[code][0], 0)
        target = target_dist.get(BRONCHUS_PATHS[code][0], 5)
        bar = "█" * c + "░" * max(0, target - c)
        flag = "✅" if c >= target else f"需要+{target - c}"
        print(f"  {BRONCHUS_PATHS[code][0]:4s} {name:14s}: {bar}  {c}/{target}  {flag}")


def cmd_plot(args):
    """画最新一条的电机曲线 + 深度曲线"""
    try:
        import matplotlib.pyplot as plt
        import matplotlib
        matplotlib.rcParams['font.family'] = ['SimHei', 'DejaVu Sans']
    except ImportError:
        print("请安装 matplotlib: pip install matplotlib")
        return

    files = sorted(DEMO_DIR.glob("*.json"))
    if not files:
        print("暂无演示文件")
        return

    fpath = files[-1] if not args.path else next(
        (f for f in reversed(files) if args.path.upper() in f.stem.upper()), files[-1]
    )
    d = load_json(str(fpath))
    frames = d["frames"]
    ts = [f["timestamp"] for f in frames]
    m0 = [f["m0_angle"] for f in frames]
    m1 = [f["m1_angle"] for f in frames]
    m2 = [f["m2_angle"] for f in frames]
    dm = [f.get("depth_mean_mm", 0) for f in frames]
    dc = [f.get("depth_center_mm", 0) for f in frames]
    pl = [f.get("path_left_mm", 0) for f in frames]
    pr = [f.get("path_right_mm", 0) for f in frames]

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    ax1, ax2 = axes

    ax1.plot(ts, m0, label="M0(前进)", color="#e74c3c")
    ax1.plot(ts, m1, label="M1(左右)", color="#2ecc71")
    ax1.plot(ts, m2, label="M2(上下)", color="#3498db")
    ax1.set_ylabel("电机角度 (°)")
    ax1.legend(loc="upper right")
    ax1.set_title(f"[{d.get('yolo_code','?')}] {d.get('path_name','?')}  "
                  f"{len(frames)}帧  {d.get('duration_s',0):.1f}s")
    ax1.grid(alpha=0.3)

    ax2.plot(ts, dm,  label="深度均值(mm)", color="#9b59b6")
    ax2.plot(ts, dc,  label="中心深度(mm)", color="#e67e22", linestyle="--")
    ax2.plot(ts, pl,  label="左路深度", color="#1abc9c", alpha=0.6)
    ax2.plot(ts, pr,  label="右路深度", color="#e74c3c", alpha=0.6)
    ax2.set_ylabel("深度 (mm)")
    ax2.set_xlabel("时间 (s)")
    ax2.legend(loc="upper right")
    ax2.grid(alpha=0.3)

    plt.tight_layout()
    plt.show()
    print(f"\n已绘制: {fpath.name}")


def main():
    p = argparse.ArgumentParser(description="BC 演示采集质量检查工具")
    sub = p.add_subparsers(dest="cmd")

    ck = sub.add_parser("check", help="检查采集质量（默认命令）")
    ck.add_argument("--last", type=int, default=0, help="只显示最新N条")
    ck.add_argument("--path", default="", help="按YOLO路径代码过滤，如 LMB / RMB")

    pl = sub.add_parser("plot", help="绘制最新一条的电机和深度曲线")
    pl.add_argument("--path", default="", help="指定YOLO代码，如 LMB")

    args = p.parse_args()

    # 默认走 check
    if args.cmd is None or args.cmd == "check":
        if not hasattr(args, "last"):
            args.last = 0
        if not hasattr(args, "path"):
            args.path = ""
        cmd_check(args)
    elif args.cmd == "plot":
        cmd_plot(args)


if __name__ == "__main__":
    main()
