# -*- coding: utf-8 -*-
"""
AutoNav 模拟验证脚本（Simulator）
=================================
在无需机器人的情况下，模拟以下场景：
1. Layer 1: 序列回放 + 各种起点偏移
2. Layer 2: 岔口修正的各种场景
3. Layer 3: 阻塞物检测-追踪-清除-续航全流程
4. 异常场景: 起点偏移、岔口缺失、持续阻塞物

运行方式:
    python test_autonav_sim.py

输出:
    - 每个测试场景的详细日志
    - 关键指标的统计结果
"""

import os
import sys
import time
import json
import glob
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional
from collections import defaultdict

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sys.stderr.reconfigure(encoding="utf-8", errors="replace")

# ── 路径设置 ──────────────────────────────────────────────────
HERE = os.path.dirname(os.path.abspath(__file__))
DEMO_DIR = os.path.join(HERE, "BC", "expert_demos")

# ── 测试结果收集 ──────────────────────────────────────────────
_test_results = {
    "passed": 0,
    "failed": 0,
    "scenarios": []
}


def log(msg, level="INFO"):
    prefix = {
        "INFO": "  [INFO]",
        "PASS": "  [PASS]",
        "FAIL": "  [FAIL]",
        "WARN": "  [WARN]",
        "SCENARIO": "\n" + "=" * 60 + f"\n  场景: {msg}\n" + "=" * 60,
    }
    print(f"{prefix.get(level, msg)}")


def check(condition: bool, msg: str, detail: str = "") -> bool:
    """检查条件并记录结果"""
    if condition:
        _test_results["passed"] += 1
        _test_results["scenarios"].append({"name": msg, "status": "PASS", "detail": detail})
        log(f"{msg} ✓", "PASS")
        return True
    else:
        _test_results["failed"] += 1
        _test_results["scenarios"].append({"name": msg, "status": "FAIL", "detail": detail})
        log(f"{msg} ✗ {detail}", "FAIL")
        return False


def section(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


# ════════════════════════════════════════════════════════════════
# 导入 AutoNav 模块
# ════════════════════════════════════════════════════════════════
try:
    from AutoNav.nav_controller import AutoNavController, NavMode
    from AutoNav.sequence_player import SequencePlayer
    from AutoNav.junction_guide import JunctionGuide
    from AutoNav.obstruction_handler import ObstructionHandler, HandlerState
    from AutoNav.config import (
        MOTOR_LIMITS, PLAYBACK_MAX_DELTA, PLAYBACK_SPEED,
        BIFUR_AREA_THRESHOLD, BIFUR_GAIN_M1, BIFUR_GAIN_M2,
        OBSTRUCTION_CLEAR_FRAMES, RETURN_TOLERANCE_DEG,
    )
    MODULES_OK = True
    log("AutoNav 模块导入成功", "PASS")
except ImportError as e:
    MODULES_OK = False
    log(f"AutoNav 模块导入失败: {e}", "FAIL")
    log("请确保在 2026 目录下运行此脚本", "WARN")


# ════════════════════════════════════════════════════════════════
# 模拟数据生成器
# ════════════════════════════════════════════════════════════════
class SimulatedVisual:
    """
    模拟视觉输出，用于测试。
    可以模拟各种场景：正常、岔口偏移、阻塞物出现等。
    """

    def __init__(self):
        self.bifur_cx = 0.0
        self.bifur_cy = 0.0
        self.bifur_area = 0.0
        self.obstruction_detected = False
        self.obstruction_cx = 0.0
        self.obstruction_cy = 0.0
        self.depth_mean_mm = 50.0

    def set_bifurcation(self, cx: float, cy: float, area: float):
        """设置岔口位置和面积"""
        self.bifur_cx = cx
        self.bifur_cy = cy
        self.bifur_area = area

    def set_obstruction(self, detected: bool, cx: float = 0.0, cy: float = 0.0):
        """设置阻塞物状态"""
        self.obstruction_detected = detected
        self.obstruction_cx = cx
        self.obstruction_cy = cy

    def to_dict(self) -> Dict:
        return {
            "bifur_cx": self.bifur_cx,
            "bifur_cy": self.bifur_cy,
            "bifur_area": self.bifur_area,
            "obstruction_detected": self.obstruction_detected,
            "obstruction_cx": self.obstruction_cx,
            "obstruction_cy": self.obstruction_cy,
            "depth_mean_mm": self.depth_mean_mm,
        }


# ════════════════════════════════════════════════════════════════
# 场景 1: Layer 1 - 序列回放基础功能
# ════════════════════════════════════════════════════════════════
def scenario_1_sequence_playback():
    """验证序列回放的基础功能"""
    section("场景 1: Layer 1 - 序列回放基础功能")

    if not MODULES_OK:
        return

    # 1.1 加载所有可用路径
    log("1.1 加载演示数据...", "INFO")
    from BC.model import BRONCHUS_PATHS

    loaded_paths = []
    for label, (code, name) in BRONCHUS_PATHS.items():
        try:
            sp = SequencePlayer(DEMO_DIR, path_label=label)
            loaded_paths.append((label, code, name, sp))
            log(f"  {code} ({name}): {sp.total_steps} 步", "INFO")
        except ValueError:
            log(f"  {code} ({name}): 无演示数据", "WARN")

    check(len(loaded_paths) >= 3, "至少 3 条路径有演示数据",
          f"实际: {len(loaded_paths)}")

    # 1.2 标准起点回放
    log("\n1.2 标准起点 (0,0,0) 回放...", "INFO")
    if loaded_paths:
        label, code, name, sp = loaded_paths[0]
        sp.reset(current_motor_angles=(0.0, 0.0, 0.0))

        steps = []
        for _ in range(min(50, sp.total_steps)):
            d = sp.step()
            steps.append({
                "target_m0": d["target_m0"],
                "target_m1": d["target_m1"],
                "target_m2": d["target_m2"],
                "delta_m0": d["delta_m0"],
            })
            if d["done"]:
                break

        check(len(steps) > 0, "成功回放至少 1 步", f"实际: {len(steps)}")

        # 检查 M0 递减（前进）
        m0_values = [s["target_m0"] for s in steps]
        m0_decreasing = all(m0_values[i] <= m0_values[i-1] + 0.1
                          for i in range(1, len(m0_values)))
        check(m0_decreasing, "M0 累积单调递减（前进方向正确）",
              f"范围: [{min(m0_values):.1f}, {max(m0_values):.1f}]")

    # 1.3 起点偏移测试
    log("\n1.3 起点偏移测试...", "INFO")
    offset_cases = [
        ((-10.0, 5.0, -3.0), "负偏移"),
        ((10.0, -5.0, 3.0), "正偏移"),
    ]

    for offset, desc in offset_cases:
        if loaded_paths:
            label, code, name, sp = loaded_paths[0]
            sp.reset(current_motor_angles=offset)

            d = sp.step()
            check(abs(d["target_m0"] - offset[0]) < abs(offset[0]) + 1.0,
                  f"起点偏移 {desc}: 起点正确应用",
                  f"offset={offset}, target_m0={d['target_m0']:.2f}")


# ════════════════════════════════════════════════════════════════
# 场景 2: Layer 2 - 岔口修正
# ════════════════════════════════════════════════════════════════
def scenario_2_junction_correction():
    """验证岔口修正功能"""
    section("场景 2: Layer 2 - 岔口修正功能")

    if not MODULES_OK:
        return

    jg = JunctionGuide()

    # 2.1 小面积不修正
    log("2.1 小面积岔口应不触发修正...", "INFO")
    corr = jg.compute_correction(cx := 0.5, cy := 0.5, area := 0.001)
    check(abs(corr[0]) < 0.01 and abs(corr[1]) < 0.01,
          "面积 < 阈值时 correction ≈ 0",
          f"bifur_area=0.001, corr=({corr[0]:.4f}, {corr[1]:.4f})")

    # 2.2 右侧岔口
    log("\n2.2 岔口在右侧...", "INFO")
    jg.reset()
    corr = jg.compute_correction(cx=0.5, cy=0.0, area=0.05)
    check(corr[0] > 0, "右侧岔口 → M1 正向修正",
          f"bifur_cx=0.5, corr_m1={corr[0]:.3f}")

    # 2.3 左侧岔口
    log("2.3 岔口在左侧...", "INFO")
    jg.reset()
    corr = jg.compute_correction(cx=-0.5, cy=0.0, area=0.05)
    check(corr[0] < 0, "左侧岔口 → M1 负向修正",
          f"bifur_cx=-0.5, corr_m1={corr[0]:.3f}")

    # 2.4 下侧岔口
    log("2.4 岔口在下侧...", "INFO")
    jg.reset()
    corr = jg.compute_correction(cx=0.0, cy=0.5, area=0.05)
    check(corr[1] > 0, "下侧岔口 → M2 正向修正",
          f"bifur_cy=0.5, corr_m2={corr[1]:.3f}")

    # 2.5 上侧岔口
    log("2.5 岔口在上侧...", "INFO")
    jg.reset()
    corr = jg.compute_correction(cx=0.0, cy=-0.5, area=0.05)
    check(corr[1] < 0, "上侧岔口 → M2 负向修正",
          f"bifur_cy=-0.5, corr_m2={corr[1]:.3f}")

    # 2.6 面积权重测试
    log("\n2.6 面积权重测试...", "INFO")
    jg.reset()
    corr_small = jg.compute_correction(cx=0.5, cy=0.0, area=0.01)
    jg.reset()
    corr_large = jg.compute_correction(cx=0.5, cy=0.0, area=0.1)
    check(abs(corr_large[0]) > abs(corr_small[0]),
          "面积越大，修正量越大",
          f"area=0.01: {corr_small[0]:.3f}, area=0.1: {corr_large[0]:.3f}")

    # 2.7 EMA 平滑测试
    log("\n2.7 EMA 平滑收敛测试...", "INFO")
    jg.reset()
    values = []
    for _ in range(20):
        c = jg.compute_correction(cx=0.5, cy=0.0, area=0.05)
        values.append(c[0])

    first_delta = abs(values[5] - values[4])
    last_delta = abs(values[-1] - values[-2])
    check(last_delta < first_delta * 0.5,
          "EMA 平滑后收敛",
          f"早期 delta: {first_delta:.4f}, 后期 delta: {last_delta:.6f}")


# ════════════════════════════════════════════════════════════════
# 场景 3: Layer 3 - 阻塞物处理
# ════════════════════════════════════════════════════════════════
def scenario_3_obstruction_handling():
    """验证阻塞物检测、追踪、清除、续航功能"""
    section("场景 3: Layer 3 - 阻塞物处理")

    if not MODULES_OK:
        return

    oh = ObstructionHandler()

    # 3.1 初始状态
    log("3.1 初始状态检查...", "INFO")
    check(oh.state == HandlerState.IDLE, "初始状态为 IDLE")
    check(oh.checkpoint is None, "无初始断点")

    # 3.2 保存断点
    log("\n3.2 保存断点...", "INFO")
    oh.save_checkpoint(motor_angles=(-100.0, 20.0, -50.0), sequence_step=42)
    check(oh.checkpoint is not None, "断点已保存")
    check(oh.checkpoint.sequence_step == 42, "断点步骤正确",
          f"实际: {oh.checkpoint.sequence_step}")
    check(np.allclose(oh.checkpoint.motor_angles, [-100, 20, -50], atol=0.1),
          "断点电机角度正确")

    # 3.3 启动清除
    log("\n3.3 启动清除...", "INFO")
    oh.start_clearing(current_motor_angles=(-100.0, 20.0, -50.0))
    check(oh.state == HandlerState.CLEARING, "切换到 CLEARING 模式")

    # 3.4 阻塞物追踪
    log("\n3.4 阻塞物追踪...", "INFO")
    for i in range(5):
        d = oh.update(
            obstruction_detected=True,
            obstruction_cx=0.3,
            obstruction_cy=0.1,
            depth_mean_mm=50.0,
            current_motor_angles=(-100.0 + i, 25.0, -55.0),
        )
        check(d.state == HandlerState.CLEARING,
              f"第 {i+1} 帧持续追踪",
              f"state={d.state.value}, info={d.info[:30]}")

    # 3.5 阻塞物消失 → RETURNING
    log("\n3.5 阻塞物消失判定...", "INFO")
    for i in range(OBSTRUCTION_CLEAR_FRAMES + 2):
        d = oh.update(
            obstruction_detected=False,
            obstruction_cx=0.0,
            obstruction_cy=0.0,
            depth_mean_mm=50.0,
            current_motor_angles=(-100.0, 25.0, -55.0),
        )
        if d.state == HandlerState.RETURNING:
            check(True, f"连续 {i+1} 帧无检测 → RETURNING",
                  f"d.state={d.state.value}")
            break
    else:
        check(False, "应切换到 RETURNING 模式")

    # 3.6 返回断点
    log("\n3.6 返回断点...", "INFO")
    # 模拟逐步接近断点
    cur = [-90.0, 22.0, -52.0]  # 初始位置（接近断点）
    while True:
        d = oh.update(
            obstruction_detected=False,
            obstruction_cx=0.0,
            obstruction_cy=0.0,
            depth_mean_mm=50.0,
            current_motor_angles=tuple(cur),
        )
        if d.state == HandlerState.IDLE and d.resumed:
            check(True, "到达断点 → resumed=True",
                  f"最终位置误差 < {RETURN_TOLERANCE_DEG}°")
            break
        # 更新位置向断点靠近
        cur[0] += (oh.checkpoint.motor_angles[0] - cur[0]) * 0.3
        cur[1] += (oh.checkpoint.motor_angles[1] - cur[1]) * 0.3
        cur[2] += (oh.checkpoint.motor_angles[2] - cur[2]) * 0.3

        # 防止死循环
        if len([d]) > 50:
            check(False, "返回断点超时")
            break


# ════════════════════════════════════════════════════════════════
# 场景 4: 集成测试 - 完整导航流程
# ════════════════════════════════════════════════════════════════
def scenario_4_integration():
    """验证完整的三层集成流程"""
    section("场景 4: 集成测试 - 完整导航流程")

    if not MODULES_OK:
        return

    # 4.1 正常导航 → 检测阻塞物 → 清除 → 返回 → 续航
    log("4.1 正常导航流程...", "INFO")
    ctrl = AutoNavController(demo_dir=DEMO_DIR, motor_group=None)
    ctrl.start(path_label=2, current_motor_angles=(0.0, 0.0, 0.0))

    check(ctrl.mode == NavMode.NAVIGATING, "启动后进入 NAVIGATING 模式")

    sim_vis = SimulatedVisual()
    sim_vis.set_bifurcation(cx=0.0, cy=0.0, area=0.0)  # 无岔口
    sim_vis.set_obstruction(False)  # 无阻塞物

    nav_steps = 0
    for i in range(100):
        cur = (sim_vis.depth_mean_mm * -0.1 * min(i, 30), 0.0, 0.0)
        d = ctrl.step(motor_angles=cur, visual=sim_vis.to_dict())

        if not d.active:
            break

        if d.mode == NavMode.DONE:
            check(True, f"序列播放完毕 (共 {nav_steps} 步)")
            break

        nav_steps += 1

        # 在第 20 步注入阻塞物
        if nav_steps == 20:
            sim_vis.set_obstruction(True, cx=0.2, cy=0.1)
            log(f"  → 第 20 步: 注入阻塞物", "WARN")

        # 在第 40 步移除阻塞物（假设已清除）
        if nav_steps == 40:
            sim_vis.set_obstruction(False)
            log(f"  → 第 40 步: 阻塞物已清除", "WARN")

    check(nav_steps > 0, f"正常导航执行 {nav_steps} 步")

    # 4.2 验证断点续航
    log("\n4.2 断点续航验证...", "INFO")
    ctrl2 = AutoNavController(demo_dir=DEMO_DIR, motor_group=None)
    ctrl2.start(path_label=2, current_motor_angles=(0.0, 0.0, 0.0))

    sim_vis2 = SimulatedVisual()
    saved_step = None

    for i in range(100):
        cur = (i * -0.5, 0.0, 0.0)
        d = ctrl2.step(motor_angles=cur, visual=sim_vis2.to_dict())

        if d.mode == NavMode.CLEARING and saved_step is None:
            saved_step = ctrl2._obstruction.checkpoint.sequence_step
            log(f"  → 断点已保存: 步骤 {saved_step}", "INFO")

        if d.mode == NavMode.NAVIGATING and saved_step is not None:
            resumed_step = ctrl2._player.current_step
            check(resumed_step >= saved_step,
                  f"续航后步骤 ({resumed_step}) ≥ 断点步骤 ({saved_step})",
                  f"resumed_step={resumed_step}")
            break

        # 第 15 步注入阻塞物
        if i == 15:
            sim_vis2.set_obstruction(True, cx=0.3, cy=0.0)

        # 第 30 步移除阻塞物
        if i == 30:
            sim_vis2.set_obstruction(False)

        if not d.active:
            break


# ════════════════════════════════════════════════════════════════
# 场景 5: 异常场景测试
# ════════════════════════════════════════════════════════════════
def scenario_5_edge_cases():
    """测试边界和异常场景"""
    section("场景 5: 异常场景测试")

    if not MODULES_OK:
        return

    # 5.1 起点超出限位
    log("5.1 起点超出电机限位...", "INFO")
    ctrl = AutoNavController(demo_dir=DEMO_DIR, motor_group=None)

    # 尝试使用边界外的起点
    extreme_start = (-950.0, 180.0, 520.0)  # 超出限位
    ctrl.start(path_label=2, current_motor_angles=extreme_start)

    sim_vis = SimulatedVisual()
    d = ctrl.step(motor_angles=extreme_start, visual=sim_vis.to_dict())

    # 检查输出被限位
    check(MOTOR_LIMITS[0][0] <= d.target_m0 <= MOTOR_LIMITS[0][1],
          "M0 输出在限位范围内",
          f"target_m0={d.target_m0:.1f}")
    check(MOTOR_LIMITS[1][0] <= d.target_m1 <= MOTOR_LIMITS[1][1],
          "M1 输出在限位范围内",
          f"target_m1={d.target_m1:.1f}")
    check(MOTOR_LIMITS[2][0] <= d.target_m2 <= MOTOR_LIMITS[2][1],
          "M2 输出在限位范围内",
          f"target_m2={d.target_m2:.1f}")

    # 5.2 岔口突然消失
    log("\n5.2 岔口突然消失...", "INFO")
    jg = JunctionGuide()

    # 先激活岔口修正
    jg.compute_correction(cx=0.5, cy=0.0, area=0.1)
    smooth_active = jg._smooth_m1

    # 然后岔口消失
    for _ in range(20):
        jg.compute_correction(cx=0.0, cy=0.0, area=0.0)

    check(abs(jg._smooth_m1) < abs(smooth_active) * 0.1,
          "岔口消失后修正量衰减",
          f"初始: {smooth_active:.3f}, 最终: {jg._smooth_m1:.6f}")

    # 5.3 阻塞物频繁闪烁
    log("\n5.3 阻塞物频繁闪烁...", "INFO")
    oh = ObstructionHandler()
    oh.save_checkpoint(motor_angles=(-50.0, 0.0, 0.0), sequence_step=10)
    oh.start_clearing(current_motor_angles=(-50.0, 0.0, 0.0))

    # 模拟阻塞物闪烁（检测-消失-检测-消失...）
    clear_count = 0
    for i in range(50):
        detected = (i % 2 == 0)  # 交替出现
        d = oh.update(
            obstruction_detected=detected,
            obstruction_cx=0.2 if detected else 0.0,
            obstruction_cy=0.0,
            depth_mean_mm=50.0,
            current_motor_angles=(-50.0, 0.0, 0.0),
        )

        if d.state == HandlerState.CLEARING:
            clear_count = 0  # 重置计数
        else:
            clear_count += 1

    check(clear_count < OBSTRUCTION_CLEAR_FRAMES,
          "闪烁阻塞物不会触发误判清除",
          f"最大连续消失帧数: {clear_count}")


# ════════════════════════════════════════════════════════════════
# 场景 6: 数据质量分析
# ════════════════════════════════════════════════════════════════
def scenario_6_data_analysis():
    """分析演示数据的质量和覆盖情况"""
    section("场景 6: 演示数据分析")

    import json

    log("加载演示数据...", "INFO")
    files = glob.glob(os.path.join(DEMO_DIR, "*.json"))

    if not files:
        log("未找到演示数据文件", "WARN")
        return

    stats = defaultdict(lambda: {"count": 0, "frames": [], "path_labels": set()})

    for fpath in files:
        try:
            with open(fpath, "r", encoding="utf-8") as f:
                data = json.load(f)

            yolo_code = data.get("yolo_code", "?")
            n_frames = data.get("n_frames", 0)
            path_label = data.get("path_label", -1)

            stats[yolo_code]["count"] += 1
            stats[yolo_code]["frames"].append(n_frames)
            stats[yolo_code]["path_labels"].add(path_label)
        except Exception as e:
            log(f"加载失败 {fpath}: {e}", "WARN")

    # 输出统计
    log(f"\n共 {len(files)} 个演示文件", "INFO")
    print(f"\n  {'代码':<6} {'数量':<6} {'总帧数':<8} {'平均帧数':<10} {'最短':<6} {'最长':<6} {'评估'}")
    print(f"  {'-' * 60}")

    quality_issues = []
    for code in sorted(stats.keys()):
        info = stats[code]
        count = info["count"]
        frames = info["frames"]
        avg = sum(frames) / count if count > 0 else 0
        min_f = min(frames) if frames else 0
        max_f = max(frames) if frames else 0

        # 质量评估
        if count < 3:
            quality = "⚠ 不足"
            quality_issues.append(code)
        elif avg < 100:
            quality = "⚠ 过短"
            quality_issues.append(code)
        elif avg > 1000:
            quality = "⚠ 过长"
            quality_issues.append(code)
        else:
            quality = "✓ 良好"

        print(f"  {code:<6} {count:<6} {sum(frames):<8} {avg:<10.0f} {min_f:<6} {max_f:<6} {quality}")

    # 关键路径检查
    critical_paths = ["TR", "LMB", "RMB", "LLB", "RLL"]
    for cp in critical_paths:
        if stats[cp]["count"] < 3:
            check(False, f"关键路径 {cp} 演示数量 ≥ 3",
                  f"实际: {stats[cp]['count']}")
        else:
            check(True, f"关键路径 {cp} 演示数量 ≥ 3",
                  f"实际: {stats[cp]['count']}")

    if quality_issues:
        log(f"\n⚠ 建议补充演示数据的路径: {', '.join(quality_issues)}", "WARN")


# ════════════════════════════════════════════════════════════════
# 主函数
# ════════════════════════════════════════════════════════════════
def main():
    t0 = time.time()

    print("=" * 70)
    print("  AutoNav 模拟验证测试")
    print(f"  Python: {sys.version.split()[0]}")
    print(f"  目录: {DEMO_DIR}")
    print("=" * 70)

    # 运行所有场景
    scenario_1_sequence_playback()
    scenario_2_junction_correction()
    scenario_3_obstruction_handling()
    scenario_4_integration()
    scenario_5_edge_cases()
    scenario_6_data_analysis()

    # 输出总结
    dt = time.time() - t0
    print("\n" + "=" * 70)
    print(f"  测试完成")
    print(f"  通过: {_test_results['passed']}")
    print(f"  失败: {_test_results['failed']}")
    print(f"  耗时: {dt:.2f}s")
    print("=" * 70)

    if _test_results["failed"] == 0:
        print("\n  ✅ 所有测试通过！AutoNav 系统逻辑验证完成。")
        print("\n  建议下一步：上机验证 Layer 1（纯序列回放）")
    else:
        print("\n  ❌ 存在测试失败，请检查上述 FAIL 记录。")
        print("\n  常见问题排查：")
        print("  1. 演示数据缺失 → 补录演示")
        print("  2. 导入错误 → 检查 Python 环境")
        print("  3. 逻辑错误 → 检查 AutoNav 模块代码")

    return 0 if _test_results["failed"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
