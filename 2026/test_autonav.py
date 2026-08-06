# -*- coding: utf-8 -*-
"""
AutoNav Offline Unit Self-Test
==============================
Hardware-free pure-logic validation for the three AutoNav layers:

  Layer 1  SequencePlayer      - sequence loading, mean trajectory, playback
  Layer 2  JunctionGuide       - bifurcation P-control, area threshold, EMA smoothing
  Layer 3  ObstructionHandler  - state machine, breakpoint resume, return tolerance
  Integration  AutoNavController  - end-to-end: nav -> detect -> clear -> return -> resume

Run (in the Napoleon_2026 conda env):
    python test_autonav.py

Pass criteria
-------------
All ASSERTs must pass, the final line prints `[ALL PASS]`.
Layer 1 also prints per-path mean-trajectory stats (total travel, per-step size).
"""
import os
import sys
import time
import numpy as np

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sys.stderr.reconfigure(encoding="utf-8", errors="replace")

# ── Path setup ──────────────────────────────────────────────────
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
DEMO_DIR = os.path.join(HERE, "BC", "expert_demos")

# ── Simple ASSERT macro ─────────────────────────────────────────
_results = {"pass": 0, "fail": 0, "cases": []}


def check(cond, msg, detail=""):
    if cond:
        _results["pass"] += 1
        _results["cases"].append(("PASS", msg, detail))
        print(f"  PASS  {msg}")
    else:
        _results["fail"] += 1
        _results["cases"].append(("FAIL", msg, detail))
        print(f"  FAIL  {msg}  {detail}")


def section(title):
    print(f"\n{'=' * 70}\n  {title}\n{'=' * 70}")


# ════════════════════════════════════════════════════════════════
# Layer 1 - SequencePlayer
# ════════════════════════════════════════════════════════════════
def test_layer1_sequence_player():
    from AutoNav.sequence_player import SequencePlayer
    from AutoNav.config import (
        MOTOR_LIMITS, PLAYBACK_MAX_DELTA, PLAYBACK_SPEED,
    )
    from BC.model import BRONCHUS_PATHS

    section("Layer 1 - SequencePlayer  (loading & mean trajectory)")

    loaded = {}
    for label, (code, name) in BRONCHUS_PATHS.items():
        try:
            sp = SequencePlayer(DEMO_DIR, path_label=label)
            loaded[label] = (code, sp)
            n = sp.total_steps
            cum = sp._avg_cumulative
            check(n > 20, f"[{code}]{name} loaded, n_steps={n}")
            check(cum.shape == (n, 3), f"  trajectory shape {cum.shape}")
            check(np.allclose(cum[0], 0, atol=1e-3),
                  f"  start cumulative ~= 0 ({cum[0]})")
            end = cum[-1]
            in_limit = all(
                MOTOR_LIMITS[i][0] - 1 <= end[i] <= MOTOR_LIMITS[i][1] + 1
                for i in range(3)
            )
            check(in_limit, f"  end cumulative within motor limits  end={tuple(end.round(1))}")
        except ValueError as e:
            print(f"  SKIP  [{label}]{code} {name} : {e}")

    # 1.2 - Single-step playback (TR path)
    if 1 in loaded:
        _, sp = loaded[1]
        sp.reset(current_motor_angles=(0.0, 0.0, 0.0))
        deltas = []
        targets = []
        for _ in range(5):
            d = sp.step()
            deltas.append((d["delta_m0"], d["delta_m1"], d["delta_m2"]))
            targets.append((d["target_m0"], d["target_m1"], d["target_m2"]))
        max_abs = max(max(abs(x) for x in step) for step in deltas)
        check(max_abs <= max(PLAYBACK_MAX_DELTA.values()) + 0.01,
              f"TR first 5 steps delta <= PLAYBACK_MAX_DELTA  max={max_abs:.2f} deg")
        m0_seq = [t[0] for t in targets]
        check(all(m0_seq[i] <= m0_seq[i - 1] + 0.5 for i in range(1, len(m0_seq))),
              f"TR M0 monotonic  targets={[round(x, 1) for x in m0_seq]}")

    # 1.3 - jump_to_step
    if 1 in loaded:
        _, sp = loaded[1]
        sp.reset(current_motor_angles=(0.0, 0.0, 0.0))
        sp.step()
        target_step = sp.total_steps // 2
        sp.jump_to_step(target_step, current_motor_angles=(-50.0, 0.0, 0.0))
        check(sp.current_step == target_step,
              f"jump_to_step -> {target_step}, current_step={sp.current_step}")
        d_after = sp.step()
        check(d_after["progress"] > target_step / sp.total_steps,
              f"resume progress increases  progress={d_after['progress']:.3f}")

    print("\n  [INFO] Speed scale reference:")
    print(f"    Record Hz:   TARGET_HZ       = 20.0")
    print(f"    Main loop Hz: CONTROL_LOOP_HZ = 60.0 (robot state machine)")
    print(f"    Current:     PLAYBACK_SPEED   = {PLAYBACK_SPEED}  -> same as recorded speed")
    print(f"    Suggested initial on-machine: 0.20 (slower, safer)")

    return loaded


# ════════════════════════════════════════════════════════════════
# Layer 2 - JunctionGuide
# ════════════════════════════════════════════════════════════════
def test_layer2_junction_guide():
    from AutoNav.junction_guide import JunctionGuide
    from AutoNav.config import BIFUR_AREA_THRESHOLD, BIFUR_GAIN_M1, BIFUR_GAIN_M2

    section("Layer 2 - JunctionGuide  (bifurcation P-control)")

    jg = JunctionGuide()

    # 2.1 - small area -> no activation
    corr = jg.compute_correction(bifur_cx=0.5, bifur_cy=-0.3, bifur_area=0.001)
    check(abs(corr[0]) < 0.01 and abs(corr[1]) < 0.01,
          f"area<threshold -> correction ~= 0  corr={tuple(round(x, 3) for x in corr)}")

    # 2.2 - direction consistency
    jg.reset()
    corr = jg.compute_correction(bifur_cx=+0.5, bifur_cy=0.0, bifur_area=0.05)
    check(corr[0] > 0, f"bifur on right -> m1 corr > 0  corr_m1={corr[0]:.3f}")

    jg.reset()
    corr = jg.compute_correction(bifur_cx=-0.5, bifur_cy=0.0, bifur_area=0.05)
    check(corr[0] < 0, f"bifur on left  -> m1 corr < 0  corr_m1={corr[0]:.3f}")

    jg.reset()
    corr = jg.compute_correction(bifur_cx=0.0, bifur_cy=+0.3, bifur_area=0.05)
    check(corr[1] > 0, f"bifur below   -> m2 corr > 0  corr_m2={corr[1]:.3f}")

    jg.reset()
    corr = jg.compute_correction(bifur_cx=0.0, bifur_cy=-0.3, bifur_area=0.05)
    check(corr[1] < 0, f"bifur above   -> m2 corr < 0  corr_m2={corr[1]:.3f}")

    # 2.3 - EMA smoothing
    jg.reset()
    vals = []
    for _ in range(10):
        c = jg.compute_correction(bifur_cx=0.5, bifur_cy=0.0, bifur_area=0.05)
        vals.append(c[0])
    diffs = [vals[i + 1] - vals[i] for i in range(len(vals) - 1)]
    check(all(d >= 0 for d in diffs[:5]),
          f"EMA smooth: first 5 steps m1 corr monotonic up  vals={[round(v, 3) for v in vals[:5]]}")
    check(abs(vals[-1] - vals[-2]) < abs(vals[1] - vals[0]),
          f"EMA converges  last delta={abs(vals[-1]-vals[-2]):.4f} < first delta={abs(vals[1]-vals[0]):.4f}")

    # 2.4 - smoothing decays when area vanishes
    jg.reset()
    jg.compute_correction(bifur_cx=0.5, bifur_cy=0.0, bifur_area=0.05)
    last_active = jg._smooth_m1
    for _ in range(10):
        jg.compute_correction(bifur_cx=0.0, bifur_cy=0.0, bifur_area=0.0)
    check(abs(jg._smooth_m1) < abs(last_active) * 0.5,
          f"area vanish -> smooth decay  end={jg._smooth_m1:.4f} < {last_active:.4f} * 0.5")

    print(f"\n  [INFO] Current gains: BIFUR_GAIN_M1={BIFUR_GAIN_M1}, "
          f"BIFUR_GAIN_M2={BIFUR_GAIN_M2}, area threshold={BIFUR_AREA_THRESHOLD}")


# ════════════════════════════════════════════════════════════════
# Layer 3 - ObstructionHandler
# ════════════════════════════════════════════════════════════════
def test_layer3_obstruction_handler():
    from AutoNav.obstruction_handler import ObstructionHandler, HandlerState
    from AutoNav.config import OBSTRUCTION_CLEAR_FRAMES, RETURN_TOLERANCE_DEG

    section("Layer 3 - ObstructionHandler  (state machine & resume)")

    oh = ObstructionHandler()
    check(oh.state == HandlerState.IDLE, "initial state IDLE")
    check(oh.checkpoint is None, "no initial checkpoint")

    # 3.2 - save_checkpoint + start_clearing
    oh.save_checkpoint(motor_angles=(-100.0, 20.0, -50.0), sequence_step=42)
    oh.start_clearing(current_motor_angles=(-100.0, 20.0, -50.0))
    check(oh.state == HandlerState.CLEARING, "after checkpoint -> CLEARING")
    check(oh.checkpoint.sequence_step == 42, f"checkpoint step=42  actual={oh.checkpoint.sequence_step}")
    check(np.allclose(oh.checkpoint.motor_angles, [-100, 20, -50], atol=0.1),
          f"checkpoint angles saved  {oh.checkpoint.motor_angles}")

    # 3.3 - persistent detection -> still CLEARING
    for i in range(5):
        d = oh.update(
            obstruction_detected=True,
            obstruction_cx=0.3,
            obstruction_cy=0.1,
            depth_mean_mm=50.0,
            current_motor_angles=(-110.0 + i, 25.0, -55.0),
        )
        check(d.state == HandlerState.CLEARING,
              f"persistent detection step{i} still CLEARING  d.info='{d.info[:50]}'")

    # 3.4 - vanish N frames -> RETURNING
    cleared = False
    for i in range(OBSTRUCTION_CLEAR_FRAMES + 2):
        d = oh.update(
            obstruction_detected=False,
            obstruction_cx=0.0,
            obstruction_cy=0.0,
            depth_mean_mm=50.0,
            current_motor_angles=(-110.0, 25.0, -55.0),
        )
        if d.state == HandlerState.RETURNING:
            cleared = True
            check(True, f"vanish {OBSTRUCTION_CLEAR_FRAMES} frames -> RETURNING (i={i})")
            break
    check(cleared, "did not switch to RETURNING within expected frames")

    # 3.5 - immediate resume when current pos == checkpoint
    cur = oh.checkpoint.motor_angles.copy()
    d = oh.update(
        obstruction_detected=False, obstruction_cx=0.0, obstruction_cy=0.0,
        depth_mean_mm=50.0, current_motor_angles=tuple(cur),
    )
    check(d.resumed is True, f"at checkpoint -> resumed=True  actual={d.resumed}")
    check(d.state == HandlerState.IDLE, f"at checkpoint -> state=IDLE  actual={d.state}")

    # 3.6 - checkpoint == start position -> first update after clear
    # should immediately mark resumed (path label 0.0 case from a fresh start)
    oh.reset()
    oh.save_checkpoint(motor_angles=(0.0, 0.0, 0.0), sequence_step=10)
    oh.start_clearing(current_motor_angles=(0.0, 0.0, 0.0))
    # Fake 15+ frames of no obstruction so it transitions to RETURNING
    for _ in range(OBSTRUCTION_CLEAR_FRAMES + 1):
        d = oh.update(obstruction_detected=False, obstruction_cx=0.0, obstruction_cy=0.0,
                      depth_mean_mm=50.0, current_motor_angles=(0.0, 0.0, 0.0))
    # The very same call that flipped to RETURNING also should have set resumed=True
    # because the current position equals the checkpoint exactly.
    # (Note: in practice the previous calls may not have set resumed since they were
    # still in CLEARING.)  We assert that the LAST update ended in resumed=True.
    check(d.resumed is True, "after vanish + at-checkpoint -> final update has resumed=True")
    check(d.state == HandlerState.IDLE, "post-return state -> IDLE")


# ════════════════════════════════════════════════════════════════
# Integration - AutoNavController end-to-end
# ════════════════════════════════════════════════════════════════
def test_integration_autonav_controller():
    from AutoNav.nav_controller import AutoNavController, NavMode

    section("Integration - AutoNavController  (end-to-end)")

    # 4.1 - normal navigation (no visual events)
    ctrl = AutoNavController(demo_dir=DEMO_DIR, motor_group=None)
    ctrl.start(path_label=2, current_motor_angles=(0.0, 0.0, 0.0))  # LMB
    check(ctrl.mode == NavMode.NAVIGATING, "after start -> NAVIGATING")

    for i in range(20):
        d = ctrl.step(
            motor_angles=(0.0 + i * 0.5, 0.0, 0.0),
            visual={},
        )
        check(d.active, f"nav step {i}  active=True  mode={d.mode.value}")
        if d.mode == NavMode.DONE:
            break

    # 4.2 - obstruction appears -> CLEARING
    ctrl2 = AutoNavController(demo_dir=DEMO_DIR, motor_group=None)
    ctrl2.start(path_label=2, current_motor_angles=(0.0, 0.0, 0.0))
    cur_angles = (0.0, 0.0, 0.0)
    for i in range(10):
        d = ctrl2.step(motor_angles=cur_angles, visual={})
        cur_angles = (d.target_m0, d.target_m1, d.target_m2)

    d = ctrl2.step(
        motor_angles=cur_angles,
        visual={
            "obstruction_detected": True,
            "obstruction_cx": 0.2, "obstruction_cy": 0.1,
            "depth_mean_mm": 50.0,
        },
    )
    check(d.mode == NavMode.CLEARING,
          f"obstruction detected -> CLEARING  actual={d.mode.value}  info='{d.info}'")
    check(ctrl2._obstruction.checkpoint is not None, "checkpoint saved")
    saved_step = ctrl2._obstruction.checkpoint.sequence_step
    print(f"  [INFO] checkpoint step={saved_step}  angles={tuple(ctrl2._obstruction.checkpoint.motor_angles.round(1))}")

    # 4.3 - simulate 15 frames of no obstruction -> RETURNING
    for i in range(20):
        d = ctrl2.step(
            motor_angles=(d.target_m0, d.target_m1, d.target_m2),
            visual={
                "obstruction_detected": False,
                "obstruction_cx": 0.0, "obstruction_cy": 0.0,
                "depth_mean_mm": 30.0,
            },
        )
        if d.mode == NavMode.RETURNING:
            print(f"  [INFO] step{i}: enter RETURNING  info='{d.info}'")
        if d.mode == NavMode.NAVIGATING:
            check(True, f"cleared -> back to NAVIGATING (step{i})  resume_from={saved_step}")
            break

    # 4.4 - after resume, current_step should be >= saved_step
    if ctrl2._player is not None:
        cur = ctrl2._player.current_step
        check(cur >= saved_step,
              f"after resume current_step={cur} >= saved_step={saved_step}")


# ════════════════════════════════════════════════════════════════
# Demo data summary
# ════════════════════════════════════════════════════════════════
def test_demo_data_summary():
    import json, glob
    from collections import Counter, defaultdict
    from BC.model import BRONCHUS_PATHS

    section("Data summary - expert_demos")

    paths = glob.glob(os.path.join(DEMO_DIR, "*.json"))
    ctr = Counter()
    nframes = defaultdict(int)
    for p in paths:
        try:
            with open(p, "r", encoding="utf-8") as f:
                d = json.load(f)
            yc = d.get("yolo_code", "?")
            n = d.get("n_frames", 0)
            ctr[yc] += 1
            nframes[yc] += n
        except Exception:
            pass

    print(f"\n  Total files: {len(paths)}")
    print(f"  {'path':6s} {'label':6s} {'count':6s} {'frames':8s} {'avg':8s} {'status'}")
    for label, (code, name) in BRONCHUS_PATHS.items():
        if ctr[code] > 0:
            avg = nframes[code] / ctr[code]
            print(f"  {code:6s} {label:<6d} {ctr[code]:<6d} "
                  f"{nframes[code]:<8d} {avg:<8.0f} OK")
        else:
            print(f"  {code:6s} {label:<6d} {'-':<6s} {'-':<8s} {'-':<8s} MISSING")


# ════════════════════════════════════════════════════════════════
# Main
# ════════════════════════════════════════════════════════════════
def main():
    t0 = time.time()
    print("=" * 70)
    print("  AutoNav Offline Unit Self-Test")
    print(f"  Python: {sys.version.split()[0]}")
    print(f"  NumPy:  {np.__version__}")
    print(f"  Demo dir: {DEMO_DIR}")
    print("=" * 70)

    test_demo_data_summary()
    test_layer1_sequence_player()
    test_layer2_junction_guide()
    test_layer3_obstruction_handler()
    test_integration_autonav_controller()

    dt = time.time() - t0
    print("\n" + "=" * 70)
    print(f"  Summary: PASS={_results['pass']}  FAIL={_results['fail']}  "
          f"elapsed={dt:.2f}s")
    if _results["fail"] == 0:
        print("  [ALL PASS] All three layers passed unit tests.")
    else:
        print("  [HAS FAILURE] Please review the FAIL lines above.")
    print("=" * 70)

    return 0 if _results["fail"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
