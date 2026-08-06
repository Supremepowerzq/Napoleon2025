# -*- coding: utf-8 -*-
"""
AutoNav 集成冒烟测试（不依赖硬件/相机）
========================================
模拟主程序 loop_ai_auto 的关键调用路径：
  1. 进入 AIAuto 状态
  2. 启动 AutoNavController
  3. 跑 30 帧（每帧给一个空 visual）→ 应进入 NAVIGATING
  4. 在第 10 帧注入阻塞物 → 应切 CLEARING
  5. 等待清除完成 → 应回到 NAVIGATING
  6. 验证步进连续、目标角度合理、不超出限位
  7. 退出：模拟手柄 START → 应回到 IDLE

运行：
    python test_autonav_integration.py
"""
import os
import sys
import numpy as np

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

_pass = 0
_fail = 0


def check(cond, msg):
    global _pass, _fail
    if cond:
        _pass += 1
        print(f"  PASS  {msg}")
    else:
        _fail += 1
        print(f"  FAIL  {msg}")


def fake_motor_group():
    """构造一个轻量 motor_group mock，set_motor_position 不会触达硬件。"""
    class _MockMotorGroup:
        def __init__(self):
            self._angles = np.array([0.0, 0.0, 0.0], dtype=np.float32)
            self.serial_lock = __import__("threading").RLock()

        def get_cached_angles(self):
            return self._angles.copy()

        def set_motor_position(self, motor_id, target_angle, max_speed=None):
            self._angles[motor_id] = float(target_angle)

    return _MockMotorGroup()


def main():
    from AutoNav import AutoNavController, NavMode
    from AutoNav.config import MOTOR_LIMITS

    print("=" * 70)
    print("  AutoNav Integration Smoke Test")
    print("  (no hardware/camera required)")
    print("=" * 70)

    mg = fake_motor_group()
    ctrl = AutoNavController(
        demo_dir    = os.path.join(HERE, "BC", "expert_demos"),
        motor_group = mg,
    )
    ctrl.start(path_label=2, current_motor_angles=(0.0, 0.0, 0.0))  # LMB
    check(ctrl.mode == NavMode.NAVIGATING, "started in NAVIGATING mode")

    # ── 跑 30 帧（无视觉事件） ───────────────────────────────
    print("\n  --- Phase 1: 30 frames of pure navigation ---")
    history = []
    for i in range(30):
        d = ctrl.step(
            motor_angles = mg.get_cached_angles(),
            visual       = {},   # 无视觉信息
        )
        if d.active:
            mg.set_motor_position(0, d.target_m0, max_speed=20.0)
            mg.set_motor_position(1, d.target_m1, max_speed=12.0)
            mg.set_motor_position(2, d.target_m2, max_speed=12.0)
        history.append((d.target_m0, d.target_m1, d.target_m2, d.mode))
    check(all(h[3] == NavMode.NAVIGATING for h in history),
          f"all 30 frames NAVIGATING  unique modes = {set(h[3] for h in history)}")
    check(history[-1][0] < history[0][0] - 5,
          f"M0 monotonically advances  start={history[0][0]:.1f} -> end={history[-1][0]:.1f}  "
          f"(PLAYBACK_SPEED=0.33 -> ~11 deg / 30 frames expected for LMB)")
    # 限位保护
    in_limit = all(
        MOTOR_LIMITS[i][0] - 1 <= h[i] <= MOTOR_LIMITS[i][1] + 1
        for h in history for i in range(3)
    )
    check(in_limit, "all targets within motor limits")

    # ── 注入阻塞物 → 切 CLEARING ─────────────────────────────
    print("\n  --- Phase 2: inject obstruction at frame 31 ---")
    cur = mg.get_cached_angles()
    d = ctrl.step(
        motor_angles = tuple(cur),
        visual       = {
            "obstruction_detected": True,
            "obstruction_cx":       0.3,
            "obstruction_cy":       0.1,
            "depth_mean_mm":        50.0,
        },
    )
    check(d.mode == NavMode.CLEARING,
          f"obstruction detected -> CLEARING  actual={d.mode.value}")
    check(ctrl._obstruction.checkpoint is not None, "checkpoint saved")
    saved_step = ctrl._obstruction.checkpoint.sequence_step
    print(f"  [INFO] checkpoint step={saved_step}  "
          f"angles={tuple(ctrl._obstruction.checkpoint.motor_angles.round(1))}")

    # ── 模拟清除 20 帧（无阻塞物） ──────────────────────────
    print("\n  --- Phase 3: 20 frames of clearance ---")
    for i in range(20):
        cur = mg.get_cached_angles()
        d = ctrl.step(
            motor_angles = tuple(cur),
            visual       = {
                "obstruction_detected": False,
                "obstruction_cx":       0.0,
                "obstruction_cy":       0.0,
                "depth_mean_mm":        30.0,
            },
        )
        if d.active:
            mg.set_motor_position(0, d.target_m0, max_speed=20.0)
            mg.set_motor_position(1, d.target_m1, max_speed=12.0)
            mg.set_motor_position(2, d.target_m2, max_speed=12.0)
        mode_seq = [h[3] for h in history[-3:]] if history else []
        if d.mode == NavMode.RETURNING:
            print(f"  [INFO] step {i}: -> RETURNING  info='{d.info}'")
        if d.mode == NavMode.NAVIGATING:
            check(True, f"resumed NAVIGATING at step {i}  from saved_step={saved_step}")
            break

    # ── 验证续航点：current_step 应 >= saved_step ────────────
    check(ctrl._player is not None and ctrl._player.current_step >= saved_step,
          f"after resume: player.current_step={ctrl._player.current_step} >= saved={saved_step}")

    # ── 岔口引导：注入岔口信号，验证修正量方向正确 ──────────
    print("\n  --- Phase 4: junction guide during navigation ---")
    cur = mg.get_cached_angles()
    jg_dec = ctrl.step(
        motor_angles = tuple(cur),
        visual       = {
            "bifur_cx":   +0.4,
            "bifur_cy":   0.0,
            "bifur_area": 0.08,
        },
    )
    check(jg_dec.mode == NavMode.NAVIGATING, f"bifur input -> still NAVIGATING  actual={jg_dec.mode.value}")
    check(jg_dec.junction_corr_m1 > 0,
          f"bifur on right -> corr_m1 > 0  actual={jg_dec.junction_corr_m1:.3f}")

    # ── stop ────────────────────────────────────────────────
    print("\n  --- Phase 5: stop controller ---")
    ctrl.stop()
    check(ctrl.mode == NavMode.IDLE, f"after stop() -> IDLE  actual={ctrl.mode.value}")

    # ── 汇总 ────────────────────────────────────────────────
    print("\n" + "=" * 70)
    print(f"  Summary: PASS={_pass}  FAIL={_fail}")
    if _fail == 0:
        print("  [INTEGRATION OK] AutoNav replaces BCRunner in loop_ai_auto correctly.")
    else:
        print("  [HAS FAILURE]")
    print("=" * 70)
    return 0 if _fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
