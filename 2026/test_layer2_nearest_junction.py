# -*- coding: utf-8 -*-
"""Layer 2 最近岔口选择与有限偏置测试（不依赖相机和电机）。"""

import os
import sys

import numpy as np


HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, PROJECT_ROOT)


def test_nearest_selection():
    from AutoNav.junction_guide import select_nearest_junction

    candidates = [
        {"cx": 20, "cy": 20, "area": 9000},
        {"cx": 105, "cy": 96, "area": 500},
        {"cx": 160, "cy": 110, "area": 3000},
    ]
    selected, index = select_nearest_junction(candidates, center_x=100, center_y=100)
    assert index == 1
    assert selected is candidates[1]


def test_deadzone_and_direction():
    from AutoNav.junction_guide import JunctionGuide, pixel_axis_tracking_speed

    guide = JunctionGuide(response_alpha=1.0)
    assert guide.compute_correction(0.04, -0.04, 0.05) == (0.0, 0.0)
    right, down = guide.compute_correction(0.5, 0.5, 0.05)
    assert right > 0.0
    assert down > 0.0

    # 独立图控闭环：死区内停止、近处慢、远处快，且方向指向目标。
    assert pixel_axis_tracking_speed(5, 10, 45, 5, 12) == 0.0
    x_near = pixel_axis_tracking_speed(11, 10, 45, 5, 12)
    x_far = pixel_axis_tracking_speed(45, 10, 45, 5, 12)
    assert x_near < 0.0 and x_far < 0.0
    assert abs(x_far) > abs(x_near)
    assert abs(x_near) > 2.0  # 超过主程序M1启动阈值
    assert pixel_axis_tracking_speed(-45, 10, 45, 5, 12) > 0.0

    y_near = pixel_axis_tracking_speed(11, 10, 80, 0.05, 8, error_to_speed_sign=1.0)
    y_far = pixel_axis_tracking_speed(80, 10, 80, 0.05, 8, error_to_speed_sign=1.0)
    assert y_near > 0.0 and y_far > 0.0
    assert abs(y_far) > abs(y_near)
    assert abs(y_near * 4.0) > 0.5  # 缩放后仍超过主程序M2精调阈值


class _FakePlayer:
    def __init__(self, steps=400, dm0=-0.2, dm1=0.1, dm2=-0.05):
        self.current_step = 0
        self.total_steps = steps
        self.dm0 = dm0
        self.dm1 = dm1
        self.dm2 = dm2

    def step(self):
        self.current_step += 1
        return {
            "done": self.current_step > self.total_steps,
            "delta_m0": self.dm0,
            "delta_m1": self.dm1,
            "delta_m2": self.dm2,
            "progress": min(self.current_step / self.total_steps, 1.0),
        }


def test_controller_offset_does_not_accumulate():
    from AutoNav.config import (
        BIFUR_FULL_EFFECT_DISTANCE,
        BIFUR_FULL_EFFECT_PROGRESS,
        BIFUR_MAX_OFFSET_M1,
        BIFUR_MAX_OFFSET_M2,
        BIFUR_ZERO_EFFECT_DISTANCE,
    )
    from AutoNav.nav_controller import AutoNavController, NavMode

    controller = AutoNavController(
        demo_dir="unused",
        enable_junction=True,
        enable_obstruction=False,
    )
    controller._player = _FakePlayer()
    controller._mode = NavMode.NAVIGATING
    controller._base_target = np.zeros(3, dtype=np.float32)
    controller._target = np.zeros(3, dtype=np.float32)

    # 路径前段：允许小幅视觉微调，但强度低于路径后段。
    decision = None
    for _ in range(100):
        decision = controller.step(
            motor_angles=(0.0, 0.0, 0.0),
            visual={
                "bifur_cx": 0.2,
                "bifur_cy": -0.2,
                "bifur_area": 0.1,
                # 当前阶段 Layer 3 关闭，即使视觉端识别到阻塞物也不能打断导航。
                "obstruction_detected": True,
            },
        )

    assert decision.progress < BIFUR_FULL_EFFECT_PROGRESS
    assert decision.junction_active is True
    assert decision.junction_corr_m1 > 0.0
    early_strength = decision.junction_strength
    early_offset = abs(decision.junction_corr_m1)

    # 推进到路径后段；极远目标超过软限制时才完全停止。
    for _ in range(230):
        decision = controller.step(
            motor_angles=(0.0, 0.0, 0.0),
            visual={
                "bifur_cx": BIFUR_ZERO_EFFECT_DISTANCE + 0.1,
                "bifur_cy": 0.0,
                "bifur_area": 0.1,
            },
        )
    assert decision.progress >= BIFUR_FULL_EFFECT_PROGRESS
    assert decision.junction_active is False
    assert decision.junction_corr_m1 == 0.0
    assert decision.junction_corr_m2 == 0.0

    # 路径后段、中心距离中等：不应完全静止，而应以衰减强度微调。
    middle_distance = (
        BIFUR_FULL_EFFECT_DISTANCE + BIFUR_ZERO_EFFECT_DISTANCE
    ) / 2.0
    for _ in range(20):
        decision = controller.step(
            motor_angles=(0.0, 0.0, 0.0),
            visual={
                "bifur_cx": middle_distance,
                "bifur_cy": 0.0,
                "bifur_area": 0.1,
            },
        )
    assert decision.junction_active is True
    assert 0.0 < decision.junction_strength < 1.0
    assert decision.junction_corr_m1 > 0.0

    # 路径后段且岔口中心较近：M1/M2达到更高但仍有限的偏置。
    for _ in range(50):
        decision = controller.step(
            motor_angles=(0.0, 0.0, 0.0),
            visual={"bifur_cx": 0.2, "bifur_cy": -0.2, "bifur_area": 0.1},
        )

    assert decision is not None
    # 原路径继续前进，但视觉作用始终只是相对基础路径的有限偏置。
    assert abs(decision.target_m1 - controller._base_target[1]) <= BIFUR_MAX_OFFSET_M1 + 1e-5
    assert abs(decision.target_m2 - controller._base_target[2]) <= BIFUR_MAX_OFFSET_M2 + 1e-5
    assert abs(decision.junction_corr_m1) <= BIFUR_MAX_OFFSET_M1 + 1e-5
    assert abs(decision.junction_corr_m2) <= BIFUR_MAX_OFFSET_M2 + 1e-5
    assert decision.junction_active is True
    assert decision.junction_strength > early_strength
    assert abs(decision.junction_corr_m1) > early_offset
    assert decision.target_m0 == controller._base_target[0]
    assert decision.mode == NavMode.NAVIGATING
    assert decision.obstruction_detected is False


def test_visual_mode_layout_isolated_in_715():
    path = os.path.join(HERE, "predict_2026_715_zck.py")
    with open(path, "r", encoding="utf-8") as stream:
        source = stream.read()
    assert 'mode_names = {1: "Mode-1: Cavity Tracking"' in source
    assert '2: "Mode-2: Obstruction Tracking"' in source
    assert "if c in (ord('h'), ord('H'))" not in source
    assert "color_cav = (255, 255, 0)" in source
    assert "cav_vx * _cav_distance_weight" not in source
    assert "cav_vy * _cav_distance_weight" not in source
    assert "cv2.setWindowTitle(" not in source
    assert "requested_track_mode = get_tracking_mode()" in source
    assert "set_tracking_mode(self._track_mode)" in source

    main_path = os.path.join(HERE, "main2026-Xhandwriting-AC-AutoNav(0710)-Bronchus.py")
    with open(main_path, "r", encoding="utf-8") as stream:
        main_source = stream.read()
    assert "VISION_M1_STOP_INPUT = 2.0" in main_source
    assert "HORIZONTAL_CLOCKWISE_SIGN = 1.0" in main_source
    assert "VISION_M2_SPD_SCALE = 2.0" in main_source
    assert "VISION_M2_STOP_SPEED = 0.5" in main_source
    assert 'text="自动模式", width=100, height=40, bg="#81D4FA"' in main_source
    assert 'text="AI巡检"' in main_source
    assert 'command=lambda: self._on_command("ai_auto")' in main_source
    assert 'text="图像识别模式"' in main_source
    assert '(1, "1 岔口"' in main_source
    assert '(2, "2 阻塞物"' in main_source
    assert '(3, "3 混合"' in main_source


def test_shared_tracking_mode_state():
    from config import get_tracking_mode, set_tracking_mode

    previous = get_tracking_mode()
    try:
        assert set_tracking_mode(2) is True
        assert get_tracking_mode() == 2
        assert set_tracking_mode("3") is True
        assert get_tracking_mode() == 3
        assert set_tracking_mode(4) is False
        assert get_tracking_mode() == 3
    finally:
        set_tracking_mode(previous)


if __name__ == "__main__":
    test_nearest_selection()
    test_deadzone_and_direction()
    test_controller_offset_does_not_accumulate()
    test_visual_mode_layout_isolated_in_715()
    test_shared_tracking_mode_state()
    print("[PASS] Layer 2 nearest-junction bounded-offset tests")
