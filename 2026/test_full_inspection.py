# -*- coding: utf-8 -*-
"""全流程自主巡检离线测试（无需机器人、相机或串口）。"""

import os
import sys
import time

import numpy as np


HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from AutoNav import AutoNavController, NavMode  # noqa: E402
from AutoNav.config import (  # noqa: E402
    BIFUR_MAX_OFFSET_M1,
    BIFUR_MAX_OFFSET_M2,
    BIFUR_STABLE_FRAMES,
    BIFUR_VISUAL_MAX_AGE_S,
    BIFUR_ZERO_EFFECT_DISTANCE,
    OBSTRUCTION_CLEAR_FRAMES,
    OBSTRUCTION_ENTER_AREA_THRESHOLD,
    OBSTRUCTION_TRIGGER_FRAMES,
    PLAYBACK_MAX_DELTA,
)
from AutoNav.inspection_player import (  # noqa: E402
    DEFAULT_INSPECTION_ROUTE,
    InspectionSequencePlayer,
    list_inspection_source_files,
)


DEMO_DIR = os.path.join(HERE, "BC", "expert_demos", "AutoNavdatasets")


def test_precomposed_route() -> None:
    player = InspectionSequencePlayer(DEMO_DIR)
    trajectory = player.get_cumulative_trajectory()

    assert [item[0] for item in player.route] == ["LUB", "LLB", "RUL", "RML", "RLL"]
    assert len(player.source_files) == 5
    assert len(player.transitions) == 4
    assert all(item.from_step > 0 and item.to_step > 0 for item in player.transitions)
    assert all(item.normalized_distance <= 1.5 for item in player.transitions)
    assert any(segment.kind == "backtrack" for segment in player.segments)
    assert any(segment.kind == "bridge" for segment in player.segments)
    assert player.segments[-1].kind == "return_origin"
    player.reset((0.0, 0.0, 0.0))
    first_info = player.get_path_info()
    assert first_info["stage_kind"] == "forward"
    assert first_info["layer2_allowed"] is True
    assert 0.0 <= first_info["stage_progress"] <= 1.0
    assert np.allclose(trajectory[0], (0.0, 0.0, 0.0), atol=1e-5)
    assert np.allclose(trajectory[-1], (0.0, 0.0, 0.0), atol=1e-5)

    # 自动选择可以随现场新录制而变化；只校验所选文件仍属于对应叶段。
    for (code, _label, _name), source_file in zip(player.route, player.source_files):
        assert f"_{code}_" in os.path.basename(source_file)

    # 7 月 15 日的特殊 RLL 长记录最终回到原点，不应作为叶段轨迹入选。
    assert "20260715_171322_RLL_1567frames.json" not in player.source_files[-1]


def test_manual_source_selection() -> None:
    selected = {}
    for code, label, _name in DEFAULT_INSPECTION_ROUTE:
        candidates = list_inspection_source_files(DEMO_DIR, code, label)
        assert candidates, f"{code} should have at least one selectable trajectory"
        selected[code] = candidates[0]

    player = InspectionSequencePlayer(DEMO_DIR, source_files=selected)
    assert player.source_files == [selected[item[0]] for item in player.route]

    invalid = dict(selected)
    invalid["RLL"] = os.path.join(
        DEMO_DIR, "20260715_171322_RLL_1567frames.json"
    )
    try:
        InspectionSequencePlayer(DEMO_DIR, source_files=invalid)
    except ValueError as exc:
        assert "RLL" in str(exc) and "前进行程不足" in str(exc)
    else:
        raise AssertionError("closed-loop RLL recording must be rejected")


def test_controller_runs_layer1_only_and_returns_origin() -> None:
    controller = AutoNavController(
        demo_dir=DEMO_DIR,
        motor_group=None,
        enable_junction=False,
        enable_obstruction=False,
    )
    controller.start_inspection((0.0, 0.0, 0.0))
    assert controller.mode == NavMode.NAVIGATING
    assert controller.enable_junction is False
    assert controller.enable_obstruction is False

    controller.pause()
    paused = controller.step((0.0, 0.0, 0.0), {})
    assert paused.mode == NavMode.PAUSED and paused.active is False
    controller.resume()
    assert controller.mode == NavMode.NAVIGATING

    current = (0.0, 0.0, 0.0)
    stages = []
    last = None
    for _ in range(10000):
        last = controller.step(
            motor_angles=current,
            visual={
                "obstruction_detected": True,
                "bifur_cx": 0.2,
                "bifur_cy": -0.2,
                "bifur_area": 0.5,
            },
        )
        if last.route_stage and (not stages or stages[-1] != last.route_stage):
            stages.append(last.route_stage)
        current = (last.target_m0, last.target_m1, last.target_m2)
        assert last.mode not in (NavMode.CLEARING, NavMode.RETURNING)
        assert last.junction_corr_m1 == 0.0
        assert last.junction_corr_m2 == 0.0
        assert abs(last.delta_m0) <= PLAYBACK_MAX_DELTA[0] + 1e-4
        assert abs(last.delta_m1) <= PLAYBACK_MAX_DELTA[1] + 1e-4
        assert abs(last.delta_m2) <= PLAYBACK_MAX_DELTA[2] + 1e-4
        if last.mode == NavMode.DONE:
            break

    assert last is not None and last.mode == NavMode.DONE
    assert np.allclose(current, (0.0, 0.0, 0.0), atol=1e-3)
    assert any("前往左上" in stage for stage in stages)
    assert any("前往右下" in stage for stage in stages)
    assert any("返回原点" in stage for stage in stages)


def test_full_inspection_layer2_is_conservative_and_stage_gated() -> None:
    controller = AutoNavController(
        demo_dir=DEMO_DIR,
        motor_group=None,
        enable_junction=True,
        enable_obstruction=False,
    )
    controller.start_inspection((0.0, 0.0, 0.0))
    assert controller.enable_junction is True

    current = (0.0, 0.0, 0.0)
    decision = None
    # 近中心候选必须连续稳定若干帧后才允许产生微调。
    for index in range(BIFUR_STABLE_FRAMES):
        decision = controller.step(
            motor_angles=current,
            visual={
                "bifur_detected": True,
                "bifur_cx": 0.15,
                "bifur_cy": -0.10,
                "bifur_area": 0.05,
                "visual_timestamp": time.monotonic(),
            },
        )
        current = (decision.target_m0, decision.target_m1, decision.target_m2)
        if index < BIFUR_STABLE_FRAMES - 1:
            assert decision.junction_active is False
            assert decision.junction_gate_reason.startswith("stabilizing:")

    assert decision is not None and decision.junction_active is True
    assert decision.route_stage_kind == "forward"
    assert 0.0 < decision.junction_strength <= 1.0
    assert 0.0 < decision.junction_corr_m1 <= BIFUR_MAX_OFFSET_M1
    assert abs(decision.junction_corr_m2) <= BIFUR_MAX_OFFSET_M2

    # 中心过远时立即回归纯 Layer 1，不尝试追赶。
    far = controller.step(
        motor_angles=current,
        visual={
            "bifur_detected": True,
            "bifur_cx": BIFUR_ZERO_EFFECT_DISTANCE + 0.05,
            "bifur_cy": 0.0,
            "bifur_area": 0.05,
            "visual_timestamp": time.monotonic(),
        },
    )
    current = (far.target_m0, far.target_m1, far.target_m2)
    assert far.junction_active is False
    assert far.junction_gate_reason == "far_from_center"
    assert far.junction_corr_m1 == 0.0
    assert far.junction_corr_m2 == 0.0

    # 视觉线程停更时也不能接受新的修正。
    stale = controller.step(
        motor_angles=current,
        visual={
            "bifur_detected": True,
            "bifur_cx": 0.15,
            "bifur_cy": 0.0,
            "bifur_area": 0.05,
            "visual_timestamp": time.monotonic() - BIFUR_VISUAL_MAX_AGE_S - 0.1,
        },
    )
    current = (stale.target_m0, stale.target_m1, stale.target_m2)
    assert stale.junction_active is False
    assert stale.junction_gate_reason == "visual_stale"
    assert stale.junction_corr_m1 == 0.0
    assert stale.junction_corr_m2 == 0.0

    missing = controller.step(
        motor_angles=current,
        visual={
            "bifur_detected": False,
            "bifur_cx": 0.0,
            "bifur_cy": 0.0,
            "bifur_area": 0.0,
            "visual_timestamp": time.monotonic(),
        },
    )
    current = (missing.target_m0, missing.target_m1, missing.target_m2)
    assert missing.junction_gate_reason == "not_detected"
    assert missing.junction_corr_m1 == 0.0
    assert missing.junction_corr_m2 == 0.0

    # 推进到首个回退段；即使视觉候选稳定且靠近中心，也必须关闭 Layer 2。
    for _ in range(2500):
        decision = controller.step(
            motor_angles=current,
            visual={
                "bifur_detected": True,
                "bifur_cx": 0.15,
                "bifur_cy": -0.10,
                "bifur_area": 0.05,
                "visual_timestamp": time.monotonic(),
            },
        )
        current = (decision.target_m0, decision.target_m1, decision.target_m2)
        if decision.route_stage_kind == "backtrack":
            break

    assert decision is not None and decision.route_stage_kind == "backtrack"
    assert decision.junction_active is False
    assert decision.junction_gate_reason == "stage:backtrack"
    assert decision.junction_corr_m1 == 0.0
    assert decision.junction_corr_m2 == 0.0


def test_full_inspection_layer3_interrupts_returns_and_resumes() -> None:
    controller = AutoNavController(
        demo_dir=DEMO_DIR,
        motor_group=None,
        enable_junction=True,
        enable_obstruction=True,
    )
    controller.start_inspection((0.0, 0.0, 0.0))
    assert controller.enable_obstruction is True

    current = (0.0, 0.0, 0.0)
    timestamp = time.monotonic()

    # 小目标不能抢占 Layer 1。
    for index in range(OBSTRUCTION_TRIGGER_FRAMES + 2):
        timestamp += 0.001
        decision = controller.step(
            current,
            {
                "obstruction_detected": True,
                "obstruction_cx": 0.05,
                "obstruction_cy": -0.04,
                "obstruction_area": OBSTRUCTION_ENTER_AREA_THRESHOLD - 0.01,
                "obstruction_depth_mm": 40.0,
                "visual_timestamp": timestamp,
            },
        )
        current = (decision.target_m0, decision.target_m1, decision.target_m2)
        assert decision.mode == NavMode.NAVIGATING
        assert decision.obstruction_gate_reason.startswith("area_below:")

    # 控制循环会以 60 Hz 重复读取同一视觉帧，同一时间戳只能计数一次。
    timestamp += 0.001
    large_obstruction = {
        "obstruction_detected": True,
        "obstruction_cx": 0.08,
        "obstruction_cy": -0.06,
        "obstruction_area": OBSTRUCTION_ENTER_AREA_THRESHOLD + 0.08,
        "obstruction_depth_mm": 40.0,
        "visual_timestamp": timestamp,
    }
    decision = controller.step(current, large_obstruction)
    current = (decision.target_m0, decision.target_m1, decision.target_m2)
    assert decision.mode == NavMode.NAVIGATING
    assert decision.obstruction_stable_frames == 1
    for _ in range(8):
        decision = controller.step(current, large_obstruction)
        current = (decision.target_m0, decision.target_m1, decision.target_m2)
        assert decision.mode == NavMode.NAVIGATING
        assert decision.obstruction_stable_frames == 1
        assert decision.obstruction_gate_reason == "waiting_new_frame"

    # 连续足够多的新视觉帧后进入清除，并冻结路径步号。
    for _ in range(OBSTRUCTION_TRIGGER_FRAMES - 1):
        timestamp += 0.001
        large_obstruction["visual_timestamp"] = timestamp
        decision = controller.step(current, dict(large_obstruction))
        current = (decision.target_m0, decision.target_m1, decision.target_m2)
    assert decision.mode == NavMode.CLEARING
    checkpoint_step = decision.checkpoint_step
    assert checkpoint_step >= 0
    assert decision.route_stage_kind == "forward"
    assert controller._player.current_step == checkpoint_step

    timestamp += 0.001
    large_obstruction["visual_timestamp"] = timestamp
    tracking = controller.step(current, dict(large_obstruction))
    current = (tracking.target_m0, tracking.target_m1, tracking.target_m2)
    assert tracking.mode == NavMode.CLEARING
    assert controller._player.current_step == checkpoint_step

    # 重复/陈旧帧不能把“已清除”计数刷满。
    no_obstruction = {
        "obstruction_detected": False,
        "obstruction_cx": 0.0,
        "obstruction_cy": 0.0,
        "obstruction_area": 0.0,
        "obstruction_depth_mm": 40.0,
        "visual_timestamp": timestamp,
    }
    for _ in range(OBSTRUCTION_CLEAR_FRAMES + 3):
        held = controller.step(current, no_obstruction)
        current = (held.target_m0, held.target_m1, held.target_m2)
        assert held.mode == NavMode.CLEARING
        assert held.obstruction_clear_frames == 0
        assert controller._player.current_step == checkpoint_step

    # 连续新帧确认目标消失后，返回断点。
    for index in range(OBSTRUCTION_CLEAR_FRAMES):
        timestamp += 0.001
        no_obstruction["visual_timestamp"] = timestamp
        decision = controller.step(current, dict(no_obstruction))
        current = (decision.target_m0, decision.target_m1, decision.target_m2)
        if index < OBSTRUCTION_CLEAR_FRAMES - 1:
            assert decision.mode == NavMode.CLEARING
        assert controller._player.current_step == checkpoint_step
    assert decision.mode == NavMode.RETURNING

    # 模拟电机跟随返回目标；到位需连续稳定若干帧才恢复。
    for _ in range(200):
        decision = controller.step(current, dict(no_obstruction))
        current = (decision.target_m0, decision.target_m1, decision.target_m2)
        if decision.mode == NavMode.NAVIGATING:
            break
    assert decision.mode == NavMode.NAVIGATING
    assert decision.checkpoint_step == checkpoint_step
    assert controller._player.current_step == checkpoint_step
    assert decision.obstruction_gate_reason == "resumed"

    # 续航后的下一步从原断点继续，而不是重新从轨迹起点播放。
    timestamp += 0.001
    no_obstruction["visual_timestamp"] = timestamp
    resumed = controller.step(current, dict(no_obstruction))
    assert resumed.mode == NavMode.NAVIGATING
    assert resumed.step == checkpoint_step + 1
    assert abs(resumed.delta_m0) <= PLAYBACK_MAX_DELTA[0] + 1e-4
    assert abs(resumed.delta_m1) <= PLAYBACK_MAX_DELTA[1] + 1e-4
    assert abs(resumed.delta_m2) <= PLAYBACK_MAX_DELTA[2] + 1e-4


def test_full_inspection_layer3_is_disabled_during_backtrack() -> None:
    controller = AutoNavController(
        demo_dir=DEMO_DIR,
        motor_group=None,
        enable_junction=False,
        enable_obstruction=True,
    )
    controller.start_inspection((0.0, 0.0, 0.0))
    current = (0.0, 0.0, 0.0)
    timestamp = time.monotonic()
    decision = None

    for _ in range(2500):
        timestamp += 0.001
        decision = controller.step(
            current,
            {
                "obstruction_detected": False,
                "obstruction_area": 0.0,
                "visual_timestamp": timestamp,
            },
        )
        current = (decision.target_m0, decision.target_m1, decision.target_m2)
        if decision.route_stage_kind == "backtrack":
            break
    assert decision is not None and decision.route_stage_kind == "backtrack"

    for _ in range(OBSTRUCTION_TRIGGER_FRAMES + 2):
        timestamp += 0.001
        decision = controller.step(
            current,
            {
                "obstruction_detected": True,
                "obstruction_cx": 0.0,
                "obstruction_cy": 0.0,
                "obstruction_area": OBSTRUCTION_ENTER_AREA_THRESHOLD + 0.2,
                "obstruction_depth_mm": 40.0,
                "visual_timestamp": timestamp,
            },
        )
        current = (decision.target_m0, decision.target_m1, decision.target_m2)
        assert decision.mode == NavMode.NAVIGATING
        assert decision.obstruction_gate_reason == "stage:backtrack"


def test_existing_single_route_still_works() -> None:
    lub_file = os.path.join(DEMO_DIR, "20260710_153920_LUB_504frames.json")
    controller = AutoNavController(demo_dir=DEMO_DIR, motor_group=None)
    controller.start(
        path_label=4,
        current_motor_angles=(0.0, 0.0, 0.0),
        demo_file=lub_file,
    )
    decision = controller.step((0.0, 0.0, 0.0), {})
    assert decision.mode == NavMode.NAVIGATING
    assert decision.route_stage == ""
    assert decision.total_steps > 0


if __name__ == "__main__":
    test_precomposed_route()
    test_manual_source_selection()
    test_controller_runs_layer1_only_and_returns_origin()
    test_full_inspection_layer2_is_conservative_and_stage_gated()
    test_full_inspection_layer3_interrupts_returns_and_resumes()
    test_full_inspection_layer3_is_disabled_during_backtrack()
    test_existing_single_route_still_works()
    print("[PASS] 全流程自主巡检离线测试全部通过")
