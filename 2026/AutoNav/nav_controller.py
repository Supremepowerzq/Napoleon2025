# -*- coding: utf-8 -*-
"""
AutoNavController — 自主导航总控制器
=====================================
集成三层架构：SequencePlayer + JunctionGuide + ObstructionHandler

状态机
------
IDLE        未启动
NAVIGATING  序列导航 + 岔口修正（正常导航）
CLEARING    阻塞物追踪清除（优先级最高）
RETURNING   阻塞物清除完毕，返回导航断点
DONE        当前路径序列播放完毕（已到达目标部位）

主程序集成方式（在主循环中）
----------------------------
    from AutoNav import AutoNavController
    ctrl = AutoNavController(demo_dir="BC/expert_demos", motor_group=mg)
    ctrl.start(path_label=2, current_motor_angles=mg.get_cached_angles())

    # 每帧：
    vis = get_visual_features()   # 从视觉线程读取
    dec = ctrl.step(
        motor_angles = mg.get_cached_angles(),
        visual       = vis,
    )
    if dec.active:
        mg.set_motor_position(0, dec.target_m0, max_speed=25)
        mg.set_motor_position(1, dec.target_m1, max_speed=15)
        mg.set_motor_position(2, dec.target_m2, max_speed=15)
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple, Dict, Any
import time
import numpy as np

from .config import (
    BIFUR_AREA_THRESHOLD,
    BIFUR_MAX_TRACK_JUMP,
    BIFUR_STABLE_FRAMES,
    BIFUR_VISUAL_MAX_AGE_S,
    CONTROL_LOOP_HZ,
    MOTOR_LIMITS,
    NAV_SETTLE_TIMEOUT_S,
    NAV_SETTLE_TOLERANCE,
    OBSTRUCTION_ENTER_AREA_THRESHOLD,
    OBSTRUCTION_TRIGGER_FRAMES,
    OBSTRUCTION_VISUAL_MAX_AGE_S,
)
from .sequence_player import SequencePlayer
from .inspection_player import InspectionSequencePlayer
from .junction_guide import (
    JunctionGuide,
    junction_distance_weight,
    junction_progress_weight,
)
from .obstruction_handler import ObstructionHandler, HandlerState


class NavMode(Enum):
    IDLE       = "idle"
    PAUSED     = "paused"
    NAVIGATING = "navigating"
    SETTLING   = "settling"
    CLEARING   = "clearing"
    RETURNING  = "returning"
    DONE       = "done"


@dataclass
class NavDecision:
    """AutoNavController 每帧的决策输出。"""
    mode:      NavMode

    # 电机目标绝对角度（°）
    target_m0: float = 0.0
    target_m1: float = 0.0
    target_m2: float = 0.0

    # 本帧增量（调试用）
    delta_m0:  float = 0.0
    delta_m1:  float = 0.0
    delta_m2:  float = 0.0

    # 导航进度
    progress:  float = 0.0    # 0.0 ~ 1.0
    step:      int   = 0
    total_steps: int = 0

    # 岔口修正量（调试）
    junction_corr_m1: float = 0.0
    junction_corr_m2: float = 0.0
    junction_active: bool = False
    junction_center_distance: float = 0.0
    junction_strength: float = 0.0
    junction_gate_reason: str = ""
    junction_stable_frames: int = 0
    visual_age_s: float = 0.0

    # 阻塞物状态
    obstruction_detected: bool = False
    obstruction_area: float = 0.0
    obstruction_gate_reason: str = ""
    obstruction_stable_frames: int = 0
    obstruction_clear_frames: int = 0
    checkpoint_step: int = -1

    # 是否有有效指令（IDLE/DONE 时为 False）
    active: bool = True

    info: str = ""
    route_stage: str = ""
    route_stage_kind: str = ""
    route_stage_progress: float = 0.0


class AutoNavController:
    """
    支气管自主巡检总控制器。

    Parameters
    ----------
    demo_dir    : str   BC 专家演示目录（包含 *.json 文件）
    motor_group : 可选，MotorGroup2025 实例（用于内部限位，若为 None 则不限位）
    """

    def __init__(
        self,
        demo_dir: str,
        motor_group=None,
        enable_junction: bool = False,
        enable_obstruction: bool = False,
    ):
        self.demo_dir    = demo_dir
        self.motor_group = motor_group
        # 分层验证默认仅启用 Layer 1；Layer 2/3 必须由 UI 明确开启。
        self.enable_junction = enable_junction
        self.enable_obstruction = enable_obstruction

        self._mode:   NavMode = NavMode.IDLE
        self._player: Optional[SequencePlayer] = None
        self._junction = JunctionGuide()
        self._obstruction = ObstructionHandler()

        # Layer 1 基础目标与最终控制目标分离。Layer 2 只能在基础目标上
        # 施加有限偏置，不能逐帧累积并覆盖原始采集路径。
        self._base_target: Optional[np.ndarray] = None
        self._target: Optional[np.ndarray] = None

        self._step_count: int = 0
        self._settle_count: int = 0
        self._junction_valid_frames: int = 0
        self._last_bifur_center: Optional[Tuple[float, float]] = None
        self._last_route_stage: Optional[Tuple[str, str]] = None
        self._obstruction_candidate_frames: int = 0
        self._last_obstruction_visual_timestamp: Optional[float] = None
        self._last_obstruction_stage: Optional[Tuple[str, str]] = None

    # ── 生命周期 ──────────────────────────────────────────────────

    def start(
        self,
        path_label: int,
        current_motor_angles: Tuple[float, float, float],
        demo_file: Optional[str] = None,
    ):
        """
        开始自主导航。

        Parameters
        ----------
        path_label            : 目标支气管标签（见 BC/model.py BRONCHUS_PATHS）
        current_motor_angles  : 当前电机角度 (m0, m1, m2)，作为序列起点
        """
        try:
            self._player = SequencePlayer(
                self.demo_dir, path_label, demo_file=demo_file
            )
        except ValueError as e:
            print(f"[AutoNav] 无法加载序列: {e}")
            return

        self._player.reset(current_motor_angles)
        self._junction.reset()
        self._reset_junction_gate()
        self._obstruction.reset()
        self._reset_obstruction_gate()

        self._base_target = np.array(current_motor_angles, dtype=np.float32)
        self._target = self._base_target.copy()
        self._mode = NavMode.NAVIGATING
        self._step_count = 0
        self._settle_count = 0

        print(f"[AutoNav] 启动 path_label={path_label}  "
              f"总步数={self._player.total_steps}")

    def start_inspection(
        self,
        current_motor_angles: Tuple[float, float, float],
        source_files: Optional[Dict[str, str]] = None,
    ):
        """启动左上→左下→右上→右中→右下→TR 的 Layer 1 全流程。"""
        self._player = InspectionSequencePlayer(
            self.demo_dir,
            source_files=source_files,
        )
        self._player.reset(current_motor_angles)
        self._junction.reset()
        self._reset_junction_gate()
        self._obstruction.reset()
        self._reset_obstruction_gate()

        # Layer 2/3 均由 UI 显式选择；此处不覆盖调用方的设置。
        self._base_target = np.array(current_motor_angles, dtype=np.float32)
        self._target = self._base_target.copy()
        self._mode = NavMode.NAVIGATING
        self._step_count = 0
        self._settle_count = 0
        print(f"[AutoNav] 启动全流程自主巡检，总步数={self._player.total_steps}")

    def stop(self):
        """停止自主导航，切回 IDLE。"""
        self._mode = NavMode.IDLE
        self._player = None
        self._obstruction.reset()
        self._junction.reset()
        self._reset_junction_gate()
        self._reset_obstruction_gate()
        print("[AutoNav] 已停止")

    def _reset_junction_gate(self):
        self._junction_valid_frames = 0
        self._last_bifur_center = None
        self._last_route_stage = None

    def _reset_obstruction_gate(self):
        self._obstruction_candidate_frames = 0
        self._last_obstruction_visual_timestamp = None
        self._last_obstruction_stage = None

    def pause(self):
        """暂停自主导航（保持当前位置）"""
        if self._mode == NavMode.NAVIGATING:
            self._mode = NavMode.PAUSED
            print("[AutoNav] 已暂停")

    def resume(self):
        """从暂停恢复"""
        if self._mode == NavMode.PAUSED:
            self._mode = NavMode.NAVIGATING
            print("[AutoNav] 已恢复")

    # ── 核心更新（每帧调用）──────────────────────────────────────

    def step(
        self,
        motor_angles: Tuple[float, float, float],
        visual: Dict[str, Any],
    ) -> NavDecision:
        """
        根据当前电机位置和视觉特征，输出本帧的控制决策。

        Parameters
        ----------
        motor_angles : 当前电机角度 (m0, m1, m2)
        visual       : get_visual_features() 返回的字典
                       需包含: obstruction_detected, obstruction_cx, obstruction_cy,
                               bifur_cx, bifur_cy, bifur_area, depth_mean_mm
        """
        if self._mode in (NavMode.IDLE, NavMode.DONE, NavMode.PAUSED) or self._player is None:
            cur = np.array(motor_angles, dtype=np.float32)
            return NavDecision(
                mode=self._mode,
                target_m0=cur[0], target_m1=cur[1], target_m2=cur[2],
                active=False,
                info=self._mode.value,
            )

        # Layer 1 可以在无视觉线程的情况下独立运行。外部读取异常或尚未
        # 产生首帧时，将视觉输入按空字典处理，Layer 2/3 自然不触发。
        if not isinstance(visual, dict):
            visual = {}

        obs_detected = bool(visual.get("obstruction_detected", False))
        obs_cx       = float(visual.get("obstruction_cx", 0.0))
        obs_cy       = float(visual.get("obstruction_cy", 0.0))
        obs_area     = float(visual.get("obstruction_area", 0.0))
        depth_mm     = float(
            visual.get(
                "obstruction_depth_mm",
                visual.get("depth_mean_mm", 50.0),
            )
        )
        bif_cx       = float(visual.get("bifur_cx", 0.0))
        bif_cy       = float(visual.get("bifur_cy", 0.0))
        bif_area     = float(visual.get("bifur_area", 0.0))
        bif_detected = bool(
            visual.get("bifur_detected", bif_area >= BIFUR_AREA_THRESHOLD)
        )
        if "visual_timestamp" in visual:
            visual_timestamp = float(visual.get("visual_timestamp", 0.0))
            visual_age_s = (
                max(0.0, time.monotonic() - visual_timestamp)
                if visual_timestamp > 0.0 else float("inf")
            )
            visual_fresh = visual_age_s <= BIFUR_VISUAL_MAX_AGE_S
        else:
            # 兼容离线测试和旧调用方；正式视觉共享字典始终包含时间戳。
            visual_age_s = 0.0
            visual_fresh = True

        obstruction_visual_fresh = (
            visual_age_s <= OBSTRUCTION_VISUAL_MAX_AGE_S
        )
        if "visual_timestamp" in visual:
            obstruction_visual_is_new = (
                visual_timestamp > 0.0
                and (
                    self._last_obstruction_visual_timestamp is None
                    or visual_timestamp
                    > self._last_obstruction_visual_timestamp + 1e-9
                )
            )
            if obstruction_visual_is_new:
                self._last_obstruction_visual_timestamp = visual_timestamp
        else:
            # 仅用于兼容旧的离线调用；正式视觉线程始终提供时间戳。
            obstruction_visual_is_new = True

        cur = np.array(motor_angles, dtype=np.float32)
        if self._target is None:
            self._target = cur.copy()
        if self._base_target is None:
            self._base_target = cur.copy()

        path_info = (
            self._player.get_path_info()
            if hasattr(self._player, "get_path_info") else {}
        )
        route_stage = str(path_info.get("stage_name", ""))
        route_stage_kind = str(path_info.get("stage_kind", "forward"))
        layer3_stage_allowed = bool(path_info.get("layer3_allowed", True))
        obstruction_stage = (route_stage, route_stage_kind)
        if obstruction_stage != self._last_obstruction_stage:
            self._obstruction_candidate_frames = 0
            self._last_obstruction_stage = obstruction_stage

        obstruction_values_finite = all(
            np.isfinite(value) for value in (obs_cx, obs_cy, obs_area)
        )
        obstruction_candidate = (
            self.enable_obstruction
            and self._mode == NavMode.NAVIGATING
            and layer3_stage_allowed
            and obstruction_visual_fresh
            and obs_detected
            and obstruction_values_finite
            and obs_area >= OBSTRUCTION_ENTER_AREA_THRESHOLD
        )

        if self._mode != NavMode.NAVIGATING:
            obstruction_gate_reason = self._mode.value
            self._obstruction_candidate_frames = 0
        elif not self.enable_obstruction:
            obstruction_gate_reason = "disabled"
            self._obstruction_candidate_frames = 0
        elif not layer3_stage_allowed:
            obstruction_gate_reason = f"stage:{route_stage_kind or 'unknown'}"
            self._obstruction_candidate_frames = 0
        elif not obstruction_visual_fresh:
            obstruction_gate_reason = "visual_stale"
            self._obstruction_candidate_frames = 0
        elif not obs_detected or not obstruction_values_finite:
            obstruction_gate_reason = "not_detected"
            if obstruction_visual_is_new:
                self._obstruction_candidate_frames = 0
        elif obs_area < OBSTRUCTION_ENTER_AREA_THRESHOLD:
            obstruction_gate_reason = (
                f"area_below:{obs_area:.3f}"
                f"<{OBSTRUCTION_ENTER_AREA_THRESHOLD:.3f}"
            )
            if obstruction_visual_is_new:
                self._obstruction_candidate_frames = 0
        elif not obstruction_visual_is_new:
            obstruction_gate_reason = "waiting_new_frame"
        else:
            self._obstruction_candidate_frames += 1
            obstruction_gate_reason = (
                f"stabilizing:{self._obstruction_candidate_frames}/"
                f"{OBSTRUCTION_TRIGGER_FRAMES}"
            )

        # 仅在前进段、连续若干个新视觉帧均超过面积阈值时抢占 Layer 1。
        if (
            obstruction_candidate
            and obstruction_visual_is_new
            and self._obstruction_candidate_frames >= OBSTRUCTION_TRIGGER_FRAMES
        ):
            self._obstruction.save_checkpoint(
                motor_angles=tuple(cur),
                sequence_step=self._player.current_step,
                base_target=tuple(self._base_target),
                route_stage=route_stage,
                route_stage_kind=route_stage_kind,
            )
            self._obstruction.start_clearing(tuple(cur))
            self._mode = NavMode.CLEARING
            self._junction.reset()
            self._reset_junction_gate()
            obstruction_gate_reason = "triggered"
            obstruction_stable_frames = self._obstruction_candidate_frames
            self._obstruction_candidate_frames = 0
        else:
            obstruction_stable_frames = self._obstruction_candidate_frames

        # ── 按当前模式计算指令 ─────────────────────────────────
        if self._mode == NavMode.NAVIGATING:
            return self._step_navigating(
                cur,
                bif_cx,
                bif_cy,
                bif_area,
                obs_detected,
                bifur_detected=bif_detected,
                visual_fresh=visual_fresh,
                visual_age_s=visual_age_s,
                obs_area=obs_area,
                obstruction_gate_reason=obstruction_gate_reason,
                obstruction_stable_frames=obstruction_stable_frames,
            )

        elif self._mode == NavMode.SETTLING:
            return self._step_settling(cur)

        elif self._mode in (NavMode.CLEARING, NavMode.RETURNING):
            return self._step_obstruction(
                cur,
                obs_detected,
                obs_cx,
                obs_cy,
                obs_area,
                depth_mm,
                obstruction_visual_fresh,
                obstruction_visual_is_new,
                obstruction_gate_reason,
            )

        # 不应到达这里
        return NavDecision(mode=self._mode, active=False, info="unknown state")

    # ── 子状态处理 ────────────────────────────────────────────────

    def _step_navigating(
        self,
        cur: np.ndarray,
        bif_cx: float, bif_cy: float, bif_area: float,
        obs_detected: bool,
        bifur_detected: Optional[bool] = None,
        visual_fresh: bool = True,
        visual_age_s: float = 0.0,
        obs_area: float = 0.0,
        obstruction_gate_reason: str = "",
        obstruction_stable_frames: int = 0,
    ) -> NavDecision:
        """导航模式：序列播放 + 岔口修正。"""
        # 先读取即将执行点所属的阶段；SequencePlayer.step() 会将 current_step
        # 前移一位，若之后再读取，会在前进/回退交界处提前一帧切换门控。
        path_info = (
            self._player.get_path_info()
            if hasattr(self._player, "get_path_info") else {}
        )
        seq = self._player.step()

        if seq["done"]:
            self._mode = NavMode.SETTLING
            self._settle_count = 0
            print("[AutoNav] 路径指令完成，等待真实电机到达最终角度")
            return self._step_settling(cur)

        # 序列基础增量
        dm0 = seq["delta_m0"]
        dm1 = seq["delta_m1"]
        dm2 = seq["delta_m2"]

        # 先仅按采集序列推进 Layer 1 基础目标。
        self._base_target[0] = float(np.clip(self._base_target[0] + dm0, *MOTOR_LIMITS[0]))
        self._base_target[1] = float(np.clip(self._base_target[1] + dm1, *MOTOR_LIMITS[1]))
        self._base_target[2] = float(np.clip(self._base_target[2] + dm2, *MOTOR_LIMITS[2]))

        progress = float(seq["progress"])
        route_stage = str(path_info.get("stage_name", ""))
        stage_kind = str(path_info.get("stage_kind", "forward"))
        stage_progress = float(path_info.get("stage_progress", progress))
        layer2_stage_allowed = bool(path_info.get("layer2_allowed", True))

        # 进入新阶段时清除上一阶段的 EMA 和连续帧状态，防止叶段之间串扰。
        stage_identity = (route_stage, stage_kind)
        if stage_identity != self._last_route_stage:
            self._junction.reset()
            self._junction_valid_frames = 0
            self._last_bifur_center = None
            self._last_route_stage = stage_identity

        # Layer 2 仅使用当前前进段的局部进度。远离画面中心时完全关闭；
        # 视觉停更、检测丢失或候选跳变时先等待连续稳定帧。
        center_distance = float(np.hypot(bif_cx, bif_cy))
        progress_weight = junction_progress_weight(stage_progress)
        distance_weight = junction_distance_weight(center_distance)
        values_finite = all(
            np.isfinite(value) for value in (bif_cx, bif_cy, bif_area)
        )
        detected = (
            bool(bifur_detected)
            if bifur_detected is not None else
            bif_area >= BIFUR_AREA_THRESHOLD
        )
        candidate_valid = (
            self.enable_junction
            and layer2_stage_allowed
            and visual_fresh
            and detected
            and values_finite
            and bif_area >= BIFUR_AREA_THRESHOLD
            and distance_weight > 0.0
        )

        candidate_jump = 0.0
        if candidate_valid:
            if self._last_bifur_center is not None:
                candidate_jump = float(np.hypot(
                    bif_cx - self._last_bifur_center[0],
                    bif_cy - self._last_bifur_center[1],
                ))
            if (
                self._last_bifur_center is None
                or candidate_jump <= BIFUR_MAX_TRACK_JUMP
            ):
                self._junction_valid_frames += 1
            else:
                self._junction.reset()
                self._junction_valid_frames = 1
            self._last_bifur_center = (bif_cx, bif_cy)
        else:
            self._junction_valid_frames = 0
            self._last_bifur_center = None

        stable = self._junction_valid_frames >= BIFUR_STABLE_FRAMES
        junction_strength = 0.0
        junction_active = False
        gate_reason = ""

        if not self.enable_junction:
            self._junction.reset()
            corr_m1, corr_m2 = 0.0, 0.0
            gate_reason = "disabled"
        elif not layer2_stage_allowed:
            # 回退、公共点对齐和返原点必须严格沿 Layer 1。
            self._junction.reset()
            corr_m1, corr_m2 = 0.0, 0.0
            gate_reason = f"stage:{stage_kind or 'unknown'}"
        elif distance_weight <= 0.0:
            # 用户验证表明远距离追踪不可靠，因此不保留旧偏置。
            self._junction.reset()
            corr_m1, corr_m2 = 0.0, 0.0
            gate_reason = "far_from_center"
        elif not visual_fresh:
            self._junction.reset()
            corr_m1, corr_m2 = 0.0, 0.0
            gate_reason = "visual_stale"
        elif not detected or bif_area < BIFUR_AREA_THRESHOLD or not values_finite:
            self._junction.reset()
            corr_m1, corr_m2 = 0.0, 0.0
            gate_reason = "not_detected"
        elif not stable:
            corr_m1, corr_m2 = self._junction.compute_correction(0.0, 0.0, 0.0)
            gate_reason = (
                f"stabilizing:{self._junction_valid_frames}/{BIFUR_STABLE_FRAMES}"
            )
        else:
            raw_m1, raw_m2 = self._junction.compute_correction(
                bif_cx, bif_cy, bif_area
            )
            junction_strength = progress_weight * distance_weight
            corr_m1 = raw_m1 * junction_strength
            corr_m2 = raw_m2 * junction_strength
            junction_active = self._junction.get_debug_info(
                bif_cx, bif_cy, bif_area
            )["active"]
            gate_reason = "active" if junction_active else "center_deadzone"

        previous_target = self._target.copy()
        self._target[0] = self._base_target[0]
        self._target[1] = float(np.clip(self._base_target[1] + corr_m1, *MOTOR_LIMITS[1]))
        self._target[2] = float(np.clip(self._base_target[2] + corr_m2, *MOTOR_LIMITS[2]))
        command_delta = self._target - previous_target

        self._step_count += 1

        return NavDecision(
            mode      = NavMode.NAVIGATING,
            target_m0 = self._target[0],
            target_m1 = self._target[1],
            target_m2 = self._target[2],
            delta_m0  = float(command_delta[0]),
            delta_m1  = float(command_delta[1]),
            delta_m2  = float(command_delta[2]),
            progress  = progress,
            step      = self._player.current_step,
            total_steps = self._player.total_steps,
            junction_corr_m1 = corr_m1,
            junction_corr_m2 = corr_m2,
            junction_active = junction_active,
            junction_center_distance = center_distance,
            junction_strength = junction_strength,
            junction_gate_reason = gate_reason,
            junction_stable_frames = self._junction_valid_frames,
            visual_age_s = visual_age_s,
            obstruction_detected = (obs_detected if self.enable_obstruction else False),
            obstruction_area = obs_area,
            obstruction_gate_reason = obstruction_gate_reason,
            obstruction_stable_frames = obstruction_stable_frames,
            active = True,
            info   = (route_stage or
                      f"nav step={self._player.current_step}/{self._player.total_steps}"),
            route_stage = route_stage,
            route_stage_kind = stage_kind,
            route_stage_progress = stage_progress,
        )

    def _step_settling(self, cur: np.ndarray) -> NavDecision:
        """保持最终目标，直到三轴真实角度到位或等待超时。"""
        self._settle_count += 1
        route_stage = ""
        if self._player is not None and hasattr(self._player, "get_path_info"):
            route_stage = str(
                self._player.get_path_info().get("stage_name", "")
            )
        error = np.abs(self._target - cur)
        reached = all(
            error[index] <= NAV_SETTLE_TOLERANCE[index]
            for index in range(3)
        )
        timed_out = self._settle_count >= int(NAV_SETTLE_TIMEOUT_S * CONTROL_LOOP_HZ)
        if reached or timed_out:
            self._mode = NavMode.DONE
            reason = "已到位" if reached else f"到位等待超时，误差={error.round(1).tolist()}"
            print(f"[AutoNav] 导航完成：{reason}")
            return NavDecision(
                mode=NavMode.DONE,
                target_m0=self._target[0], target_m1=self._target[1], target_m2=self._target[2],
                progress=1.0, step=self._player.current_step,
                total_steps=self._player.total_steps, active=False, info=reason,
                route_stage=route_stage,
            )
        return NavDecision(
            mode=NavMode.SETTLING,
            target_m0=self._target[0], target_m1=self._target[1], target_m2=self._target[2],
            progress=1.0, step=self._player.current_step,
            total_steps=self._player.total_steps, active=True,
            info=f"等待到位，误差={error.round(1).tolist()}",
            route_stage=route_stage,
        )

    def _step_obstruction(
        self,
        cur: np.ndarray,
        obs_detected: bool,
        obs_cx: float, obs_cy: float,
        obs_area: float,
        depth_mm: float,
        visual_fresh: bool,
        visual_is_new: bool,
        obstruction_gate_reason: str,
    ) -> NavDecision:
        """阻塞物追踪/返回模式。"""
        hdec = self._obstruction.update(
            obstruction_detected = obs_detected,
            obstruction_cx       = obs_cx,
            obstruction_cy       = obs_cy,
            depth_mean_mm        = depth_mm,
            current_motor_angles = tuple(cur),
            obstruction_area     = obs_area,
            visual_fresh         = visual_fresh,
            visual_is_new        = visual_is_new,
        )

        # 同步模式
        chk = self._obstruction.checkpoint
        if hdec.state == HandlerState.IDLE:
            if hdec.resumed:
                # 阻塞物已清除并返回断点，恢复导航
                self._mode = NavMode.NAVIGATING
                if chk is not None and self._player is not None:
                    self._player.jump_to_step(chk.sequence_step, tuple(cur))
                self._obstruction.reset()
                self._junction.reset()
                self._reset_junction_gate()
                self._reset_obstruction_gate()
                # 恢复中断时的 Layer 1 基础目标，避免丢失原轨迹累计位置。
                self._base_target = (
                    chk.base_target.copy()
                    if chk is not None and chk.base_target is not None
                    else cur.copy()
                )
                self._target = cur.copy()
                mode_str = NavMode.NAVIGATING
                info = f"resumed from step={chk.sequence_step if chk else '?'}"
            else:
                mode_str = NavMode.IDLE
                info = "handler idle"
        elif hdec.state == HandlerState.CLEARING:
            self._mode = NavMode.CLEARING
            mode_str = NavMode.CLEARING
            info = hdec.info
        else:
            self._mode = NavMode.RETURNING
            mode_str = NavMode.RETURNING
            info = hdec.info

        # 更新内部目标
        self._target = np.array(
            [hdec.target_m0, hdec.target_m1, hdec.target_m2], dtype=np.float32
        )

        return NavDecision(
            mode      = mode_str,
            target_m0 = hdec.target_m0,
            target_m1 = hdec.target_m1,
            target_m2 = hdec.target_m2,
            delta_m0  = hdec.delta_m0,
            delta_m1  = hdec.delta_m1,
            delta_m2  = hdec.delta_m2,
            progress  = (self._player.current_step / max(self._player.total_steps, 1)
                         if self._player else 0.0),
            step      = (self._player.current_step if self._player else 0),
            total_steps = (self._player.total_steps if self._player else 0),
            obstruction_detected = obs_detected,
            obstruction_area = obs_area,
            obstruction_gate_reason = (
                "resumed"
                if hdec.resumed else
                obstruction_gate_reason
            ),
            obstruction_clear_frames = hdec.clear_count,
            checkpoint_step = (chk.sequence_step if chk is not None else -1),
            active    = True,
            info      = info,
            route_stage = (chk.route_stage if chk is not None else ""),
            route_stage_kind = (
                chk.route_stage_kind if chk is not None else ""
            ),
        )

    # ── 状态查询 ──────────────────────────────────────────────────

    @property
    def mode(self) -> NavMode:
        return self._mode

    @property
    def progress(self) -> float:
        if self._player is None:
            return 0.0
        return self._player.current_step / max(self._player.total_steps, 1)

    def get_status(self) -> Dict:
        return {
            "mode":         self._mode.value,
            "progress":     self.progress,
            "step":         (self._player.current_step if self._player else 0),
            "total_steps":  (self._player.total_steps if self._player else 0),
            "handler_state": self._obstruction.state.value,
            "has_checkpoint": self._obstruction.checkpoint is not None,
            "route_stage": (
                str(self._player.get_path_info().get("stage_name", ""))
                if self._player and hasattr(self._player, "get_path_info") else ""
            ),
        }
