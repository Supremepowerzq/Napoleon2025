# -*- coding: utf-8 -*-
"""
AutoNav — 支气管自主巡检系统
=================================
三层自主导航架构：
  Layer 1  SequencePlayer       — 纯电机序列导航（从固定起点到达目标部位）
  Layer 2  JunctionGuide        — 实时岔口识别引导（路线纠正，减少碰壁）
  Layer 3  ObstructionHandler   — 阻塞物检测→清除→断点续航

用法:
    from AutoNav import AutoNavController
    ctrl = AutoNavController(demo_dir="BC/expert_demos", motor_group=mg)
    ctrl.start(path_label=2, current_motor_angles=mg.get_cached_angles())

    # 在控制循环中每帧调用
    decision = ctrl.step(motor_angles=mg.get_cached_angles(),
                         visual=get_visual_features())
"""

from .nav_controller import AutoNavController, NavDecision, NavMode
from .sequence_player import SequencePlayer
from .inspection_player import (
    DEFAULT_INSPECTION_ROUTE,
    InspectionSequencePlayer,
    list_inspection_source_files,
)
from .junction_guide import JunctionGuide
from .obstruction_handler import ObstructionHandler

__all__ = [
    "AutoNavController", "NavDecision", "NavMode",
    "SequencePlayer", "InspectionSequencePlayer",
    "DEFAULT_INSPECTION_ROUTE", "list_inspection_source_files",
    "JunctionGuide", "ObstructionHandler",
]
