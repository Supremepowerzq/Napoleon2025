# -*- coding: utf-8 -*-
"""
BCRunner — 实时推理接口
==========================
在主控程序的控制循环中以 20~60Hz 运行，
输出电机增量指令。

集成到主程序（main2026-Xhandwriting-AC-Auto...py）的方式:
────────────────────────────────────────────────────────────
1. 在文件顶部导入:
      from BC.inference import BCRunner, BCDecision

2. 初始化（在 robot_init 阶段）:
      bc_runner = BCRunner("BC/checkpoints/bc_best.pth", goal_id=2)  # 左主支气管

3. 在状态机新增 AI_AUTO 状态，控制循环:
      if self.state == "AI_AUTO":
          decision = bc_runner.step(motor_group)
          motor_group.set_motor_position(0, decision.new_m0, max_speed=30)
          motor_group.set_motor_position(1, decision.new_m1, max_speed=20)
          motor_group.set_motor_position(2, decision.new_m2, max_speed=20)

4. 键盘/语音触发 AI_AUTO 模式:
      voice_commands["自主巡检"] = "ai_auto"

Author: ZQ  Date: 2026-05
"""

import sys
import time
import threading
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Dict, Any

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent.parent))

from model import (
    BronchusPolicy, ACTION_SCALE,
    load_checkpoint, BRONCHUS_PATHS,
    TASK_NAVIGATE, TASK_CLEAR,
    OBS_DIM, OBS_FIELDS,
    obs_from_config_and_motors,
)
from data_collector import get_visual_features


# ──────────────────────────────────────────────────────────────
# 输出数据结构
# ──────────────────────────────────────────────────────────────

@dataclass
class BCDecision:
    """BCRunner 每步的决策输出。"""
    # 目标电机绝对角度（°）
    new_m0:     float
    new_m1:     float
    new_m2:     float
    # 当前子任务
    task:       int           # TASK_NAVIGATE=0 or TASK_CLEAR=1
    task_name:  str           # "navigate" / "clear_obstruction"
    # 置信度
    task_prob:  float         # softmax 概率
    # 调试信息
    raw_action: np.ndarray    # 归一化动作 [-1,1]
    goal_id:    int
    step_count: int


# ──────────────────────────────────────────────────────────────
# 安全限位（与主程序保持一致）
# ──────────────────────────────────────────────────────────────
MOTOR_LIMITS = {
    0: (-900.0, 0.0),
    1: (-170.0, 170.0),
    2: (-600.0, 600.0),
}

# 单步最大角度变化量（安全保护）
MAX_DELTA_DEG = {0: 8.0, 1: 5.0, 2: 5.0}


# ──────────────────────────────────────────────────────────────
# 主类
# ──────────────────────────────────────────────────────────────

class BCRunner:
    """
    实时 BC 策略推理器。

    Parameters
    ----------
    model_path : str   训练好的 checkpoint 路径
    goal_id    : int   目标支气管标签（见 BRONCHUS_PATHS）
    device     : str   "auto" / "cuda" / "cpu"
    seq_len    : int   GRU 历史帧数，必须与训练时一致（default=8）
    action_scale_factor : float  全局动作幅度缩放（初始建议 0.3，调稳后增大）
    """

    def __init__(
        self,
        model_path:          str,
        goal_id:             int   = 0,
        device:              str   = "auto",
        seq_len:             int   = 8,
        action_scale_factor: float = 0.3,
    ):
        if device == "auto":
            self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        else:
            self._device = torch.device(device)

        self.goal_id    = goal_id
        self.seq_len    = seq_len
        self._scale     = action_scale_factor

        # 加载模型
        self._model, ckpt = load_checkpoint(model_path, str(self._device))
        self._model.eval()

        # GRU 隐状态
        self._hidden: Optional[torch.Tensor] = None

        # 观测历史缓冲区（用全零初始化）
        self._obs_buffer: deque = deque(
            [np.zeros(OBS_DIM, dtype=np.float32)] * seq_len,
            maxlen=seq_len,
        )

        # 动作平滑滤波（指数移动平均）
        self._ema_action = np.zeros(3, dtype=np.float32)
        self._ema_alpha  = 0.6   # 提高响应速度（0.4→0.6）

        # 内部目标角度累积器（不依赖电机位置反馈，避免整数截断导致目标永远在原点）
        self._target_angles: Optional[np.ndarray] = None  # 首步从实际位置初始化

        # 统计
        self._step_count = 0
        self._last_task  = TASK_NAVIGATE
        self._lock = threading.Lock()

        path_name = BRONCHUS_PATHS.get(goal_id, "unknown")
        print(f"[BCRunner] 已加载模型: {model_path}")
        print(f"  目标支气管: [{goal_id}] {path_name}")
        print(f"  设备: {self._device}  seq_len={seq_len}")
        print(f"  动作缩放: {action_scale_factor}（可调 runner.set_scale(x)）")

    # ── 核心推理接口 ───────────────────────────────────────────

    def step(self, motor_group) -> BCDecision:
        """
        推理一步，返回 BCDecision。

        Parameters
        ----------
        motor_group : MotorGroup2025 实例
        """
        with self._lock:
            # 1. 读取当前电机角度
            try:
                angles = motor_group.get_cached_angles()
                m0, m1, m2 = float(angles[0]), float(angles[1]), float(angles[2])
            except Exception:
                m0 = m1 = m2 = 0.0

            # 首步：用实际电机位置初始化内部累积目标
            if self._target_angles is None:
                self._target_angles = np.array([m0, m1, m2], dtype=np.float32)

            # 2. 读取视觉特征
            vis = get_visual_features()
            obs = obs_from_config_and_motors(
                obstruction_cx       = vis.get("obstruction_cx",       0.0),
                obstruction_cy       = vis.get("obstruction_cy",       0.0),
                obstruction_area     = vis.get("obstruction_area",     0.0),
                obstruction_detected = bool(vis.get("obstruction_detected", False)),
                bifur_cx        = vis.get("bifur_cx",        0.0),
                bifur_cy        = vis.get("bifur_cy",        0.0),
                bifur_area      = vis.get("bifur_area",      0.0),
                depth_center_mm = vis.get("depth_center_mm", 0.0),
                depth_mean_mm   = vis.get("depth_mean_mm",   0.0),
                depth_max_mm    = vis.get("depth_max_mm",    0.0),
                path_depths     = (vis.get("path_left_mm",   0.0),
                                   vis.get("path_center_mm", 0.0),
                                   vis.get("path_right_mm",  0.0)),
                motor_angles    = (m0, m1, m2),
            )  # [OBS_DIM]

            # 3. 更新缓冲区
            self._obs_buffer.append(obs.numpy() if hasattr(obs, "numpy") else obs)

            # 4. 模型推理
            obs_seq = torch.tensor(
                np.stack(list(self._obs_buffer)),  # [T, OBS_DIM]
                dtype=torch.float32
            ).unsqueeze(0).to(self._device)        # [1, T, OBS_DIM]

            goal_t = torch.tensor([self.goal_id], dtype=torch.long, device=self._device)

            with torch.no_grad():
                raw_action, task_logits, self._hidden = self._model(
                    obs_seq, goal_t, self._hidden
                )

            raw  = raw_action[0].cpu().numpy()   # [-1,1] × 3
            task_probs  = torch.softmax(task_logits[0], dim=0).cpu().numpy()
            task        = int(task_probs.argmax())
            task_prob   = float(task_probs[task])

            # 5. 反归一化 + 动作缩放
            scale = ACTION_SCALE.numpy()          # [5, 3, 3] °
            delta = raw * scale * self._scale     # 实际角度增量

            # 6. 指数移动平均平滑
            self._ema_action = (self._ema_alpha * delta
                                + (1 - self._ema_alpha) * self._ema_action)
            delta_smooth = self._ema_action

            # 7. 单步限幅（安全保护）
            delta_smooth[0] = np.clip(delta_smooth[0], -MAX_DELTA_DEG[0], MAX_DELTA_DEG[0])
            delta_smooth[1] = np.clip(delta_smooth[1], -MAX_DELTA_DEG[1], MAX_DELTA_DEG[1])
            delta_smooth[2] = np.clip(delta_smooth[2], -MAX_DELTA_DEG[2], MAX_DELTA_DEG[2])

            # 8. 累积目标角度（每步叠加 delta，不依赖位置反馈的整数截断）
            self._target_angles[0] = float(np.clip(
                self._target_angles[0] + delta_smooth[0], *MOTOR_LIMITS[0]))
            self._target_angles[1] = float(np.clip(
                self._target_angles[1] + delta_smooth[1], *MOTOR_LIMITS[1]))
            self._target_angles[2] = float(np.clip(
                self._target_angles[2] + delta_smooth[2], *MOTOR_LIMITS[2]))
            new_m0 = float(self._target_angles[0])
            new_m1 = float(self._target_angles[1])
            new_m2 = float(self._target_angles[2])

            self._step_count += 1
            self._last_task   = task

            return BCDecision(
                new_m0    = new_m0,
                new_m1    = new_m1,
                new_m2    = new_m2,
                task      = task,
                task_name = "clear_obstruction" if task == TASK_CLEAR else "navigate",
                task_prob = task_prob,
                raw_action = raw,
                goal_id    = self.goal_id,
                step_count = self._step_count,
            )

    # ── 控制接口 ───────────────────────────────────────────────

    def reset(self):
        """切换新路径目标或重新进入 AI_AUTO 模式时调用。"""
        with self._lock:
            self._hidden        = None
            self._ema_action    = np.zeros(3, dtype=np.float32)
            self._target_angles = None   # 下次 step() 首步从实际位置重新初始化
            self._obs_buffer    = deque(
                [np.zeros(OBS_DIM, dtype=np.float32)] * self.seq_len,
                maxlen=self.seq_len,
            )
            self._step_count = 0
        print(f"[BCRunner] 已重置 → goal_id={self.goal_id} "
              f"({BRONCHUS_PATHS.get(self.goal_id,'?')})")

    def set_goal(self, goal_id: int):
        """切换目标支气管（切换前自动 reset）。"""
        self.goal_id = goal_id
        self.reset()

    def set_scale(self, factor: float):
        """
        动态调节动作幅度缩放。
        建议值: 0.2（保守） → 0.5（正常） → 1.0（激进）
        """
        self._scale = float(factor)
        print(f"[BCRunner] 动作缩放已设为 {self._scale}")

    # ── 状态查询 ───────────────────────────────────────────────

    @property
    def current_task_name(self) -> str:
        return "clear_obstruction" if self._last_task == TASK_CLEAR else "navigate"

    @property
    def step_count(self) -> int:
        return self._step_count


# ──────────────────────────────────────────────────────────────
# 主程序集成帮助函数
# ──────────────────────────────────────────────────────────────

_bc_runner_instance: Optional[BCRunner] = None


def get_bc_runner(
    model_path:          str   = "BC/checkpoints/bc_best.pth",
    goal_id:             int   = 0,
    action_scale_factor: float = 0.3,
) -> BCRunner:
    """
    单例工厂：获取全局 BCRunner 实例（自动懒加载）。
    在主程序控制循环中直接调用，无需手动管理实例。
    """
    global _bc_runner_instance
    if _bc_runner_instance is None:
        _bc_runner_instance = BCRunner(
            model_path, goal_id, action_scale_factor=action_scale_factor
        )
    return _bc_runner_instance


# ──────────────────────────────────────────────────────────────
# 集成示例：在主程序状态机中添加 AI_AUTO 状态
# ──────────────────────────────────────────────────────────────

AI_AUTO_INTEGRATION_SNIPPET = '''
# ─── 在 main2026-Xhandwriting-AC-Auto...py 中的添加位置 ─────────

# 1. 文件顶部导入（与其他 import 并排）
from BC.inference import BCRunner, BCDecision

# 2. 在 MotorGroup2025 初始化之后添加
bc_runner: Optional[BCRunner] = None

# 3. 在语音命令字典中添加
STANDARD_VOICE_COMMANDS["自主巡检"] = "ai_auto"
STANDARD_VOICE_COMMANDS["切换为自主模式"] = "ai_auto"

# 4. 在状态机 states 列表中添加
states = ['Idle', 'ManualControl', 'VisionTracking', 'AIAuto', 'PowerOff']

# 5. 新增控制循环处理函数
def _loop_ai_auto(self):
    global bc_runner

    # 懒加载模型（第一次进入时）
    if bc_runner is None:
        try:
            from BC.inference import BCRunner
            from BC.model import BRONCHUS_PATHS
            bc_runner = BCRunner(
                model_path="BC/checkpoints/bc_best.pth",
                goal_id=0,              # 0=自主探索；改为 2 则走左主支气管
                action_scale_factor=0.3  # 先用小值，验证安全后调大
            )
            self.voice_window.push_log(f"BC策略已加载 → 自主巡检模式")
        except Exception as e:
            self.voice_window.push_log(f"BC模型加载失败: {e}")
            self._on_command("manual")
            return

    # 推理一步
    decision = bc_runner.step(self.motor_group)

    # 执行动作
    self.motor_group.set_motor_position(0, decision.new_m0, max_speed=25.0)
    self.motor_group.set_motor_position(1, decision.new_m1, max_speed=15.0)
    self.motor_group.set_motor_position(2, decision.new_m2, max_speed=15.0)

    # 状态日志
    self.voice_window.update_state(
        "AI自主巡检",
        f"[{decision.goal_id}]{BRONCHUS_PATHS.get(decision.goal_id,'?')} "
        f"| {decision.task_name} | 步={decision.step_count}"
    )

# 6. 在 _handle_command() 中添加
elif cmd == "ai_auto":
    if bc_runner:
        bc_runner.reset()
    self.machine.trigger("start_ai_auto")
'''


if __name__ == "__main__":
    print("集成代码片段（复制到主程序）:")
    print(AI_AUTO_INTEGRATION_SNIPPET)
