"""
动作录制和回放模块
支持录制电机动作序列，保存为文件，并提供回放功能（支持暂停、倒放、倍速）

使用说明：
1. 录制动作：
   - 点击"开始录制"按钮开始录制所有电机操作
   - 在手动模式或视觉模式下操作电机
   - 点击"停止录制"按钮停止录制并保存文件
   - 文件保存在 recorded_actions/ 目录下，命名为 action_YYYYMMDD_HHMMSS.json

2. 回放动作：
   - 在动作列表中选择要回放的动作文件（双击或选择后点击播放）
   - 点击"播放"按钮开始回放（自动回到初始状态后执行）
   - 点击"暂停"按钮暂停/继续回放
   - 点击"停止"按钮停止回放
   - 点击"倒放"按钮倒放动作（从终止状态回到初始状态）
   - 使用倍速选择器调整回放速度（0.5x, 1.0x, 2.0x）
   - 实时显示总时长、已播放时间、剩余时间和进度条
"""

import json
import os
import time
import threading
from typing import Dict, List, Optional, Tuple, Callable
import math
from datetime import datetime
from enum import Enum


class PlaybackState(Enum):
    """回放状态"""
    STOPPED = "stopped"
    PLAYING = "playing"
    PAUSED = "paused"
    REVERSING = "reversing"


class ActionRecorder:
    """动作录制器"""
    
    def __init__(self, motor_group, save_dir: str = "recorded_actions"):
        """
        初始化录制器
        
        Args:
            motor_group: 电机组对象（MotorGroup2025）
            save_dir: 保存录制文件的目录
        """
        self.motor_group = motor_group
        self.save_dir = save_dir
        self.is_recording = False
        self.recorded_actions: List[Dict] = []
        self.start_time: Optional[float] = None
        self.initial_state: Optional[Dict] = None
        
        # 确保保存目录存在
        os.makedirs(save_dir, exist_ok=True)
    
    def start_recording(self) -> bool:
        """开始录制"""
        if self.is_recording:
            return False
        
        # 记录初始状态（优先使用电机缓存角度，避免额外串口查询带来超时）
        try:
            if hasattr(self.motor_group, "get_cached_angles"):
                angles = self.motor_group.get_cached_angles()
            else:
                angles = self.motor_group.get_angles()
            self.initial_state = {
                'timestamp': 0.0,
                'motor_0': angles[0],
                'motor_1': angles[1],
                'motor_2': angles[2],
            }
            self.recorded_actions = [self.initial_state.copy()]
            self.start_time = time.time()
            self.is_recording = True
            return True
        except Exception as e:
            print(f"开始录制失败: {e}")
            return False
    
    def stop_recording(self) -> Optional[str]:
        """停止录制并保存文件"""
        if not self.is_recording:
            return None
        
        self.is_recording = False
        
        # 记录终止状态
        try:
            angles = self.motor_group.get_angles()
            end_time = time.time()
            final_state = {
                'timestamp': end_time - self.start_time if self.start_time else 0.0,
                'motor_0': angles[0],
                'motor_1': angles[1],
                'motor_2': angles[2],
            }
            self.recorded_actions.append(final_state)
        except Exception as e:
            print(f"记录终止状态失败: {e}")
        
        # 生成文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        file_path = os.path.join(self.save_dir, f"action_{timestamp}.json")
        
        # 保存到文件
        try:
            action_data = {
                'initial_state': self.initial_state,
                'final_state': self.recorded_actions[-1] if self.recorded_actions else None,
                'actions': self.recorded_actions,
                'duration': self.recorded_actions[-1]['timestamp'] if self.recorded_actions else 0.0,
                'created_at': timestamp
            }
            
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(action_data, f, indent=2, ensure_ascii=False)
            
            print(f"录制已保存: {file_path}")
            return file_path
        except Exception as e:
            print(f"保存录制文件失败: {e}")
            return None
        finally:
            # 清理
            self.recorded_actions = []
            self.start_time = None
            self.initial_state = None
    
    def record_action(self) -> None:
        """记录当前电机状态（应在控制循环中定期调用）"""
        if not self.is_recording:
            return
        
        try:
            if hasattr(self.motor_group, "get_cached_angles"):
                angles = self.motor_group.get_cached_angles()
            else:
                angles = self.motor_group.get_angles()
            current_time = time.time()
            timestamp = current_time - self.start_time if self.start_time else 0.0
            
            # 只记录有变化的状态（减少数据量）
            last_action = self.recorded_actions[-1] if self.recorded_actions else None
            if last_action:
                # 检查是否有显著变化（阈值：0.1度）
                if (abs(angles[0] - last_action['motor_0']) < 0.1 and
                    abs(angles[1] - last_action['motor_1']) < 0.1 and
                    abs(angles[2] - last_action['motor_2']) < 0.1):
                    return  # 没有显著变化，不记录
            
            action = {
                'timestamp': timestamp,
                'motor_0': angles[0],
                'motor_1': angles[1],
                'motor_2': angles[2],
            }
            self.recorded_actions.append(action)
        except Exception as e:
            print(f"记录动作失败: {e}")


class ActionPlayer:
    """动作回放器"""
    
    def __init__(
        self,
        motor_group,
        update_callback: Optional[Callable[[Dict], None]] = None,
        finish_callback: Optional[Callable[[bool], None]] = None,
    ):
        """
        初始化回放器
        
        Args:
            motor_group: 电机组对象（MotorGroup2025）
            update_callback: 更新回调函数，用于更新UI（进度、时间等）
        """
        self.motor_group = motor_group
        self.update_callback = update_callback
        self.finish_callback = finish_callback
        
        self.state = PlaybackState.STOPPED
        self.current_action_data: Optional[Dict] = None
        self.current_index = 0
        self.playback_speed = 1.0  # 倍速：0.5, 1.0, 2.0
        self.is_reverse = False
        
        self.playback_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.pause_event = threading.Event()
        self.pause_event.set()  # 初始为暂停状态
        
        # 回放控制频率（Hz）
        self.playback_frequency = 60
        # 目标插值频率（回放轨迹分辨率），以及允许的最大角度步长（度）
        # 这里选择较温和的参数，减轻串口压力，避免电机因指令过密而抖动或超时
        self.sequence_step = 0.02  # 基础时间步，约 20ms
        self.interp_target_hz = 20.0  # 目标插值频率 ~20Hz
        self.interp_max_angle_step = 2.0  # 单步角度变化不超过 2°
        self.playback_sequence: List[Dict] = []
        # 上一帧下发的目标角度，用于抑制微小抖动和限制单步速度
        self._last_cmd_m0: Optional[float] = None
        self._last_cmd_m1: Optional[float] = None
        self._last_cmd_m2: Optional[float] = None
        self.min_move_delta = 0.3    # 小于此角度变化则忽略（度）
        self.max_speed_deg = 80.0    # set_position 的最大速度上限（度/秒）
        self.total_playback_duration: float = 0.0
    
    def load_action_file(self, file_path: str) -> bool:
        """加载动作文件"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                self.current_action_data = json.load(f)
            self.current_index = 0
            return True
        except Exception as e:
            print(f"加载动作文件失败: {e}")
            return False
    
    def get_available_actions(self) -> List[Tuple[str, str]]:
        """获取所有可用的动作文件列表"""
        recorder = ActionRecorder(self.motor_group)
        save_dir = recorder.save_dir
        
        if not os.path.exists(save_dir):
            return []
        
        actions = []
        for filename in sorted(os.listdir(save_dir), reverse=True):
            if filename.endswith('.json'):
                file_path = os.path.join(save_dir, filename)
                try:
                    with open(file_path, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                        duration = data.get('duration', 0.0)
                        created_at = data.get('created_at', filename)
                        actions.append((file_path, f"{created_at} ({duration:.1f}s)"))
                except:
                    pass
        
        return actions
    
    def start_playback(self, file_path: str, reverse: bool = False) -> bool:
        """开始回放"""
        if self.state != PlaybackState.STOPPED:
            self.stop_playback()
        
        if not self.load_action_file(file_path):
            return False

        if self.current_action_data is None:
            print("动作文件加载后数据为空，无法回放")
            return False

        action_data = self.current_action_data
        actions = action_data.get('actions', [])
        if not actions:
            print("动作文件中不包含任何动作")
            return False

        self.playback_sequence = self._prepare_playback_sequence(actions, reverse)
        if not self.playback_sequence:
            print("动作序列为空，无法回放")
            return False

        # 以录制时保存的 duration 为基准调整时间轴，确保 1x 播放时长与录制一致
        recorded_duration = float(self.current_action_data.get("duration", 0.0))
        seq_duration = float(self.playback_sequence[-1].get("timestamp", 0.0))
        if recorded_duration > 0 and seq_duration > 0 and abs(recorded_duration - seq_duration) > 1e-3:
            scale = recorded_duration / seq_duration
            for item in self.playback_sequence:
                item["timestamp"] = float(item.get("timestamp", 0.0)) * scale
            self.total_playback_duration = recorded_duration
        else:
            # 两者本身就接近，则直接使用重采样序列的终止时间
            self.total_playback_duration = seq_duration
        
        self.is_reverse = reverse
        self.current_index = 0
        self.state = PlaybackState.REVERSING if reverse else PlaybackState.PLAYING
        
        # 先移动到初始状态（倒放时移动到终止状态）
        # 为避免在这里长时间阻塞串口导致“卡死”，改为非强制预对齐：
        # 如有错误仅打印日志，不影响后续正式回放；主要由回放过程逐步对齐轨迹。
        if reverse:
            target_state = action_data.get('final_state')
            if not target_state and actions:
                target_state = actions[-1]
        else:
            target_state = action_data.get('initial_state')
        
        if target_state:
            try:
                target_m0 = float(target_state.get('motor_0', 0.0))
                target_m1 = float(target_state.get('motor_1', 0.0))
                target_m2 = float(target_state.get('motor_2', 0.0))

                if hasattr(self.motor_group, "m0_limit"):
                    lo, hi = self.motor_group.m0_limit
                    target_m0 = max(lo, min(target_m0, hi))
                if hasattr(self.motor_group, "m1_limit"):
                    lo, hi = self.motor_group.m1_limit
                    target_m1 = max(lo, min(target_m1, hi))
                if hasattr(self.motor_group, "m2_limit"):
                    lo, hi = self.motor_group.m2_limit
                    target_m2 = max(lo, min(target_m2, hi))

                # 只发送一次对齐指令，不做长时间等待
                try:
                    self.motor_group.set_motor_position(0, target_m0)
                    self.motor_group.set_motor_position(1, target_m1)
                    self.motor_group.set_motor_position(2, target_m2)
                except Exception as ee:
                    print(f"预对齐目标位置失败: {ee}")
            except Exception as e:
                print(f"计算预对齐目标位置失败: {e}")
        
        # 启动回放线程
        self.stop_event.clear()
        self.pause_event.set()
        self.playback_thread = threading.Thread(target=self._playback_loop, daemon=True)
        self.playback_thread.start()
        
        return True
    
    def pause_playback(self) -> None:
        """暂停/继续回放"""
        if self.state == PlaybackState.PLAYING:
            self.pause_event.clear()
            self.state = PlaybackState.PAUSED
        elif self.state == PlaybackState.PAUSED:
            self.pause_event.set()
            self.state = PlaybackState.PLAYING
        elif self.state == PlaybackState.REVERSING:
            self.pause_event.clear()
            self.state = PlaybackState.PAUSED
        elif self.state == PlaybackState.PAUSED and self.is_reverse:
            self.pause_event.set()
            self.state = PlaybackState.REVERSING
    
    def stop_playback(self) -> None:
        """停止回放"""
        self.stop_event.set()
        self.pause_event.set()  # 确保线程能退出
        if self.playback_thread and self.playback_thread.is_alive():
            self.playback_thread.join(timeout=2.0)
        self.state = PlaybackState.STOPPED
        self.current_index = 0
        self.playback_sequence = []
        self.total_playback_duration = 0.0
        self._last_cmd_m0 = self._last_cmd_m1 = self._last_cmd_m2 = None
        try:
            self.motor_group.stop()
        except:
            pass
    
    def set_playback_speed(self, speed: float) -> None:
        """设置回放倍速（0.5, 1.0, 2.0）"""
        if speed in [0.5, 1.0, 2.0]:
            self.playback_speed = speed
    
    def _playback_loop(self) -> None:
        """回放循环（在独立线程中运行）"""
        if not self.playback_sequence:
            return
        total_duration = self.total_playback_duration or (self.current_action_data.get('duration', 0.0) if self.current_action_data else 0.0)
        prev_timestamp = self.playback_sequence[0]['timestamp']
        # 初始化上一帧命令角度为当前实际角度，避免第一步大跳变
        try:
            current_angles = self.motor_group.get_cached_angles() if hasattr(self.motor_group, "get_cached_angles") else self.motor_group.get_angles()
            self._last_cmd_m0, self._last_cmd_m1, self._last_cmd_m2 = current_angles
        except Exception:
            self._last_cmd_m0 = self._last_cmd_m1 = self._last_cmd_m2 = None
        
        completed = False
        try:
            while not self.stop_event.is_set() and self.current_index < len(self.playback_sequence):
                # 检查暂停
                self.pause_event.wait()
                
                if self.stop_event.is_set():
                    break

                action = self.playback_sequence[self.current_index]
                current_timestamp = action['timestamp']
                time_delta = max(current_timestamp - prev_timestamp, 0.0) / self.playback_speed
                
                # 执行动作（对 M0/M1/M2 进行位置控制，带抖动抑制与速度限制）
                try:
                    m0 = float(action.get('motor_0', 0.0))
                    m1 = float(action.get('motor_1', 0.0))
                    m2 = float(action.get('motor_2', 0.0))

                    # 根据限位裁剪，防止越界
                    try:
                        if hasattr(self.motor_group, "m0_limit"):
                            lo, hi = self.motor_group.m0_limit
                            m0 = max(lo, min(m0, hi))
                        if hasattr(self.motor_group, "m1_limit"):
                            lo, hi = self.motor_group.m1_limit
                            m1 = max(lo, min(m1, hi))
                        if hasattr(self.motor_group, "m2_limit"):
                            lo, hi = self.motor_group.m2_limit
                            m2 = max(lo, min(m2, hi))
                    except Exception:
                        pass

                    # 与上一帧命令比较，抑制微小抖动
                    last0 = self._last_cmd_m0 if self._last_cmd_m0 is not None else m0
                    last1 = self._last_cmd_m1 if self._last_cmd_m1 is not None else m1
                    last2 = self._last_cmd_m2 if self._last_cmd_m2 is not None else m2

                    d0 = m0 - last0
                    d1 = m1 - last1
                    d2 = m2 - last2

                    # 如果变化太小，直接忽略该轴
                    if abs(d0) < self.min_move_delta:
                        m0 = last0
                    if abs(d1) < self.min_move_delta:
                        m1 = last1
                    if abs(d2) < self.min_move_delta:
                        m2 = last2

                    # 估算所需速度并进行限幅
                    if time_delta > 0:
                        v0 = min(self.max_speed_deg, abs(d0) / time_delta) if abs(d0) >= self.min_move_delta else 0.0
                        v1 = min(self.max_speed_deg, abs(d1) / time_delta) if abs(d1) >= self.min_move_delta else 0.0
                        v2 = min(self.max_speed_deg, abs(d2) / time_delta) if abs(d2) >= self.min_move_delta else 0.0
                    else:
                        v0 = v1 = v2 = self.max_speed_deg

                    # 仅对有明显变化的轴下发位置指令
                    if abs(m0 - last0) >= self.min_move_delta:
                        self.motor_group.set_motor_position(0, m0, max_speed=v0 or self.max_speed_deg)
                        self._last_cmd_m0 = m0
                    if abs(m1 - last1) >= self.min_move_delta:
                        self.motor_group.set_motor_position(1, m1, max_speed=v1 or self.max_speed_deg)
                        self._last_cmd_m1 = m1
                    if abs(m2 - last2) >= self.min_move_delta:
                        self.motor_group.set_motor_position(2, m2, max_speed=v2 or self.max_speed_deg)
                        self._last_cmd_m2 = m2
                except Exception as e:
                    print(f"回放执行失败: {e}")

                # 更新进度
                if self.update_callback:
                    progress = ((self.current_index + 1) / len(self.playback_sequence)) * 100
                    elapsed = current_timestamp
                    remaining = total_duration - elapsed
                    self.update_callback({
                        'progress': progress,
                        'elapsed': elapsed,
                        'remaining': remaining,
                        'total': total_duration,
                        'speed': self.playback_speed,
                        'state': self.state.value
                    })
                
                if time_delta > 0:
                    time.sleep(time_delta)

                self.current_index += 1
                prev_timestamp = current_timestamp
            
            # 回放结束，确保进度归零/停止
            if not self.stop_event.is_set():
                completed = True
                if self.update_callback:
                    self.update_callback({
                        'progress': 100.0,
                        'elapsed': total_duration,
                        'remaining': 0.0,
                        'total': total_duration,
                        'speed': self.playback_speed,
                        'state': PlaybackState.STOPPED.value
                    })
        
        except Exception as e:
            print(f"回放循环异常: {e}")
        finally:
            self.state = PlaybackState.STOPPED
            try:
                self.motor_group.stop()
            except:
                pass
            if self.finish_callback:
                self.finish_callback(completed)

    def _prepare_playback_sequence(self, actions: List[Dict], reverse: bool) -> List[Dict]:
        """
        准备回放序列：
        - 先按录制的时间戳进行排序并归零；
        - 对角度做轻度平滑，减小手动抖动；
        - 依据时间间隔与角度变化自适应插值，生成近似 50Hz 的平滑轨迹；
        - 总时长严格与录制时保持一致。
        """
        if not actions:
            return []

        # 正放/倒放排序
        ordered_actions = list(reversed(actions)) if reverse else list(actions)

        # 时间基准归零
        t0 = float(ordered_actions[0].get("timestamp", 0.0))
        base: List[Dict] = []
        for a in ordered_actions:
            base.append({
                "timestamp": max(float(a.get("timestamp", 0.0)) - t0, 0.0),
                "motor_0": float(a.get("motor_0", 0.0)),
                "motor_1": float(a.get("motor_1", 0.0)),
                "motor_2": float(a.get("motor_2", 0.0)),
            })

        # 简单 3 点移动平均平滑角度，减少噪声
        smoothed: List[Dict] = []
        n = len(base)
        for i in range(n):
            w_total = 0.0
            acc0 = acc1 = acc2 = 0.0
            for j, w in ((i - 1, 1.0), (i, 2.0), (i + 1, 1.0)):
                if 0 <= j < n:
                    acc0 += base[j]["motor_0"] * w
                    acc1 += base[j]["motor_1"] * w
                    acc2 += base[j]["motor_2"] * w
                    w_total += w
            if w_total <= 0:
                smoothed.append(base[i])
            else:
                smoothed.append({
                    "timestamp": base[i]["timestamp"],
                    "motor_0": acc0 / w_total,
                    "motor_1": acc1 / w_total,
                    "motor_2": acc2 / w_total,
                })

        # 基于时间+角度变化的自适应插值
        sequence: List[Dict] = [smoothed[0]]
        for i in range(1, n):
            prev_state = smoothed[i - 1]
            next_state = smoothed[i]
            dt = max(next_state["timestamp"] - prev_state["timestamp"], 1e-4)

            # 根据时间与角度变化决定步数
            max_angle_delta = max(
                abs(next_state["motor_0"] - prev_state["motor_0"]),
                abs(next_state["motor_1"] - prev_state["motor_1"]),
                abs(next_state["motor_2"] - prev_state["motor_2"]),
            )
            steps_time = dt * self.interp_target_hz
            steps_angle = max_angle_delta / max(self.interp_max_angle_step, 1e-3)
            steps = max(1, int(steps_time), int(steps_angle))

            step_time = dt / steps
            delta_0 = next_state["motor_0"] - prev_state["motor_0"]
            delta_1 = next_state["motor_1"] - prev_state["motor_1"]
            delta_2 = next_state["motor_2"] - prev_state["motor_2"]

            for s in range(1, steps + 1):
                alpha = s / steps
                sequence.append({
                    "timestamp": prev_state["timestamp"] + step_time * s,
                    "motor_0": prev_state["motor_0"] + alpha * delta_0,
                    "motor_1": prev_state["motor_1"] + alpha * delta_1,
                    "motor_2": prev_state["motor_2"] + alpha * delta_2,
                })

        return sequence
    
    def get_progress(self) -> Dict:
        """获取当前回放进度"""
        if not self.playback_sequence:
            return {
                'progress': 0.0,
                'elapsed': 0.0,
                'remaining': 0.0,
                'total': 0.0,
                'speed': self.playback_speed,
                'state': self.state.value
            }
        
        sequence = self.playback_sequence
        total_duration = self.total_playback_duration or (sequence[-1]['timestamp'] if sequence else 0.0)
        
        if not sequence:
            progress = elapsed = 0.0
        else:
            progress = min((self.current_index / len(sequence)) * 100, 100.0)
            if self.current_index < len(sequence):
                elapsed = sequence[self.current_index]['timestamp']
            else:
                elapsed = total_duration
        
        remaining = max(0.0, total_duration - elapsed)
        
        return {
            'progress': progress,
            'elapsed': elapsed,
            'remaining': remaining,
            'total': total_duration,
            'speed': self.playback_speed,
            'state': self.state.value
        }

