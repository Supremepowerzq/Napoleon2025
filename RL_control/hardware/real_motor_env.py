"""
真实硬件接口
将训练好的 RL 策略部署到真实 RMD 电机硬件上

功能:
1. 与现有 MotorGroup/RmdInterfaceV2 无缝集成
2. Gymnasium 环境接口
3. RealSense L515 相机集成
4. 实时控制循环
5. 安全保障
"""

import os
import sys
import time
import numpy as np
import threading
from typing import Dict, List, Tuple, Optional, Any, Callable
from dataclasses import dataclass
import warnings

# 尝试导入现有接口
try:
    sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))
    from Interface.RmdInterfaceV2 import RmdInterface
    from MotorGroup.RmdGroupV2 import RmdGroup
    from Interface.CameraInterface import CameraInterface
except ImportError:
    RmdInterface = None
    RmdGroup = None
    CameraInterface = None

import gymnasium as gym
from vision.vision_processor import RealSenseCamera, VisionObservationModule
from config.rl_config import MOTOR_CONFIG


# ============================================================
# 硬件配置
# ============================================================

@dataclass
class HardwareConfig:
    """硬件配置"""
    # 串口配置
    serial_port: str = "COM3"
    baudrate: int = 115200
    serial_timeout: float = 0.1

    # 电机配置
    num_motors: int = 3
    motor_ids: List[int] = None

    # 相机配置
    camera_width: int = 640
    camera_height: int = 480
    camera_fps: int = 30

    # 控制配置
    control_frequency: float = 60.0  # Hz
    action_scale: float = 1.0  # 动作缩放因子
    safety_limits: Dict = None

    def __post_init__(self):
        if self.motor_ids is None:
            self.motor_ids = list(range(self.num_motors))
        if self.safety_limits is None:
            self.safety_limits = {
                "m0": (-1000, 1000),
                "m1": (-135, 135),
                "m2": (-135, 135),
                "max_velocity": 120.0,
            }


# ============================================================
# 安全控制器
# ============================================================

class SafetyController:
    """
    安全控制器
    监控电机状态，防止危险操作
    """

    def __init__(self, config: HardwareConfig):
        self.config = config
        self.enabled = True

        # 限位
        self.position_limits = {
            0: config.safety_limits.get("m0", (-1000, 1000)),
            1: config.safety_limits.get("m1", (-135, 135)),
            2: config.safety_limits.get("m2", (-135, 135)),
        }

        # 速度限制
        self.max_velocity = config.safety_limits.get("max_velocity", 120.0)

        # 紧急停止回调
        self.emergency_stop_callbacks: List[Callable] = []

    def add_emergency_callback(self, callback: Callable):
        """添加紧急停止回调"""
        self.emergency_stop_callbacks.append(callback)

    def check_action(self, action: np.ndarray) -> Tuple[bool, str]:
        """
        检查动作是否安全

        Returns:
            (is_safe, message)
        """
        if not self.enabled:
            return True, ""

        # 检查动作范围
        if np.any(np.abs(action) > 1.0):
            return False, f"Action out of range: {action}"

        # 检查速度
        # (需要速度信息)

        return True, ""

    def check_state(self, positions: np.ndarray) -> Tuple[bool, str]:
        """
        检查状态是否安全

        Returns:
            (is_safe, message)
        """
        if not self.enabled:
            return True, ""

        for i, pos in enumerate(positions):
            if i in self.position_limits:
                low, high = self.position_limits[i]
                if pos < low or pos > high:
                    return False, f"Motor {i} position {pos:.2f} out of limits [{low}, {high}]"

        return True, ""

    def emergency_stop(self):
        """执行紧急停止"""
        print("EMERGENCY STOP TRIGGERED!")
        for callback in self.emergency_stop_callbacks:
            try:
                callback()
            except Exception as e:
                print(f"Emergency stop callback failed: {e}")

        self.enabled = False

    def reset(self):
        """重置安全控制器"""
        self.enabled = True


# ============================================================
# 真实电机环境 (Gymnasium 接口)
# ============================================================

class RealMotorEnv(gym.Env):
    """
    真实电机 Gymnasium 环境
    用于 RL 策略在真实硬件上运行
    """

    metadata = {"render_modes": ["human", "rgb_array"]}

    def __init__(
        self,
        config: Optional[HardwareConfig] = None,
        motor_group: Any = None,
        camera: Any = None,
        render_mode: str = "human",
    ):
        super().__init__()

        # 配置
        self.config = config or HardwareConfig()
        self.render_mode = render_mode

        # 硬件接口
        self.motor_group = motor_group
        self.camera = camera
        self.safety_controller = SafetyController(self.config)

        # 状态
        self.current_positions = np.zeros(self.config.num_motors)
        self.current_velocities = np.zeros(self.config.num_motors)
        self.target_positions = np.zeros(self.config.num_motors)

        # 相机处理
        self.vision_module: Optional[VisionObservationModule] = None

        # 控制循环
        self.control_thread: Optional[threading.Thread] = None
        self.is_running = False
        self.control_lock = threading.Lock()

        # 时间
        self.last_update_time = 0.0
        self.dt = 1.0 / self.config.control_frequency

        # Gym 空间
        self._setup_spaces()

        # 连接硬件
        self._connect_hardware()

    def _setup_spaces(self):
        """设置 Gym 空间"""
        # 动作空间: [-1, 1] 归一化动作
        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(self.config.num_motors,),
            dtype=np.float32
        )

        # 观察空间: 电机位置 + 速度 + 目标信息
        obs_dim = self.config.num_motors * 2 + 3  # positions + velocities + target
        self.observation_space = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(obs_dim,),
            dtype=np.float32
        )

    def _connect_hardware(self):
        """连接硬件"""
        # 连接电机
        if self.motor_group is None and RmdGroup is not None:
            try:
                self.motor_group = RmdGroup(
                    port=self.config.serial_port,
                    baudrate=self.config.baudrate,
                    timeout=self.config.serial_timeout,
                )
                print(f"Connected to motor group on {self.config.serial_port}")
            except Exception as e:
                print(f"Failed to connect motor group: {e}")
                print("Running in simulation mode (no actual hardware)")

        # 连接相机
        if self.camera is None and RealSenseCamera is not None:
            try:
                self.camera = RealSenseCamera(
                    width=self.config.camera_width,
                    height=self.config.camera_height,
                    fps=self.config.camera_fps,
                )
                self.camera.start()
                print("Connected to RealSense camera")
            except Exception as e:
                print(f"Failed to connect camera: {e}")

        # 安全控制器设置紧急停止
        if self.motor_group is not None:
            self.safety_controller.add_emergency_callback(self._emergency_stop_hardware)

        # 视觉模块
        if self.camera is not None:
            self.vision_module = VisionObservationModule(
                camera_config={
                    "width": self.config.camera_width,
                    "height": self.config.camera_height,
                    "fps": self.config.camera_fps,
                }
            )

    def _emergency_stop_hardware(self):
        """硬件紧急停止"""
        if self.motor_group is not None:
            try:
                for motor_id in self.config.motor_ids:
                    self.motor_group.stop(motor_id)
                print("Hardware emergency stop executed")
            except Exception as e:
                print(f"Emergency stop failed: {e}")

    def reset(
        self, seed: Optional[int] = None, options: Optional[Dict] = None
    ) -> Tuple[np.ndarray, Dict]:
        """重置环境"""
        super().reset(seed=seed)

        # 重置安全控制器
        self.safety_controller.reset()

        # 重置电机到初始位置
        if self.motor_group is not None:
            initial_positions = options.get("initial_positions") if options else None
            if initial_positions is None:
                initial_positions = [0.0, 0.0, 0.0]

            try:
                for i, pos in enumerate(initial_positions):
                    self.motor_group.set_motor_position(i, pos)
                time.sleep(0.5)  # 等待电机到达位置
            except Exception as e:
                print(f"Failed to reset motor positions: {e}")

        # 更新状态
        self._update_state()

        # 重置目标
        if options and "target_position" in options:
            self.target_position = np.array(options["target_position"])
        else:
            self.target_position = np.array([0.0, 0.0, 0.5])  # 默认目标

        return self._get_obs(), self._get_info()

    def step(
        self, action: np.ndarray
    ) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """执行一步"""
        # 动作检查
        action = np.clip(action, -1, 1)

        is_safe, message = self.safety_controller.check_action(action)
        if not is_safe:
            warnings.warn(f"Unsafe action: {message}")
            action = np.zeros_like(action)

        # 转换动作到实际控制
        target_positions = self._action_to_position(action)

        # 再次检查目标位置
        is_safe, message = self.safety_controller.check_state(target_positions)
        if not is_safe:
            warnings.warn(f"Unsafe target position: {message}")
            self.safety_controller.emergency_stop()
            return self._get_obs(), -10.0, True, False, {"error": message}

        # 发送控制命令
        if self.motor_group is not None:
            try:
                for i, target in enumerate(target_positions):
                    # 计算目标速度
                    delta = target - self.current_positions[i]
                    velocity = np.clip(
                        delta / self.dt,
                        -self.safety_controller.max_velocity,
                        self.safety_controller.max_velocity,
                    )
                    self.motor_group.set_motor_position(i, target, max_speed=velocity)
            except Exception as e:
                print(f"Motor control error: {e}")

        # 更新状态
        self._update_state()

        # 计算奖励
        reward, reward_info = self._compute_reward()

        # 检查终止
        terminated = self._check_termination()
        truncated = False

        return self._get_obs(), reward, terminated, truncated, reward_info

    def _action_to_position(self, action: np.ndarray) -> np.ndarray:
        """
        将归一化动作转换为电机目标位置

        Args:
            action: 归一化动作 [-1, 1]
        Returns:
            target_positions: 电机目标位置 (度)
        """
        # 动作表示位置增量或绝对位置
        # 这里假设动作表示绝对位置
        scale = self.config.action_scale

        # 将 [-1, 1] 映射到实际角度范围
        # M0: 前后移动，范围 [-180, 180]
        # M1: 水平旋转，范围 [-135, 135]
        # M2: 垂直旋转，范围 [-135, 135]

        m0_range = (-180, 180)
        m1_range = (-135, 135)
        m2_range = (-135, 135)

        ranges = [m0_range, m1_range, m2_range]

        target_positions = np.zeros(len(action))
        for i in range(len(action)):
            low, high = ranges[i]
            target_positions[i] = low + (action[i] + 1) / 2 * (high - low)
            target_positions[i] *= scale

        return target_positions

    def _update_state(self):
        """更新电机状态"""
        if self.motor_group is not None:
            try:
                positions = self.motor_group.get_angles()
                self.current_positions = np.array(positions)

                # 估计速度 (简化)
                current_time = time.time()
                dt = current_time - self.last_update_time
                if dt > 0:
                    self.current_velocities = (
                        self.current_positions - self.target_positions
                    ) / dt
                self.target_positions = self.current_positions.copy()
                self.last_update_time = current_time

            except Exception as e:
                print(f"Failed to update motor state: {e}")
                # 使用上次状态
                self.current_positions = self.target_positions.copy()

        # 更新相机
        if self.camera is not None:
            self.camera.update()

    def _get_obs(self) -> np.ndarray:
        """获取观察"""
        # 电机状态
        state = np.concatenate([
            self.current_positions,
            self.current_velocities,
        ])

        # 添加目标信息
        target_obs = np.array([
            self.target_position[0] if hasattr(self, "target_position") else 0.0,
            self.target_position[1] if hasattr(self, "target_position") else 0.0,
            self.target_position[2] if hasattr(self, "target_position") else 0.5,
        ])

        return np.concatenate([state, target_obs]).astype(np.float32)

    def _get_info(self) -> Dict:
        """获取额外信息"""
        return {
            "positions": self.current_positions.tolist(),
            "velocities": self.current_velocities.tolist(),
            "target_position": self.target_position.tolist() if hasattr(self, "target_position") else None,
        }

    def _compute_reward(self) -> Tuple[float, Dict]:
        """计算奖励"""
        # 距离奖励
        robot_pos = np.array([0, 0, 0])  # 简化
        distance = np.linalg.norm(self.target_position - robot_pos)

        reward = -distance

        return reward, {"distance": distance}

    def _check_termination(self) -> bool:
        """检查是否终止"""
        # 距离太远
        robot_pos = np.array([0, 0, 0])
        distance = np.linalg.norm(self.target_position - robot_pos)
        if distance > 2.0:
            return True

        return False

    def render(self) -> Optional[np.ndarray]:
        """渲染环境"""
        if self.render_mode == "rgb_array" and self.camera is not None:
            frame = self.camera.read()
            if frame is not None:
                return frame.rgb

        return None

    def close(self):
        """关闭环境"""
        self.is_running = False

        if self.control_thread is not None:
            self.control_thread.join(timeout=1.0)

        # 停止电机
        if self.motor_group is not None:
            try:
                for motor_id in self.config.motor_ids:
                    self.motor_group.stop(motor_id)
            except:
                pass

        # 停止相机
        if self.camera is not None:
            self.camera.stop()

    def start_control_loop(self):
        """启动独立控制循环"""
        if self.is_running:
            return

        self.is_running = True
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()

    def _control_loop(self):
        """控制循环 (独立线程)"""
        rate = 1.0 / self.config.control_frequency

        while self.is_running:
            loop_start = time.time()

            # 更新状态
            self._update_state()

            # 计算下一步的睡眠时间
            elapsed = time.time() - loop_start
            sleep_time = max(0, rate - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def set_policy(self, policy: Any):
        """设置 RL 策略"""
        self.policy = policy

    def run_policy(self, deterministic: bool = True, max_steps: int = 1000):
        """
        运行 RL 策略

        Args:
            deterministic: 是否使用确定性策略
            max_steps: 最大步数
        """
        if not hasattr(self, "policy"):
            raise ValueError("No policy set. Call set_policy() first.")

        obs, _ = self.reset()
        total_reward = 0.0

        for step in range(max_steps):
            # 获取动作
            action, _ = self.policy.predict(obs, deterministic=deterministic)

            # 执行
            obs, reward, terminated, truncated, info = self.step(action)
            total_reward += reward

            print(f"Step {step}: reward={reward:.3f}, pos={info.get('positions')}")

            if terminated or truncated:
                print(f"Episode ended at step {step}. Total reward: {total_reward:.2f}")
                break

        return total_reward


# ============================================================
# 离线推理模式
# ============================================================

class OfflineInference:
    """
    离线推理器
    加载训练好的策略并在真实硬件上运行
    """

    def __init__(
        self,
        model_path: str,
        hardware_config: Optional[HardwareConfig] = None,
    ):
        self.model_path = model_path
        self.config = hardware_config or HardwareConfig()

        # 加载策略
        self.policy = self._load_policy()

        # 创建环境
        self.env = RealMotorEnv(config=self.config)

    def _load_policy(self):
        """加载策略"""
        from training.rl_trainer import load_trained_model

        # 从路径推断算法
        if "ppo" in self.model_path.lower():
            algo = "PPO"
        elif "sac" in self.model_path.lower():
            algo = "SAC"
        elif "td3" in self.model_path.lower():
            algo = "TD3"
        else:
            algo = "PPO"

        policy = load_trained_model(self.model_path, algorithm=algo)
        return policy

    def run(
        self,
        n_episodes: int = 1,
        max_steps_per_episode: int = 1000,
        render: bool = True,
    ):
        """
        运行策略

        Args:
            n_episodes: 运行 episode 数
            max_steps_per_episode: 每个 episode 最大步数
            render: 是否渲染
        """
        print(f"\n{'='*60}")
        print(f"Running policy from {self.model_path}")
        print(f"{'='*60}\n")

        episode_rewards = []
        episode_successes = []

        for episode in range(n_episodes):
            print(f"\n--- Episode {episode + 1}/{n_episodes} ---")

            obs, _ = self.env.reset()
            episode_reward = 0.0
            episode_success = False

            for step in range(max_steps_per_episode):
                # 预测动作
                action, _ = self.policy.predict(obs, deterministic=True)

                # 执行
                obs, reward, terminated, truncated, info = self.env.step(action)
                episode_reward += reward

                # 检查成功
                if info.get("is_success"):
                    episode_success = True

                # 渲染
                if render:
                    frame = self.env.render()
                    if frame is not None:
                        # 可以保存帧或显示
                        pass

                # 打印进度
                if step % 60 == 0:
                    print(f"  Step {step}: reward={reward:.3f}, pos={info.get('positions')}")

                if terminated or truncated:
                    break

            episode_rewards.append(episode_reward)
            episode_successes.append(episode_success)

            print(f"Episode {episode + 1}: reward={episode_reward:.2f}, success={episode_success}")

        # 总结
        print(f"\n{'='*60}")
        print("Summary:")
        print(f"  Mean reward: {np.mean(episode_rewards):.2f} +/- {np.std(episode_rewards):.2f}")
        print(f"  Success rate: {np.mean(episode_successes):.2%}")
        print(f"{'='*60}\n")

        return {
            "mean_reward": np.mean(episode_rewards),
            "std_reward": np.std(episode_rewards),
            "success_rate": np.mean(episode_successes),
        }

    def close(self):
        """关闭"""
        self.env.close()


# ============================================================
# 主函数 - 部署示例
# ============================================================

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Deploy RL policy on real hardware")
    parser.add_argument("--model", type=str, required=True, help="Path to trained model")
    parser.add_argument("--port", type=str, default="COM3", help="Serial port for motors")
    parser.add_argument("--episodes", type=int, default=1, help="Number of episodes")
    parser.add_argument("--render", action="store_true", help="Render environment")

    args = parser.parse_args()

    # 配置
    config = HardwareConfig(
        serial_port=args.port,
        baudrate=115200,
        control_frequency=60.0,
    )

    # 创建离线推理器
    inference = OfflineInference(
        model_path=args.model,
        hardware_config=config,
    )

    # 运行
    results = inference.run(
        n_episodes=args.episodes,
        render=args.render,
    )

    # 关闭
    inference.close()
