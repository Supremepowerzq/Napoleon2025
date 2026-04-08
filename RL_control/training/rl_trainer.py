"""
深度强化学习训练框架
支持 PPO、SAC、TD3 等算法

功能:
1. 基于 Stable-Baselines3 的 RL 训练
2. 自定义策略网络 (支持视觉输入)
3. 训练监控和可视化
4. 最佳实践检查点保存
5. 评估和推理
"""

import os
import gymnasium as gym
import numpy as np
from typing import Any, Dict, Optional, Tuple, Type, Callable, List
from dataclasses import dataclass, field
from pathlib import Path
import torch
import torch.nn as nn
from collections import OrderedDict

# Stable-Baselines3
from stable_baselines3 import PPO, SAC, TD3, DDPG
from stable_baselines3.common.callbacks import (
    BaseCallback,
    CallbackList,
    CheckpointCallback,
    EvalCallback,
)
from stable_baselines3.common.vec_env import (
    DummyVecEnv,
    SubprocVecEnv,
    VecNormalize,
)
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.preprocessing import is_image_space
from stable_baselines3.common.torch_layers import (
    BaseFeaturesExtractor,
    NatureCNN,
)
from stable_baselines3.common.policies import (
    ActorCriticPolicy,
    MultiInputActorCriticPolicy,
)

# 图像处理
try:
    from PIL import Image
except ImportError:
    Image = None

from config.rl_config import RL_CONFIG, EXPERIMENT_CONFIG


# ============================================================
# 自定义策略网络
# ============================================================

class MotorActorCriticPolicy(ActorCriticPolicy):
    """
    自定义 Actor-Critic 策略网络
    支持多模态输入 (状态 + 视觉)
    """

    def __init__(
        self,
        *args,
        vision_encoder: Optional[nn.Module] = None,
        use_vision: bool = False,
        **kwargs
    ):
        self.use_vision = use_vision
        self.vision_encoder = vision_encoder
        super().__init__(*args, **kwargs)

    def _build_mlp_extractor(self) -> None:
        """构建 MLP 特征提取器"""
        super()._build_mlp_extractor()

        # 如果使用视觉输入，扩展 MLP 输入维度
        if self.use_vision and self.vision_encoder is not None:
            vision_dim = getattr(self.vision_encoder, 'feature_dim', 256)
            # 保存视觉编码器用于前向传播
            self._vision_encoder = self.vision_encoder

    @staticmethod
    def _dummy_schedule(progress: float) -> float:
        """虚拟调度函数"""
        return 1.0


class MultiModalPolicy(MultiInputActorCriticPolicy):
    """
    多模态输入策略
    支持状态、视觉、深度图等多种输入
    """

    def __init__(
        self,
        *args,
        vision_encoder: Optional[nn.Module] = None,
        use_vision: bool = False,
        **kwargs
    ):
        self.use_vision = use_vision
        self.vision_encoder = vision_encoder
        super().__init__(*args, **kwargs)

        # 存储视觉编码器
        if vision_encoder is not None:
            self.vision_encoder = vision_encoder


class VisionFeatureExtractor(BaseFeaturesExtractor):
    """
    视觉特征提取器
    用于从图像中提取特征
    """

    def __init__(
        self,
        observation_space: gym.Space,
        features_dim: int = 256,
        backbone: str = "resnet18",
    ):
        super().__init__(observation_space, features_dim)

        # 图像空间检测
        if is_image_space(observation_space):
            n_input_channels = observation_space.shape[-1] if len(observation_space.shape) == 3 else observation_space.shape[0]
            self.cnn = NatureCNN(
                observation_space,
                features_dim,
            )
        else:
            # 非图像空间，使用线性层
            self.cnn = nn.Sequential(
                nn.Linear(np.prod(observation_space.shape), features_dim),
                nn.ReLU(),
            )

    def forward(self, observations: np.ndarray) -> torch.Tensor:
        """
        Args:
            observations: 观测 (可以是图像或特征向量)
        Returns:
            features: 特征向量
        """
        if isinstance(observations, np.ndarray):
            observations = torch.FloatTensor(observations)

        return self.cnn(observations)


# ============================================================
# 训练回调
# ============================================================

class TrainingMetricsCallback(BaseCallback):
    """
    训练指标回调
    记录奖励、损失、熵等指标到 TensorBoard
    """

    def __init__(
        self,
        verbose: int = 0,
        log_freq: int = 100,
    ):
        super().__init__(verbose)
        self.log_freq = log_freq
        self.episode_rewards: List[float] = []
        self.episode_lengths: List[int] = []

    def _on_step(self) -> bool:
        """每步调用"""
        # 记录奖励
        if len(self.model.ep_info_buffer) > 0:
            for info in self.model.ep_info_buffer:
                if "r" in info:
                    self.episode_rewards.append(info["r"])
                if "l" in info:
                    self.episode_lengths.append(int(info["l"]))

        return True

    def _on_rollout_end(self) -> None:
        """每个 rollout 结束时调用"""
        if len(self.episode_rewards) > 0:
            mean_reward = np.mean(self.episode_rewards[-100:])
            mean_length = np.mean(self.episode_lengths[-100:])

            self.logger.record("rollout/ep_rew_mean", mean_reward)
            self.logger.record("rollout/ep_len_mean", mean_length)

        self.logger.record("time/n_updates", self.num_timesteps, exclude="tensorboard")

        self.episode_rewards = []
        self.episode_lengths = []


class EarlyStoppingCallback(BaseCallback):
    """
    早停回调
    当性能不再提升时停止训练
    """

    def __init__(
        self,
        eval_freq: int = 5000,
        n_eval_episodes: int = 5,
        patience: int = 10,
        threshold: float = 0.01,
        verbose: int = 1,
    ):
        super().__init__(verbose)
        self.eval_freq = eval_freq
        self.n_eval_episodes = n_eval_episodes
        self.patience = patience
        self.threshold = threshold

        self.best_mean_reward = -np.inf
        self.wait = 0

    def _on_step(self) -> bool:
        """每步调用"""
        if self.n_calls % self.eval_freq == 0:
            # 评估
            eval_env = self.training_env
            episode_rewards = []

            for _ in range(self.n_eval_episodes):
                obs, _ = eval_env.reset()
                done = False
                episode_reward = 0.0

                while not done:
                    action, _ = self.model.predict(obs, deterministic=True)
                    obs, reward, terminated, truncated, _ = eval_env.step(action)
                    episode_reward += reward
                    done = terminated or truncated

                episode_rewards.append(episode_reward)

            mean_reward = np.mean(episode_rewards)

            if self.verbose > 0:
                print(f"Eval mean reward: {mean_reward:.2f}")

            # 检查是否改进
            if mean_reward > self.best_mean_reward + self.threshold:
                self.best_mean_reward = mean_reward
                self.wait = 0
            else:
                self.wait += 1

            # 早停检查
            if self.wait >= self.patience:
                if self.verbose > 0:
                    print(f"Early stopping at step {self.num_timesteps}")
                return False

        return True


class VideoRecordingCallback(BaseCallback):
    """
    视频录制回调
    定期录制环境渲染视频
    """

    def __init__(
        self,
        video_freq: int = 10000,
        video_length: int = 500,
        video_prefix: str = "rl_video",
        verbose: int = 0,
    ):
        super().__init__(verbose)
        self.video_freq = video_freq
        self.video_length = video_length
        self.video_prefix = video_prefix
        self.recording = False
        self.video_frames: List[np.ndarray] = []

    def _on_step(self) -> bool:
        """每步调用"""
        if self.n_calls % self.video_freq == 0 and not self.recording:
            self.recording = True
            self.video_frames = []

        if self.recording:
            # 录制帧
            if hasattr(self.training_env, 'render'):
                frame = self.training_env.render()
                if frame is not None:
                    self.video_frames.append(frame)

            # 停止录制
            if len(self.video_frames) >= self.video_length:
                self._save_video()
                self.recording = False

        return True

    def _save_video(self):
        """保存视频"""
        if len(self.video_frames) == 0:
            return

        save_dir = Path(self.logger.log_dir) / "videos"
        save_dir.mkdir(exist_ok=True)

        video_path = save_dir / f"{self.video_prefix}_{self.num_timesteps}.mp4"

        try:
            # 使用 OpenCV 保存视频
            import cv2
            if len(self.video_frames[0].shape) == 3:
                h, w = self.video_frames[0].shape[:2]
            else:
                h, w = self.video_frames[0].shape

            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter(str(video_path), fourcc, 30, (w, h))

            for frame in self.video_frames:
                if frame.ndim == 2:
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                elif frame.shape[-1] == 1:
                    frame = cv2.cvtColor(frame.squeeze(), cv2.COLOR_GRAY2BGR)
                out.write(frame)

            out.release()

            if self.verbose > 0:
                print(f"Video saved to {video_path}")

        except Exception as e:
            if self.verbose > 0:
                print(f"Failed to save video: {e}")


# ============================================================
# 训练器
# ============================================================

@dataclass
class TrainingConfig:
    """训练配置"""
    algorithm: str = "PPO"
    total_timesteps: int = 1_000_000
    eval_freq: int = 5000
    save_freq: int = 10000
    log_freq: int = 100
    n_eval_episodes: int = 10
    n_eval_envs: int = 1
    seed: int = 42
    device: str = "auto"


class RLTrainer:
    """
    强化学习训练器
    封装训练流程，提供简单易用的接口
    """

    def __init__(
        self,
        env: gym.Env,
        config: Optional[TrainingConfig] = None,
        log_dir: str = "./logs",
        checkpoint_dir: str = "./checkpoints",
    ):
        self.env = env
        self.config = config or TrainingConfig()
        self.log_dir = Path(log_dir)
        self.checkpoint_dir = Path(checkpoint_dir)

        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.checkpoint_dir.mkdir(parents=True, exist_ok=True)

        self.model: Optional[Any] = None
        self.best_model: Optional[Any] = None

        # 随机种子
        self._set_seed(self.config.seed)

        # 设备
        if self.config.device == "auto":
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
        else:
            self.device = self.config.device

    def _set_seed(self, seed: int):
        """设置随机种子"""
        np.random.seed(seed)
        torch.manual_seed(seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed(seed)

    def _create_model(self, policy: str = "MlpPolicy", **policy_kwargs) -> Any:
        """创建 RL 模型"""
        algo_config = RL_CONFIG.get(self.config.algorithm.lower(), {})

        # 创建模型
        # PPO + MlpPolicy 在 CPU 上训练更快（GPU 仅对 CNN 策略有意义）
        effective_device = "cpu" if (self.config.algorithm == "PPO" and policy == "MlpPolicy") else self.device

        if self.config.algorithm == "PPO":
            model = PPO(
                policy=policy,
                env=self.env,
                learning_rate=algo_config.get("learning_rate", 3e-4),
                n_steps=algo_config.get("n_steps", 2048),
                batch_size=algo_config.get("batch_size", 64),
                n_epochs=algo_config.get("n_epochs", 10),
                gamma=algo_config.get("gamma", 0.99),
                gae_lambda=algo_config.get("gae_lambda", 0.95),
                clip_range=algo_config.get("clip_range", 0.2),
                ent_coef=algo_config.get("ent_coef", 0.01),
                vf_coef=algo_config.get("vf_coef", 0.5),
                max_grad_norm=algo_config.get("max_grad_norm", 0.5),
                tensorboard_log=str(self.log_dir / "tensorboard"),
                device=effective_device,
                policy_kwargs=policy_kwargs,
            )
        elif self.config.algorithm == "SAC":
            model = SAC(
                policy=policy,
                env=self.env,
                learning_rate=algo_config.get("learning_rate", 3e-4),
                buffer_size=algo_config.get("buffer_size", 100000),
                learning_starts=algo_config.get("learning_starts", 1000),
                batch_size=algo_config.get("batch_size", 256),
                tau=algo_config.get("tau", 0.005),
                gamma=algo_config.get("gamma", 0.99),
                ent_coef=algo_config.get("ent_coef", "auto"),
                tensorboard_log=str(self.log_dir / "tensorboard"),
                device=self.device,
                policy_kwargs=policy_kwargs,
            )
        elif self.config.algorithm == "TD3":
            model = TD3(
                policy=policy,
                env=self.env,
                learning_rate=algo_config.get("learning_rate", 3e-4),
                buffer_size=algo_config.get("buffer_size", 100000),
                learning_starts=algo_config.get("learning_starts", 1000),
                batch_size=algo_config.get("batch_size", 256),
                tau=algo_config.get("tau", 0.005),
                gamma=algo_config.get("gamma", 0.99),
                tensorboard_log=str(self.log_dir / "tensorboard"),
                device=self.device,
                policy_kwargs=policy_kwargs,
            )
        else:
            raise ValueError(f"Unsupported algorithm: {self.config.algorithm}")

        return model

    def train(
        self,
        policy: str = "MlpPolicy",
        load_path: Optional[str] = None,
        **policy_kwargs
    ) -> Any:
        """
        训练模型

        Args:
            policy: 策略类型 ("MlpPolicy", "CnnPolicy", "MultiInputPolicy")
            load_path: 加载已有模型的路径
            **policy_kwargs: 传递给策略网络的额外参数

        Returns:
            训练好的模型
        """
        # 加载已有模型或创建新模型
        if load_path and os.path.exists(load_path):
            print(f"Loading model from {load_path}")
            self.model = self._load_model(load_path)
        else:
            print(f"Creating new {self.config.algorithm} model")
            self.model = self._create_model(policy=policy, **policy_kwargs)

        # 创建回调
        callbacks = self._create_callbacks()

        # 开始训练
        print(f"Starting training for {self.config.total_timesteps} timesteps...")
        self.model.learn(
            total_timesteps=self.config.total_timesteps,
            callback=callbacks,
            log_interval=self.config.log_freq,
            progress_bar=True,
        )

        print("Training completed!")
        return self.model

    def _create_callbacks(self) -> CallbackList:
        """创建训练回调"""
        callbacks = []

        # 检查点回调
        checkpoint_callback = CheckpointCallback(
            save_freq=self.config.save_freq,
            save_path=str(self.checkpoint_dir),
            name_prefix="rl_model",
            save_replay_buffer=False,
            save_vecnormalize=True,
        )
        callbacks.append(checkpoint_callback)

        # 评估回调
        if self.config.n_eval_episodes > 0:
            eval_env = self.env
            eval_callback = EvalCallback(
                eval_env,
                best_model_save_path=str(self.checkpoint_dir / "best_model"),
                log_path=str(self.log_dir / "eval"),
                eval_freq=self.config.eval_freq,
                n_eval_episodes=self.config.n_eval_episodes,
                deterministic=True,
                render=False,
            )
            callbacks.append(eval_callback)

        # 训练指标回调
        metrics_callback = TrainingMetricsCallback(
            log_freq=self.config.log_freq
        )
        callbacks.append(metrics_callback)

        return CallbackList(callbacks)

    def evaluate(
        self,
        n_episodes: int = 10,
        deterministic: bool = True,
        render: bool = False,
    ) -> Dict[str, float]:
        """
        评估模型

        Args:
            n_episodes: 评估 episode 数
            deterministic: 是否使用确定性策略
            render: 是否渲染环境

        Returns:
            评估统计信息
        """
        if self.model is None:
            raise ValueError("Model not trained yet!")

        episode_rewards = []
        episode_lengths = []
        episode_successes = []

        for i in range(n_episodes):
            obs, _ = self.env.reset()
            done = False
            episode_reward = 0.0
            episode_length = 0
            episode_success = False

            while not done:
                action, _ = self.model.predict(obs, deterministic=deterministic)
                obs, reward, terminated, truncated, info = self.env.step(action)

                episode_reward += reward
                episode_length += 1
                done = terminated or truncated

                # 检查成功标志
                if "is_success" in info and info["is_success"]:
                    episode_success = True

                if render:
                    self.env.render()

            episode_rewards.append(episode_reward)
            episode_lengths.append(episode_length)
            episode_successes.append(episode_success)

            print(f"Episode {i+1}/{n_episodes}: reward={episode_reward:.2f}, length={episode_length}, success={episode_success}")

        results = {
            "mean_reward": np.mean(episode_rewards),
            "std_reward": np.std(episode_rewards),
            "mean_length": np.mean(episode_lengths),
            "std_length": np.std(episode_lengths),
            "success_rate": np.mean(episode_successes),
        }

        return results

    def _load_model(self, path: str) -> Any:
        """加载模型"""
        if self.config.algorithm == "PPO":
            return PPO.load(path, env=self.env, device=self.device)
        elif self.config.algorithm == "SAC":
            return SAC.load(path, env=self.env, device=self.device)
        elif self.config.algorithm == "TD3":
            return TD3.load(path, env=self.env, device=self.device)
        else:
            raise ValueError(f"Cannot load model for algorithm: {self.config.algorithm}")

    def save(self, path: str):
        """保存模型"""
        if self.model is not None:
            self.model.save(path)
            print(f"Model saved to {path}")

    def predict(self, obs: np.ndarray, deterministic: bool = True) -> Tuple[np.ndarray, None]:
        """使用训练好的模型预测动作"""
        if self.model is None:
            raise ValueError("Model not trained yet!")
        return self.model.predict(obs, deterministic=deterministic)


# ============================================================
# 多环境并行训练
# ============================================================

def make_vec_env(
    env_id: str,
    n_envs: int = 4,
    seed: int = 0,
    vec_env_cls: Type = SubprocVecEnv,
    vec_env_kwargs: Optional[Dict] = None,
) -> VecNormalize:
    """
    创建向量化环境

    Args:
        env_id: 环境 ID 或环境实例
        n_envs: 并行环境数
        seed: 随机种子
        vec_env_cls: 向量化环境类
        vec_env_kwargs: 向量化环境参数

    Returns:
        规范化的向量化环境
    """
    def make_env(rank: int) -> Callable:
        def _init():
            if isinstance(env_id, str):
                env = gym.make(env_id)
            else:
                env = env_id

            env.reset(seed=seed + rank)
            return env
        return _init

    if vec_env_kwargs is None:
        vec_env_kwargs = {}

    env_fns = [make_env(i) for i in range(n_envs)]
    vec_env = vec_env_cls(env_fns, **vec_env_kwargs)

    # 自动规范化
    vec_normalize = VecNormalize(vec_env, norm_obs=True, norm_reward=True)

    return vec_normalize


# ============================================================
# 便捷函数
# ============================================================

def load_trained_model(
    model_path: str,
    algorithm: str = "PPO",
    env: Optional[gym.Env] = None,
) -> Any:
    """
    加载训练好的模型

    Args:
        model_path: 模型路径
        algorithm: 算法类型
        env: 环境 (可选)

    Returns:
        加载的模型
    """
    if algorithm == "PPO":
        return PPO.load(model_path, env=env)
    elif algorithm == "SAC":
        return SAC.load(model_path, env=env)
    elif algorithm == "TD3":
        return TD3.load(model_path, env=env)
    elif algorithm == "DDPG":
        return DDPG.load(model_path, env=env)
    else:
        raise ValueError(f"Unsupported algorithm: {algorithm}")


def record_video(
    env: gym.Env,
    model: Any,
    video_path: str,
    video_length: int = 500,
    deterministic: bool = True,
) -> bool:
    """
    录制环境视频

    Args:
        env: 环境
        model: 训练好的模型
        video_path: 视频保存路径
        video_length: 视频长度 (帧数)
        deterministic: 是否使用确定性策略

    Returns:
        是否成功
    """
    try:
        import cv2

        obs, _ = env.reset()
        frames = []

        for _ in range(video_length):
            action, _ = model.predict(obs, deterministic=deterministic)
            obs, _, terminated, truncated, _ = env.step(action)

            frame = env.render()
            if frame is not None:
                frames.append(frame)

            if terminated or truncated:
                obs, _ = env.reset()

        if len(frames) == 0:
            return False

        # 保存视频
        h, w = frames[0].shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(video_path, fourcc, 30, (w, h))

        for frame in frames:
            if frame.ndim == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            out.write(frame)

        out.release()
        print(f"Video saved to {video_path}")
        return True

    except Exception as e:
        print(f"Failed to record video: {e}")
        return False


# ============================================================
# 主函数 - 训练示例
# ============================================================

if __name__ == "__main__":
    # 示例: 训练 PPO 模型

    # 1. 创建环境
    from envs.pybullet_motor_env import PyBulletMotorEnv

    env = PyBulletMotorEnv(render_mode="human")
    env = Monitor(env)  # 包装环境以记录训练指标

    # 2. 配置训练
    config = TrainingConfig(
        algorithm="PPO",
        total_timesteps=100_000,  # 演示用，实际需要更多
        eval_freq=5000,
        save_freq=10000,
        seed=42,
    )

    # 3. 创建训练器
    trainer = RLTrainer(
        env=env,
        config=config,
        log_dir="./logs/ppo_motor",
        checkpoint_dir="./checkpoints/ppo_motor",
    )

    # 4. 训练
    model = trainer.train(policy="MlpPolicy")

    # 5. 评估
    results = trainer.evaluate(n_episodes=10)
    print("\n=== Evaluation Results ===")
    for k, v in results.items():
        print(f"{k}: {v}")

    # 6. 保存最终模型
    trainer.save("./checkpoints/ppo_motor/final_model")

    # 7. 关闭环境
    env.close()
