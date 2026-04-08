"""
Sim-to-Real 迁移模块
将仿真环境训练的策略迁移到真实硬件

功能:
1. 域随机化 (Domain Randomization)
2. 域适应 (Domain Adaptation)
3. 系统辨识 (System Identification)
4. 现实差距检测
5. 在线微调
"""

import os
import numpy as np
import torch
import torch.nn as nn
from typing import Dict, List, Tuple, Optional, Any, Callable
from dataclasses import dataclass, field
from pathlib import Path
import json

from config.rl_config import SIM2REAL_CONFIG, SIM_CONFIG


# ============================================================
# 域随机化
# ============================================================

class DomainRandomizer:
    """
    域随机化器
    在仿真中随机化物理和视觉参数，增加策略的鲁棒性
    """

    def __init__(
        self,
        config: Optional[Dict] = None,
        seed: Optional[int] = None,
    ):
        self.config = config or SIM2REAL_CONFIG.get("domain_randomization", {})
        self.rng = np.random.default_rng(seed)

        # 随机化参数
        self.physics_params = self._init_physics_params()
        self.visual_params = self._init_visual_params()
        self.dynamics_params = self._init_dynamics_params()

    def _init_physics_params(self) -> Dict:
        """初始化物理参数"""
        physics = self.config.get("physics", {})
        return {
            "friction": self._sample_param(physics.get("friction", {})),
            "mass": self._sample_param(physics.get("mass", {})),
            "motor_constant": self._sample_param(physics.get("motor_constant", {})),
        }

    def _init_visual_params(self) -> Dict:
        """初始化视觉参数"""
        visual = self.config.get("visual", {})
        return {
            "light_direction": self.rng.uniform(-1, 1, size=3) if visual.get("light_direction") else np.zeros(3),
            "background_noise": 0.0,
            "camera_noise_std": 0.01,
        }

    def _init_dynamics_params(self) -> Dict:
        """初始化动力学参数"""
        dynamics = self.config.get("dynamics", {})
        latency_options = dynamics.get("latency", [0.0, 0.005, 0.01, 0.02])
        deadzone_options = dynamics.get("deadzone", [0.0, 0.1, 0.2])

        return {
            "latency": self.rng.choice(latency_options),
            "deadzone": self.rng.choice(deadzone_options),
            "action_delay": 0.0,
        }

    def _sample_param(self, param_config: Dict) -> float:
        """采样参数"""
        mean = param_config.get("mean", 1.0)
        std = param_config.get("std", 0.0)
        return mean + self.rng.normal(0, std)

    def randomize(self) -> Dict:
        """随机化所有参数"""
        self.physics_params = self._init_physics_params()
        self.visual_params = self._init_visual_params()
        self.dynamics_params = self._init_dynamics_params()

        return self.get_all_params()

    def get_all_params(self) -> Dict:
        """获取所有随机化参数"""
        return {
            "physics": self.physics_params,
            "visual": self.visual_params,
            "dynamics": self.dynamics_params,
        }

    def apply_to_env(self, env: Any) -> None:
        """应用随机化参数到仿真环境"""
        params = self.get_all_params()

        if hasattr(env, "set_physics_params"):
            env.set_physics_params(params["physics"])

        if hasattr(env, "set_visual_params"):
            env.set_visual_params(params["visual"])

        if hasattr(env, "set_dynamics_params"):
            env.set_dynamics_params(params["dynamics"])


class ProgressiveDomainRandomizer(DomainRandomizer):
    """
    渐进式域随机化
    随着训练进行，逐步增加随机化范围
    """

    def __init__(
        self,
        config: Optional[Dict] = None,
        initial_std_factor: float = 0.5,
        final_std_factor: float = 1.0,
        annealing_steps: int = 500000,
        seed: Optional[int] = None,
    ):
        super().__init__(config, seed)
        self.initial_std_factor = initial_std_factor
        self.final_std_factor = final_std_factor
        self.annealing_steps = annealing_steps
        self.current_step = 0

    def randomize(self, step: int) -> Dict:
        """根据当前步数随机化参数"""
        self.current_step = step

        # 计算进度 (0 到 1)
        progress = min(step / self.annealing_steps, 1.0)

        # 插值随机化强度
        std_factor = self.initial_std_factor + (self.final_std_factor - self.initial_std_factor) * progress

        # 重新采样参数
        physics = self.config.get("physics", {})
        self.physics_params = {
            "friction": self._sample_param_with_factor(physics.get("friction", {}), std_factor),
            "mass": self._sample_param_with_factor(physics.get("mass", {}), std_factor),
            "motor_constant": self._sample_param_with_factor(physics.get("motor_constant", {}), std_factor),
        }

        return self.get_all_params()

    def _sample_param_with_factor(self, param_config: Dict, factor: float) -> float:
        """根据因子采样参数"""
        mean = param_config.get("mean", 1.0)
        std = param_config.get("std", 0.0) * factor
        return mean + self.rng.normal(0, std)


# ============================================================
# 系统辨识
# ============================================================

class SystemIdentifier:
    """
    系统辨识器
    从真实数据中学习仿真模型和真实模型之间的差异
    """

    def __init__(
        self,
        state_dim: int = 9,
        action_dim: int = 3,
        model_hidden_dim: int = 128,
        device: str = "cuda" if torch.cuda.is_available() else "cpu",
    ):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.device = device

        # 残差网络 (学习 sim-to-real 差异)
        self.residual_model = ResidualNetwork(
            state_dim=state_dim,
            action_dim=action_dim,
            hidden_dim=model_hidden_dim,
        ).to(device)

        self.optimizer = torch.optim.Adam(
            self.residual_model.parameters(),
            lr=1e-3,
        )

        # 数据缓冲区
        self.real_data_buffer: List[Tuple[np.ndarray, np.ndarray, np.ndarray]] = []
        self.sim_data_buffer: List[Tuple[np.ndarray, np.ndarray, np.ndarray]] = []

    def add_real_trajectory(
        self,
        states: np.ndarray,
        actions: np.ndarray,
        next_states: np.ndarray,
    ):
        """添加真实轨迹数据"""
        self.real_data_buffer.append((states, actions, next_states))

    def add_sim_trajectory(
        self,
        states: np.ndarray,
        actions: np.ndarray,
        next_states: np.ndarray,
    ):
        """添加仿真轨迹数据"""
        self.sim_data_buffer.append((states, actions, next_states))

    def fit(self, epochs: int = 100, batch_size: int = 64):
        """
        训练残差模型

        loss = || (next_state_real - state_real) - (next_state_sim - state_sim) - residual(s,a) ||^2
        """
        if len(self.real_data_buffer) == 0:
            print("Warning: No real data available for system identification")
            return

        # 合并数据
        real_states = np.concatenate([t[0] for t in self.real_data_buffer])
        real_actions = np.concatenate([t[1] for t in self.real_data_buffer])
        real_next_states = np.concatenate([t[2] for t in self.real_data_buffer])

        sim_states = np.concatenate([t[0] for t in self.sim_data_buffer]) if self.sim_data_buffer else None
        sim_actions = np.concatenate([t[1] for t in self.sim_data_buffer]) if self.sim_data_buffer else None
        sim_next_states = np.concatenate([t[2] for t in self.sim_data_buffer]) if self.sim_data_buffer else None

        real_deltas = real_next_states - real_states

        if sim_states is not None:
            sim_deltas = sim_next_states - sim_states
        else:
            sim_deltas = np.zeros_like(real_deltas)

        # 训练循环
        for epoch in range(epochs):
            indices = np.random.permutation(len(real_states))
            total_loss = 0.0

            for i in range(0, len(indices), batch_size):
                batch_idx = indices[i:i + batch_size]
                batch_states = torch.FloatTensor(real_states[batch_idx]).to(self.device)
                batch_actions = torch.FloatTensor(real_actions[batch_idx]).to(self.device)
                batch_real_deltas = torch.FloatTensor(real_deltas[batch_idx]).to(self.device)
                batch_sim_deltas = torch.FloatTensor(sim_deltas[batch_idx]).to(self.device) if sim_deltas is not None else None

                # 预测残差
                predicted_residual = self.residual_model(batch_states, batch_actions)

                # 真实差异
                real_deltas_combined = batch_real_deltas - batch_sim_deltas if batch_sim_deltas is not None else batch_real_deltas

                # 损失
                loss = torch.mean((predicted_residual - real_deltas_combined) ** 2)

                # 反向传播
                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()

                total_loss += loss.item()

            if (epoch + 1) % 10 == 0:
                print(f"Epoch {epoch+1}/{epochs}, Loss: {total_loss / (len(indices) / batch_size):.6f}")

    def predict_residual(
        self,
        state: np.ndarray,
        action: np.ndarray,
    ) -> np.ndarray:
        """预测 sim-to-real 残差"""
        self.residual_model.eval()
        with torch.no_grad():
            state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
            action_tensor = torch.FloatTensor(action).unsqueeze(0).to(self.device)
            residual = self.residual_model(state_tensor, action_tensor)
        return residual.squeeze(0).cpu().numpy()

    def save(self, path: str):
        """保存模型"""
        torch.save(self.residual_model.state_dict(), path)

    def load(self, path: str):
        """加载模型"""
        self.residual_model.load_state_dict(torch.load(path, map_location=self.device))


class ResidualNetwork(nn.Module):
    """
    残差网络
    学习仿真和真实环境之间的差异
    """

    def __init__(
        self,
        state_dim: int,
        action_dim: int,
        hidden_dim: int = 128,
    ):
        super().__init__()

        self.network = nn.Sequential(
            nn.Linear(state_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, state_dim),
        )

    def forward(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        """
        Args:
            state: 状态 [B, state_dim]
            action: 动作 [B, action_dim]
        Returns:
            residual: 残差 [B, state_dim]
        """
        concat = torch.cat([state, action], dim=-1)
        return self.network(concat)


# ============================================================
# 现实差距检测
# ============================================================

class RealityGapDetector:
    """
    现实差距检测器
    检测仿真策略在真实环境中的性能差距
    """

    def __init__(
        self,
        sim_env: Any,
        real_env: Any,
        policy: Any,
    ):
        self.sim_env = sim_env
        self.real_env = real_env
        self.policy = policy

    def evaluate_on_sim(self, n_episodes: int = 10) -> Dict[str, float]:
        """在仿真环境中评估"""
        rewards = []
        successes = []

        for _ in range(n_episodes):
            obs, _ = self.sim_env.reset()
            done = False
            episode_reward = 0.0
            episode_success = False

            while not done:
                action, _ = self.policy.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = self.sim_env.step(action)
                episode_reward += reward
                done = terminated or truncated

                if info.get("is_success"):
                    episode_success = True

            rewards.append(episode_reward)
            successes.append(episode_success)

        return {
            "sim_mean_reward": np.mean(rewards),
            "sim_std_reward": np.std(rewards),
            "sim_success_rate": np.mean(successes),
        }

    def evaluate_on_real(self, n_episodes: int = 10) -> Dict[str, float]:
        """在真实环境中评估"""
        rewards = []
        successes = []

        for _ in range(n_episodes):
            obs, _ = self.real_env.reset()
            done = False
            episode_reward = 0.0
            episode_success = False

            while not done:
                action, _ = self.policy.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = self.real_env.step(action)
                episode_reward += reward
                done = terminated or truncated

                if info.get("is_success"):
                    episode_success = True

            rewards.append(episode_reward)
            successes.append(episode_success)

        return {
            "real_mean_reward": np.mean(rewards),
            "real_std_reward": np.std(rewards),
            "real_success_rate": np.mean(successes),
        }

    def compute_gap(self, n_episodes: int = 10) -> Dict:
        """
        计算 sim-to-real 差距

        Returns:
            gap_metrics: 差距指标
        """
        sim_results = self.evaluate_on_sim(n_episodes)
        real_results = self.evaluate_on_real(n_episodes)

        return {
            "reward_gap": sim_results["sim_mean_reward"] - real_results["real_mean_reward"],
            "success_rate_gap": sim_results["sim_success_rate"] - real_results["real_success_rate"],
            "sim_metrics": sim_results,
            "real_metrics": real_results,
        }


# ============================================================
# 在线微调
# ============================================================

class OnlineFineTuner:
    """
    在线微调器
    在真实环境中收集数据并微调策略
    """

    def __init__(
        self,
        policy: Any,
        real_env: Any,
        sim_env: Optional[Any] = None,
        batch_size: int = 64,
        learning_rate: float = 1e-5,
        real_data_ratio: float = 0.2,
        device: str = "cuda" if torch.cuda.is_available() else "cpu",
    ):
        self.policy = policy
        self.real_env = real_env
        self.sim_env = sim_env
        self.batch_size = batch_size
        self.real_data_ratio = real_data_ratio
        self.device = device

        # 数据缓冲区
        self.real_data_buffer: List[Tuple[np.ndarray, np.ndarray, float, np.ndarray]] = []

        # 自适应学习率
        self.optimizer = torch.optim.Adam(
            policy.parameters() if hasattr(policy, 'parameters') else [],
            lr=learning_rate,
        )

        self.loss_fn = nn.MSELoss()

    def collect_real_data(self, n_steps: int = 1000):
        """
        在真实环境中收集数据

        Args:
            n_steps: 收集的步数
        """
        obs, _ = self.real_env.reset()
        episode_buffer = []

        for _ in range(n_steps):
            action, _ = self.policy.predict(obs, deterministic=False)  # 探索
            next_obs, reward, terminated, truncated, info = self.real_env.step(action)

            episode_buffer.append((obs, action, reward, next_obs))

            obs = next_obs
            if terminated or truncated:
                obs, _ = self.real_env.reset()

            # 定期添加数据到缓冲区
            if len(episode_buffer) >= self.batch_size:
                self._add_to_buffer(episode_buffer)
                episode_buffer = []

        # 添加剩余数据
        if episode_buffer:
            self._add_to_buffer(episode_buffer)

        print(f"Collected {len(self.real_data_buffer)} samples from real environment")

    def _add_to_buffer(self, episode_buffer: List):
        """添加 episode 数据到缓冲区"""
        for obs, action, reward, next_obs in episode_buffer:
            self.real_data_buffer.append((obs, action, reward, next_obs))

    def update(self, n_epochs: int = 5):
        """
        使用真实数据微调策略

        Args:
            n_epochs: 训练轮数
        """
        if len(self.real_data_buffer) < self.batch_size:
            print("Not enough real data for training")
            return

        for epoch in range(n_epochs):
            # 采样批次
            indices = np.random.choice(
                len(self.real_data_buffer),
                size=min(self.batch_size, len(self.real_data_buffer)),
                replace=False,
            )

            batch_data = [self.real_data_buffer[i] for i in indices]

            # 提取数据
            states = np.array([d[0] for d in batch_data])
            actions = np.array([d[1] for d in batch_data])

            # 更新策略 (简化版本 - 实际需要根据算法实现)
            # 这里可以集成到 PPO/SAC 等算法的更新步骤中
            # ...

    def train(
        self,
        total_steps: int = 10000,
        eval_freq: int = 1000,
        n_eval_episodes: int = 5,
    ):
        """
        在线训练循环

        Args:
            total_steps: 总训练步数
            eval_freq: 评估频率
            n_eval_episodes: 每次评估的 episode 数
        """
        total_collected = 0
        step = 0

        while total_collected < total_steps:
            # 收集数据
            collect_steps = min(eval_freq, total_steps - total_collected)
            self.collect_real_data(n_steps=collect_steps)
            total_collected += collect_steps
            step += 1

            # 更新策略
            self.update(n_epochs=5)

            # 评估
            if step % 5 == 0:
                results = self._evaluate(n_eval_episodes)
                print(f"Step {step}: Real reward = {results['mean_reward']:.2f} +/- {results['std_reward']:.2f}")

    def _evaluate(self, n_episodes: int) -> Dict[str, float]:
        """评估当前策略"""
        rewards = []

        for _ in range(n_episodes):
            obs, _ = self.real_env.reset()
            done = False
            episode_reward = 0.0

            while not done:
                action, _ = self.policy.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, _ = self.real_env.step(action)
                episode_reward += reward
                done = terminated or truncated

            rewards.append(episode_reward)

        return {
            "mean_reward": np.mean(rewards),
            "std_reward": np.std(rewards),
        }


# ============================================================
# Sim-to-Real 迁移管理器
# ============================================================

class Sim2RealManager:
    """
    Sim-to-Real 迁移管理器
    整合所有迁移组件，提供端到端流程
    """

    def __init__(
        self,
        sim_env: Any,
        real_env: Any,
        sim_config: Optional[Dict] = None,
        sim2real_config: Optional[Dict] = None,
    ):
        self.sim_env = sim_env
        self.real_env = real_env
        self.sim_config = sim_config or SIM_CONFIG
        self.sim2real_config = sim2real_config or SIM2REAL_CONFIG

        # 组件
        self.randomizer = ProgressiveDomainRandomizer(
            config=self.sim2real_config.get("domain_randomization"),
        )
        self.system_identifier: Optional[SystemIdentifier] = None
        self.gap_detector: Optional[RealityGapDetector] = None

        # 训练历史
        self.training_history: List[Dict] = []

    def train_with_dr(
        self,
        policy: Any,
        total_timesteps: int = 1_000_000,
        eval_freq: int = 10000,
        randomization_freq: int = 1000,
    ) -> Any:
        """
        使用域随机化训练策略

        Args:
            policy: RL 策略
            total_timesteps: 总训练步数
            eval_freq: 评估频率
            randomization_freq: 随机化频率

        Returns:
            训练好的策略
        """
        from training.rl_trainer import RLTrainer, TrainingConfig

        config = TrainingConfig(
            algorithm=RL_CONFIG["algorithm"],
            total_timesteps=total_timesteps,
            eval_freq=eval_freq,
        )

        trainer = RLTrainer(
            env=self.sim_env,
            config=config,
        )

        # 训练
        model = trainer.train()

        return model

    def identify_and_adapt(
        self,
        policy: Any,
        real_trajectories: List,
        sim_trajectories: List,
        n_adaptation_steps: int = 1000,
    ) -> Any:
        """
        系统辨识 + 域适应

        Args:
            policy: 从仿真训练的策略
            real_trajectories: 真实轨迹数据
            sim_trajectories: 仿真轨迹数据
            n_adaptation_steps: 适应步数

        Returns:
            适应后的策略
        """
        # 1. 系统辨识
        self.system_identifier = SystemIdentifier()
        for traj in real_trajectories:
            self.system_identifier.add_real_trajectory(*traj)
        for traj in sim_trajectories:
            self.system_identifier.add_sim_trajectory(*traj)

        self.system_identifier.fit(epochs=100)

        # 2. 在线微调
        fine_tuner = OnlineFineTuner(
            policy=policy,
            real_env=self.real_env,
            sim_env=self.sim_env,
        )
        fine_tuner.train(total_steps=n_adaptation_steps)

        return fine_tuner.policy

    def evaluate_gap(
        self,
        policy: Any,
        n_episodes: int = 10,
    ) -> Dict:
        """
        评估 sim-to-real 差距

        Args:
            policy: 要评估的策略
            n_episodes: 评估 episode 数

        Returns:
            差距报告
        """
        if self.gap_detector is None:
            self.gap_detector = RealityGapDetector(
                sim_env=self.sim_env,
                real_env=self.real_env,
                policy=policy,
            )

        gap_report = self.gap_detector.compute_gap(n_episodes=n_episodes)
        self.training_history.append({
            "type": "gap_evaluation",
            "results": gap_report,
        })

        return gap_report

    def full_pipeline(
        self,
        policy: Any,
        n_sim_timesteps: int = 1_000_000,
        n_real_adaptation_steps: int = 5000,
        n_eval_episodes: int = 10,
    ) -> Tuple[Any, Dict]:
        """
        完整 sim-to-real 流程

        Args:
            policy: 初始策略 (可选)
            n_sim_timesteps: 仿真训练步数
            n_real_adaptation_steps: 真实环境适应步数
            n_eval_episodes: 评估 episode 数

        Returns:
            最终策略
            完整报告
        """
        report = {}

        # 阶段 1: 仿真训练
        print("=" * 60)
        print("Stage 1: Simulation Training with Domain Randomization")
        print("=" * 60)
        policy = self.train_with_dr(
            policy=policy,
            total_timesteps=n_sim_timesteps,
        )
        report["sim_training"] = {"status": "completed"}

        # 评估仿真性能
        sim_results = self.gap_detector.evaluate_on_sim(n_eval_episodes)
        report["sim_results"] = sim_results
        print(f"Simulation results: {sim_results}")

        # 阶段 2: 在线适应
        if n_real_adaptation_steps > 0:
            print("\n" + "=" * 60)
            print("Stage 2: Online Adaptation on Real Hardware")
            print("=" * 60)

            fine_tuner = OnlineFineTuner(
                policy=policy,
                real_env=self.real_env,
                sim_env=self.sim_env,
            )

            # 收集真实数据并微调
            fine_tuner.train(total_steps=n_real_adaptation_steps)
            policy = fine_tuner.policy
            report["adaptation"] = {"status": "completed", "steps": n_real_adaptation_steps}

        # 阶段 3: 最终评估
        print("\n" + "=" * 60)
        print("Stage 3: Final Evaluation")
        print("=" * 60)
        gap_report = self.evaluate_gap(policy, n_eval_episodes)
        report["gap_evaluation"] = gap_report

        print(f"\nSim-to-Real Transfer Results:")
        print(f"  Simulation Success Rate: {gap_report['sim_metrics']['sim_success_rate']:.2%}")
        print(f"  Real Success Rate: {gap_report['real_metrics']['real_success_rate']:.2%}")
        print(f"  Success Rate Gap: {gap_report['success_rate_gap']:.2%}")

        return policy, report


# ============================================================
# 主函数
# ============================================================

if __name__ == "__main__":
    # 示例: Sim-to-Real 迁移

    # 1. 创建仿真和真实环境
    from envs.pybullet_motor_env import PyBulletMotorEnv

    sim_env = PyBulletMotorEnv(render_mode="human")

    # 真实环境 (需要硬件接口)
    # from hardware.real_motor_env import RealMotorEnv
    # real_env = RealMotorEnv()

    # 2. 创建迁移管理器
    manager = Sim2RealManager(
        sim_env=sim_env,
        real_env=None,  # 暂时用 None
    )

    # 3. 域随机化示例
    randomizer = ProgressiveDomainRandomizer()
    for step in [0, 10000, 50000, 100000]:
        params = randomizer.randomize(step)
        print(f"Step {step}: {params}")
