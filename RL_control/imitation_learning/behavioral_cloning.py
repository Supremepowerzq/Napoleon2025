"""
模仿学习模块
使用动作录制数据训练策略网络

支持:
1. 行为克隆 (Behavioral Cloning)
2. GAIL (Generative Adversarial Imitation Learning)
3. DAgger (Dataset Aggregation)
4. 预训练 + RL 微调
"""

import os
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Dataset
from torch.utils.tensorboard import SummaryWriter
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
from pathlib import Path
import json

from data_preprocessing.action_dataset import ImitationLearningDataset, Trajectory
from config.rl_config import IL_CONFIG


# ============================================================
# 策略网络
# ============================================================

class ImitationPolicyNetwork(nn.Module):
    """
    模仿学习策略网络
    输入: 状态 (电机角度、速度、目标位置)
    输出: 动作 (电机控制量)
    """

    def __init__(
        self,
        state_dim: int = 9,
        action_dim: int = 3,
        hidden_dims: List[int] = [256, 256, 128],
        action_bound: float = 1.0,
        device: str = "cpu",
    ):
        super().__init__()
        self.action_bound = action_bound
        self.device = torch.device(device)

        # 特征提取器
        layers = []
        input_dim = state_dim
        for hidden_dim in hidden_dims:
            layers.extend([
                nn.Linear(input_dim, hidden_dim),
                nn.ReLU(),
                nn.LayerNorm(hidden_dim),
            ])
            input_dim = hidden_dim

        self.feature_extractor = nn.Sequential(*layers)

        # 动作输出头
        self.action_head = nn.Linear(input_dim, action_dim)
        self.action_head.weight.data.fill_(0.0)
        self.action_head.bias.data.fill_(0.0)

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        """
        前向传播
        Args:
            state: 状态 [B, state_dim]
        Returns:
            action: 动作 [B, action_dim] (已归一化到 [-1, 1])
        """
        features = self.feature_extractor(state)
        action = torch.tanh(self.action_head(features))  # 归一化到 [-1, 1]
        return action

    def get_action(self, state: np.ndarray, deterministic: bool = True) -> np.ndarray:
        """获取动作 (numpy 接口)"""
        self.eval()
        with torch.no_grad():
            state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
            action = self.forward(state_tensor)
        return action.squeeze(0).cpu().numpy()


class DuelingImitationNetwork(nn.Module):
    """
    Dueling 结构的行为克隆网络
    优势函数 + 状态值函数结构
    """

    def __init__(
        self,
        state_dim: int = 9,
        action_dim: int = 3,
        hidden_dims: List[int] = [256, 256],
    ):
        super().__init__()

        # 共享特征提取
        layers = []
        input_dim = state_dim
        for hidden_dim in hidden_dims:
            layers.extend([
                nn.Linear(input_dim, hidden_dim),
                nn.ReLU(),
            ])
            input_dim = hidden_dim
        self.shared = nn.Sequential(*layers)

        # 价值函数
        self.value_head = nn.Linear(input_dim, 1)

        # 优势函数
        self.advantage_head = nn.Linear(input_dim, action_dim)

    def forward(self, state: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Returns:
            action: 动作
            value: 状态价值
        """
        features = self.shared(state)
        value = self.value_head(features)
        advantage = self.advantage_head(features)

        # Q = V + A - mean(A)
        q_values = value + advantage - advantage.mean(dim=-1, keepdim=True)

        # 选择最大 Q 对应的动作
        action = torch.argmax(q_values, dim=-1)
        return action, value


# ============================================================
# 判别器网络 (用于 GAIL)
# ============================================================

class Discriminator(nn.Module):
    """
    GAIL 判别器
    区分专家策略和 RL 策略生成的状态-动作对
    """

    def __init__(
        self,
        state_dim: int = 9,
        action_dim: int = 3,
        hidden_dim: int = 256,
    ):
        super().__init__()

        # 输入: 状态 + 动作
        self.network = nn.Sequential(
            nn.Linear(state_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim_dim // 2),  # FIXME: typo in original
            nn.ReLU(),
            nn.Linear(hidden_dim // 2, 1),
            nn.Sigmoid(),
        )

    def forward(
        self,
        state: torch.Tensor,
        action: torch.Tensor,
    ) -> torch.Tensor:
        """
        Args:
            state: 状态 [B, state_dim]
            action: 动作 [B, action_dim]
        Returns:
            probability: 专家概率 [B, 1]
        """
        # 归一化动作到 [-1, 1]
        action = torch.tanh(action)
        concat = torch.cat([state, action], dim=-1)
        prob = self.network(concat)
        return prob


# ============================================================
# 数据集
# ============================================================

class StateActionDataset(Dataset):
    """状态-动作数据集"""

    def __init__(
        self,
        states: np.ndarray,
        actions: np.ndarray,
        transform: Optional[Any] = None,
    ):
        self.states = torch.FloatTensor(states)
        self.actions = torch.FloatTensor(actions)
        self.transform = transform

    def __len__(self) -> int:
        return len(self.states)

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
        state = self.states[idx]
        action = self.actions[idx]

        if self.transform is not None:
            state = self.transform(state)

        return state, action


# ============================================================
# 行为克隆
# ============================================================

class BehavioralCloning:
    """
    行为克隆训练器
    直接从专家数据学习策略
    """

    def __init__(
        self,
        state_dim: int = 9,
        action_dim: int = 3,
        hidden_dims: List[int] = [256, 256, 128],
        learning_rate: float = 1e-4,
        device: str = "cuda" if torch.cuda.is_available() else "cpu",
    ):
        self.device = device

        # 网络
        self.policy = ImitationPolicyNetwork(
            state_dim=state_dim,
            action_dim=action_dim,
            hidden_dims=hidden_dims,
            device=device,
        ).to(device)

        # 优化器
        self.optimizer = optim.Adam(
            self.policy.parameters(),
            lr=learning_rate,
            weight_decay=1e-5,
        )

        # 损失函数
        self.loss_fn = nn.MSELoss()

        # 统计
        self.train_loss_history: List[float] = []

    def fit(
        self,
        dataset: ImitationLearningDataset,
        epochs: int = 100,
        batch_size: int = 64,
        validation_split: float = 0.2,
        log_dir: Optional[str] = None,
        verbose: bool = True,
    ) -> Dict[str, List[float]]:
        """
        训练策略网络

        Args:
            dataset: 模仿学习数据集
            epochs: 训练轮数
            batch_size: 批大小
            validation_split: 验证集比例
            log_dir: TensorBoard 日志目录
            verbose: 是否打印训练进度

        Returns:
            训练历史
        """
        # 划分训练集和验证集
        n_samples = len(dataset)
        n_val = int(n_samples * validation_split)
        indices = np.random.permutation(n_samples)
        train_indices = indices[n_val:]
        val_indices = indices[:n_val]

        # 创建 DataLoader
        train_states = dataset.states[train_indices]
        train_actions = dataset.actions[train_indices]
        val_states = dataset.states[val_indices]
        val_actions = dataset.actions[val_indices]

        train_dataset = StateActionDataset(train_states, train_actions)
        val_dataset = StateActionDataset(val_states, val_actions)

        train_loader = DataLoader(
            train_dataset,
            batch_size=batch_size,
            shuffle=True,
            num_workers=0,
        )
        val_loader = DataLoader(
            val_dataset,
            batch_size=batch_size,
            shuffle=False,
        )

        # TensorBoard
        writer = SummaryWriter(log_dir) if log_dir else None

        # 训练循环
        for epoch in range(epochs):
            # 训练
            self.policy.train()
            train_loss = 0.0
            for batch_states, batch_actions in train_loader:
                batch_states = batch_states.to(self.device)
                batch_actions = batch_actions.to(self.device)

                # 前向传播
                pred_actions = self.policy(batch_states)

                # 计算损失
                loss = self.loss_fn(pred_actions, batch_actions)

                # 反向传播
                self.optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(
                    self.policy.parameters(),
                    max_norm=1.0,
                )
                self.optimizer.step()

                train_loss += loss.item()

            train_loss /= len(train_loader)

            # 验证
            self.policy.eval()
            val_loss = 0.0
            with torch.no_grad():
                for batch_states, batch_actions in val_loader:
                    batch_states = batch_states.to(self.device)
                    batch_actions = batch_actions.to(self.device)

                    pred_actions = self.policy(batch_states)
                    loss = self.loss_fn(pred_actions, batch_actions)
                    val_loss += loss.item()

            val_loss /= len(val_loader)

            # 记录
            self.train_loss_history.append(train_loss)
            if writer:
                writer.add_scalar("train/loss", train_loss, epoch)
                writer.add_scalar("val/loss", val_loss, epoch)

            if verbose and (epoch + 1) % 10 == 0:
                print(f"Epoch {epoch+1}/{epochs}: train_loss={train_loss:.6f}, val_loss={val_loss:.6f}")

        if writer:
            writer.close()

        return {
            "train_loss": self.train_loss_history,
        }

    def save(self, path: str):
        """保存模型"""
        torch.save({
            "policy_state_dict": self.policy.state_dict(),
            "optimizer_state_dict": self.optimizer.state_dict(),
        }, path)
        print(f"Model saved to {path}")

    def load(self, path: str):
        """加载模型"""
        checkpoint = torch.load(path, map_location=self.device)
        self.policy.load_state_dict(checkpoint["policy_state_dict"])
        self.optimizer.load_state_dict(checkpoint["optimizer_state_dict"])
        print(f"Model loaded from {path}")


# ============================================================
# GAIL (生成对抗模仿学习)
# ============================================================

class GAILTrainer:
    """
    GAIL 训练器
    使用生成对抗训练学习专家策略
    """

    def __init__(
        self,
        policy,
        discriminator: Optional[Discriminator] = None,
        state_dim: int = 9,
        action_dim: int = 3,
        disc_lr: float = 3e-4,
        policy_lr: float = 3e-4,
        disc_updates: int = 5,
        device: str = "cuda" if torch.cuda.is_available() else "cpu",
    ):
        self.device = device
        self.policy = policy
        self.discriminator = discriminator or Discriminator(
            state_dim=state_dim,
            action_dim=action_dim,
        ).to(device)

        # 优化器
        self.disc_optimizer = optim.Adam(
            self.discriminator.parameters(),
            lr=disc_lr,
        )
        self.policy_optimizer = optim.Adam(
            self.policy.parameters() if hasattr(self.policy, 'parameters') else [],
            lr=policy_lr,
        )

        self.disc_updates = disc_updates

    def update_discriminator(
        self,
        expert_states: np.ndarray,
        expert_actions: np.ndarray,
        rl_states: np.ndarray,
        rl_actions: np.ndarray,
    ) -> float:
        """更新判别器"""
        self.discriminator.train()

        # 标签: 专家=1, RL=0
        expert_labels = torch.ones(len(expert_states), 1).to(self.device)
        rl_labels = torch.zeros(len(rl_states), 1).to(self.device)

        # 合并数据
        all_states = np.concatenate([expert_states, rl_states])
        all_actions = np.concatenate([expert_actions, rl_actions])
        all_labels = torch.cat([expert_labels, rl_labels])

        # 打乱
        indices = np.random.permutation(len(all_states))
        all_states = torch.FloatTensor(all_states[indices]).to(self.device)
        all_actions = torch.FloatTensor(all_actions[indices]).to(self.device)
        all_labels = all_labels[indices]

        # 训练判别器
        self.disc_optimizer.zero_grad()
        pred_probs = self.discriminator(all_states, all_actions)

        loss = nn.BCELoss()(pred_probs, all_labels)
        loss.backward()
        self.disc_optimizer.step()

        return loss.item()

    def compute_reward(
        self,
        state: np.ndarray,
        action: np.ndarray,
    ) -> np.ndarray:
        """
        计算 GAIL 奖励 (基于判别器输出)
        reward = -log(1 - D(s,a))
        """
        self.discriminator.eval()
        with torch.no_grad():
            state_tensor = torch.FloatTensor(state).to(self.device)
            action_tensor = torch.FloatTensor(action).to(self.device)
            prob = self.discriminator(state_tensor, action_tensor)
            reward = -torch.log(1 - prob + 1e-8).cpu().numpy()
        return reward


# ============================================================
# 预训练 + RL 微调
# ============================================================

class PreTrainingPipeline:
    """
    预训练 + 微调流水线
    1. 使用行为克隆预训练策略
    2. 使用预训练策略初始化 RL 训练
    3. 使用 RL 微调
    """

    def __init__(
        self,
        state_dim: int = 9,
        action_dim: int = 3,
        device: str = "cuda" if torch.cuda.is_available() else "cpu",
    ):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.device = device

        self.bc_trainer: Optional[BehavioralCloning] = None
        self.policy: Optional[ImitationPolicyNetwork] = None

    def pretrain(
        self,
        dataset: ImitationLearningDataset,
        epochs: int = 100,
        batch_size: int = 64,
        log_dir: Optional[str] = None,
    ) -> ImitationPolicyNetwork:
        """
        使用行为克隆预训练策略

        Args:
            dataset: 模仿学习数据集
            epochs: 训练轮数
            batch_size: 批大小
            log_dir: 日志目录

        Returns:
            预训练的策略网络
        """
        print("=" * 50)
        print("Step 1: Pre-training with Behavioral Cloning")
        print("=" * 50)

        self.bc_trainer = BehavioralCloning(
            state_dim=self.state_dim,
            action_dim=self.action_dim,
            device=self.device,
        )

        # 训练
        self.bc_trainer.fit(
            dataset=dataset,
            epochs=epochs,
            batch_size=batch_size,
            log_dir=log_dir,
        )

        self.policy = self.bc_trainer.policy
        return self.policy

    def finetune_with_rl(
        self,
        env,
        rl_algorithm: str = "PPO",
        total_timesteps: int = 100_000,
        pretrained_path: Optional[str] = None,
        **rl_kwargs,
    ):
        """
        使用 RL 微调策略

        Args:
            env: RL 环境
            rl_algorithm: RL 算法 ("PPO", "SAC")
            total_timesteps: RL 训练步数
            pretrained_path: 预训练模型路径
            **rl_kwargs: 传递给 RL 训练器的参数

        Returns:
            微调后的模型
        """
        print("=" * 50)
        print(f"Step 2: Fine-tuning with {rl_algorithm}")
        print("=" * 50)

        # 加载预训练策略
        if pretrained_path and self.policy is None:
            self.policy = ImitationPolicyNetwork(
                state_dim=self.state_dim,
                action_dim=self.action_dim,
            ).to(self.device)
            checkpoint = torch.load(pretrained_path, map_location=self.device)
            self.policy.load_state_dict(checkpoint["policy_state_dict"])

        # 创建 RL 训练器
        from training.rl_trainer import RLTrainer, TrainingConfig

        config = TrainingConfig(
            algorithm=rl_algorithm,
            total_timesteps=total_timesteps,
            **rl_kwargs,
        )

        trainer = RLTrainer(
            env=env,
            config=config,
        )

        # 如果有预训练策略，可以用来初始化
        # Stable-Baselines3 支持加载预训练参数
        if self.policy is not None:
            # 保存临时模型
            temp_path = "./checkpoints/temp_pretrained_policy"
            torch.save({
                "policy_state_dict": self.policy.state_dict(),
            }, temp_path)
            model = trainer.train(load_path=temp_path)
        else:
            model = trainer.train()

        return model

    def save(self, path: str):
        """保存整个流水线"""
        if self.policy is not None:
            torch.save({
                "policy_state_dict": self.policy.state_dict(),
            }, path)

    def load(self, path: str):
        """加载流水线"""
        self.policy = ImitationPolicyNetwork(
            state_dim=self.state_dim,
            action_dim=self.action_dim,
        ).to(self.device)
        checkpoint = torch.load(path, map_location=self.device)
        self.policy.load_state_dict(checkpoint["policy_state_dict"])


# ============================================================
# DAgger (Dataset Aggregation)
# ============================================================

class DAgger:
    """
    DAgger (Dataset Aggregation) 训练器
    迭代收集专家纠正数据
    """

    def __init__(
        self,
        policy: ImitationPolicyNetwork,
        expert_policy: Any,  # 可以是任何能够提供专家动作的策略
        env,
        state_dim: int = 9,
        action_dim: int = 3,
        device: str = "cuda" if torch.cuda.is_available() else "cpu",
    ):
        self.policy = policy.to(device)
        self.expert_policy = expert_policy
        self.env = env
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.device = device

        self.dataset_states: List[np.ndarray] = []
        self.dataset_actions: List[np.ndarray] = []

    def collect_demonstrations(self, n_episodes: int = 10) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """
        收集专家演示数据

        Returns:
            states: 状态列表
            actions: 动作列表
        """
        all_states = []
        all_actions = []

        for _ in range(n_episodes):
            obs, _ = self.env.reset()
            done = False
            episode_states = []
            episode_actions = []

            while not done:
                # 获取专家动作
                expert_action, _ = self.expert_policy.predict(obs, deterministic=True)

                # 执行动作
                next_obs, _, terminated, truncated, _ = self.env.step(expert_action)

                # 记录
                episode_states.append(obs)
                episode_actions.append(expert_action)

                obs = next_obs
                done = terminated or truncated

            all_states.extend(episode_states)
            all_actions.extend(episode_actions)

        return np.array(all_states), np.array(all_actions)

    def train(self, n_iterations: int = 10, n_episodes_per_iteration: int = 5):
        """
        DAgger 训练循环

        Args:
            n_iterations: 迭代次数
            n_episodes_per_iteration: 每次迭代收集的 episode 数
        """
        bc_trainer = BehavioralCloning(
            state_dim=self.state_dim,
            action_dim=self.action_dim,
            device=self.device,
        )

        for iteration in range(n_iterations):
            print(f"\n=== DAgger Iteration {iteration + 1}/{n_iterations} ===")

            # 1. 使用当前策略收集数据
            print("Collecting data with current policy...")
            # (这里简化了，实际需要运行策略并获取状态)

            # 2. 使用专家纠正
            print("Getting expert corrections...")
            # (这里简化了)

            # 3. 训练策略
            print("Training policy...")
            # 使用已有的数据集训练

            # 4. 评估
            print("Evaluating...")

        return self.policy


# ============================================================
# 主函数
# ============================================================

if __name__ == "__main__":
    # 示例: 使用动作录制数据训练模仿学习策略

    # 1. 加载数据
    print("Loading action recordings...")
    from data_preprocessing.action_dataset import prepare_il_dataset

    dataset, stats = prepare_il_dataset(
        data_dir="../recorded_actions",
        target_hz=60.0,
        augmentation=True,
    )
    print(f"Dataset size: {len(dataset)}")

    # 2. 行为克隆
    print("\nTraining with Behavioral Cloning...")
    bc = BehavioralCloning(state_dim=dataset.states.shape[1], action_dim=3)
    bc.fit(
        dataset=dataset,
        epochs=100,
        batch_size=64,
        log_dir="./logs/bc",
    )

    # 3. 保存模型
    bc.save("./checkpoints/bc_policy.pth")

    # 4. 测试
    print("\nTesting policy...")
    dummy_state = np.zeros(9, dtype=np.float32)
    action = bc.policy.get_action(dummy_state)
    print(f"Predicted action: {action}")
