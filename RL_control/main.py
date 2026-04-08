"""
深度强化学习控制系统 - 主训练脚本
整合所有模块，提供端到端的训练流程

使用方式:
    # 完整流程 (仿真预训练 -> 模仿学习 -> 域随机化 -> 真实部署)
    python main.py --mode full

    # 仅仿真训练
    python main.py --mode sim --timesteps 1000000

    # 模仿学习预训练
    python main.py --mode pretrain --il-epochs 100

    # 真实硬件部署
    python main.py --mode deploy --model checkpoints/best_model.zip
"""

import os
import sys
import argparse
import time
import numpy as np
import torch
from pathlib import Path
from datetime import datetime

# 添加项目路径
PROJECT_ROOT = Path(__file__).parent
sys.path.insert(0, str(PROJECT_ROOT))

from stable_baselines3.common.monitor import Monitor
from config.rl_config import RL_CONFIG, SIM_CONFIG, MOTOR_CONFIG, IL_CONFIG, SIM2REAL_CONFIG
from training.rl_trainer import RLTrainer, TrainingConfig
from imitation_learning.behavioral_cloning import (
    BehavioralCloning,
    PreTrainingPipeline,
    ImitationPolicyNetwork,
)
from data_preprocessing.action_dataset import prepare_il_dataset, ActionDatasetLoader
from envs.pybullet_motor_env import PyBulletMotorEnv
from sim2real.domain_randomization import Sim2RealManager, DomainRandomizer
from hardware.real_motor_env import RealMotorEnv, HardwareConfig, OfflineInference


# ============================================================
# 命令行参数
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="RMD Motor RL Control System",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # 运行模式
    parser.add_argument(
        "--mode",
        type=str,
        choices=["full", "sim", "pretrain", "dr", "deploy", "eval"],
        default="full",
        help="运行模式",
    )

    # 训练参数
    parser.add_argument("--timesteps", type=int, default=1_000_000, help="总训练步数")
    parser.add_argument("--algorithm", type=str, default="PPO", choices=["PPO", "SAC", "TD3", "DDPG"])
    parser.add_argument("--seed", type=int, default=42, help="随机种子")
    parser.add_argument("--device", type=str, default="auto", help="计算设备 (cuda/cpu/auto)")

    # 模仿学习参数
    parser.add_argument("--il-epochs", type=int, default=100, help="模仿学习训练轮数")
    parser.add_argument("--il-batch-size", type=int, default=64, help="模仿学习批大小")
    parser.add_argument("--data-dir", type=str, default="../recorded_actions", help="动作数据目录")

    # 模型路径
    parser.add_argument("--model", type=str, default=None, help="模型路径")
    parser.add_argument("--output-dir", type=str, default="./output", help="输出目录")

    # 硬件配置
    parser.add_argument("--serial-port", type=str, default="COM3", help="串口")
    parser.add_argument("--baudrate", type=int, default=115200, help="波特率")

    # 其他
    parser.add_argument("--render", action="store_true", help="渲染仿真环境")
    parser.add_argument("--verbose", action="store_true", help="详细输出")
    parser.add_argument("--config", type=str, default=None, help="配置文件路径")

    return parser.parse_args()


def _resolve_device(device: str) -> str:
    """将 'auto' 解析为实际可用设备"""
    if device == "auto":
        return "cuda" if torch.cuda.is_available() else "cpu"
    return device


# ============================================================
# 模式 1: 完整流程
# ============================================================

def run_full_pipeline(args):
    """完整训练流程"""
    print("=" * 70)
    print("RMD Motor RL Control System - Full Pipeline")
    print("=" * 70)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # 阶段 0: 数据准备
    print("\n" + "=" * 50)
    print("Stage 0: Data Preparation")
    print("=" * 50)

    try:
        dataset, stats = prepare_il_dataset(
            data_dir=args.data_dir,
            output_path=output_dir / "dataset.npz",
            target_hz=60.0,
            augmentation=True,
        )
        print(f"Dataset prepared: {len(dataset)} samples")
    except Exception as e:
        print(f"Data preparation failed: {e}")
        print("Continuing without imitation learning...")
        dataset = None

    # 阶段 1: 仿真环境创建
    print("\n" + "=" * 50)
    print("Stage 1: Simulation Environment Setup")
    print("=" * 50)

    env = PyBulletMotorEnv(render_mode="human" if args.render else "rgb_array")
    env = Monitor(env, info_keywords=("is_success",))
    print(f"Simulation environment created")

    # 阶段 2: 模仿学习预训练 (可选)
    pretrained_policy = None
    if dataset is not None and args.il_epochs > 0:
        print("\n" + "=" * 50)
        print("Stage 2: Imitation Learning Pre-training")
        print("=" * 50)

        bc = BehavioralCloning(
            state_dim=dataset.states.shape[1],
            action_dim=3,
            device=args.device,
        )

        bc.fit(
            dataset=dataset,
            epochs=args.il_epochs,
            batch_size=args.il_batch_size,
            log_dir=str(output_dir / "logs" / "bc"),
        )

        bc.save(str(output_dir / "bc_policy.pth"))
        pretrained_policy = bc.policy
        print("Imitation learning pre-training completed")

    # 阶段 3: RL 训练
    print("\n" + "=" * 50)
    print("Stage 3: Reinforcement Learning Training")
    print("=" * 50)

    config = TrainingConfig(
        algorithm=args.algorithm,
        total_timesteps=args.timesteps,
        eval_freq=10000,
        save_freq=20000,
        seed=args.seed,
        device=args.device,
    )

    trainer = RLTrainer(
        env=env,
        config=config,
        log_dir=str(output_dir / "logs" / "rl"),
        checkpoint_dir=str(output_dir / "checkpoints"),
    )

    model = trainer.train(
        policy="MlpPolicy",
        load_path=pretrained_policy if pretrained_policy else None,
    )

    # 保存最终模型
    trainer.save(str(output_dir / "final_model.zip"))
    print("RL training completed")

    # 阶段 4: 评估
    print("\n" + "=" * 50)
    print("Stage 4: Evaluation")
    print("=" * 50)

    results = trainer.evaluate(n_episodes=10)
    print(f"Evaluation results: {results}")

    # 保存评估结果
    with open(output_dir / "eval_results.json", "w") as f:
        import json
        json.dump(results, f, indent=2)

    # 关闭环境
    env.close()

    print("\n" + "=" * 70)
    print("Full pipeline completed!")
    print(f"Model saved to: {output_dir}")
    print("=" * 70)

    return model


# ============================================================
# 模式 2: 仅仿真训练
# ============================================================

def run_sim_training(args):
    """仅在仿真环境中训练"""
    print("=" * 70)
    print("Simulation Training Mode")
    print("=" * 70)

    # 创建环境
    env = PyBulletMotorEnv(render_mode="human" if args.render else "rgb_array")
    env = Monitor(env, info_keywords=("is_success",))

    # 配置训练
    config = TrainingConfig(
        algorithm=args.algorithm,
        total_timesteps=args.timesteps,
        seed=args.seed,
        device=args.device,
    )

    # 创建训练器
    trainer = RLTrainer(
        env=env,
        config=config,
        log_dir=f"{args.output_dir}/logs",
        checkpoint_dir=f"{args.output_dir}/checkpoints",
    )

    # 训练
    model = trainer.train(policy="MlpPolicy")

    # 评估
    results = trainer.evaluate(n_episodes=10)
    print(f"Evaluation results: {results}")

    # 保存
    trainer.save(f"{args.output_dir}/final_model.zip")

    env.close()
    return model


# ============================================================
# 模式 3: 模仿学习预训练
# ============================================================

def run_pretrain(args):
    """模仿学习预训练"""
    print("=" * 70)
    print("Imitation Learning Pre-training Mode")
    print("=" * 70)

    # 准备数据
    dataset, stats = prepare_il_dataset(
        data_dir=args.data_dir,
        target_hz=60.0,
        augmentation=True,
    )

    print(f"Dataset size: {len(dataset)}")
    state_dim = dataset.states.shape[1]  # 从数据集获取实际维度
    print(f"State dimension: {state_dim}")

    # 行为克隆
    bc = BehavioralCloning(
        state_dim=state_dim,
        action_dim=3,
        device=args.device,
    )

    bc.fit(
        dataset=dataset,
        epochs=args.il_epochs,
        batch_size=args.il_batch_size,
        log_dir=f"{args.output_dir}/logs",
    )

    # 保存
    bc.save(f"{args.output_dir}/bc_policy.pth")

    # 测试
    print("\nTesting trained policy...")
    dummy_state = np.zeros(state_dim, dtype=np.float32)
    action = bc.policy.get_action(dummy_state)
    print(f"Predicted action for zero state: {action}")

    return bc.policy


# ============================================================
# 模式 4: 域随机化训练
# ============================================================

def run_domain_randomization(args):
    """域随机化训练"""
    print("=" * 70)
    print("Domain Randomization Training Mode")
    print("=" * 70)

    # 创建环境
    env = PyBulletMotorEnv(render_mode="human" if args.render else "rgb_array")
    env = Monitor(env, info_keywords=("is_success",))

    # 创建迁移管理器
    manager = Sim2RealManager(
        sim_env=env,
        real_env=None,
        sim_config=SIM_CONFIG,
        sim2real_config=SIM2REAL_CONFIG,
    )

    # 训练
    model = manager.train_with_dr(
        policy=None,
        total_timesteps=args.timesteps,
    )

    # 保存
    model.save(f"{args.output_dir}/dr_policy.zip")

    env.close()
    return model


# ============================================================
# 模式 5: 真实硬件部署
# ============================================================

def run_deploy(args):
    """真实硬件部署"""
    print("=" * 70)
    print("Real Hardware Deployment Mode")
    print("=" * 70)

    if args.model is None:
        print("Error: --model is required for deployment mode")
        return None

    # 硬件配置
    config = HardwareConfig(
        serial_port=args.serial_port,
        baudrate=args.baudrate,
        control_frequency=60.0,
    )

    # 创建推理器
    inference = OfflineInference(
        model_path=args.model,
        hardware_config=config,
    )

    # 运行
    results = inference.run(n_episodes=1)

    inference.close()
    return results


# ============================================================
# 模式 6: 评估
# ==========================================================

def run_eval(args):
    """评估模式"""
    print("=" * 70)
    print("Evaluation Mode")
    print("=" * 70)

    if args.model is None:
        print("Error: --model is required for evaluation mode")
        return None

    # 创建环境
    env = PyBulletMotorEnv(render_mode="human" if args.render else "rgb_array")
    env = Monitor(env, info_keywords=("is_success",))

    # 加载模型
    from training.rl_trainer import load_trained_model
    model = load_trained_model(args.model, algorithm=args.algorithm)

    # 评估
    trainer = RLTrainer(env=env, config=TrainingConfig())
    trainer.model = model

    results = trainer.evaluate(n_episodes=10, render=args.render)
    print(f"Evaluation results: {results}")

    env.close()
    return results


# ============================================================
# 主函数
# ============================================================

def main():
    args = parse_args()

    # 设置随机种子
    import random
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    random.seed(args.seed)

    # 解析设备
    args.device = _resolve_device(args.device)

    # 根据模式运行
    if args.mode == "full":
        run_full_pipeline(args)
    elif args.mode == "sim":
        run_sim_training(args)
    elif args.mode == "pretrain":
        run_pretrain(args)
    elif args.mode == "dr":
        run_domain_randomization(args)
    elif args.mode == "deploy":
        run_deploy(args)
    elif args.mode == "eval":
        run_eval(args)


if __name__ == "__main__":
    main()
