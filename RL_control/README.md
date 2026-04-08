# RMD Motor Deep Reinforcement Learning Control System

基于 PyBullet 仿真 + 真实硬件迁移的深度强化学习控制系统

## 项目概述

本项目为 RMD 伺服电机开发深度强化学习控制框架，支持：

- **PyBullet 仿真环境**：无需真实硬件即可训练策略
- **模仿学习**：利用已有的动作录制数据加速训练
- **域随机化**：提高策略从仿真到真实硬件的迁移能力
- **真实硬件部署**：无缝对接现有 RMD 电机控制代码

## 项目结构

```
RL_control/
├── config/                 # 配置文件
│   └── rl_config.py       # RL 超参数、系统配置
├── envs/                   # 仿真环境
│   └── pybullet_motor_env.py  # PyBullet 电机仿真环境
├── data_preprocessing/     # 数据预处理
│   └── action_dataset.py   # 动作录制数据处理
├── vision/                 # 视觉处理
│   └── vision_processor.py # RealSense + 视觉编码器
├── training/               # RL 训练
│   └── rl_trainer.py       # Stable-Baselines3 训练器
├── imitation_learning/     # 模仿学习
│   └── behavioral_cloning.py  # 行为克隆、GAIL、DAgger
├── sim2real/              # Sim-to-Real 迁移
│   └── domain_randomization.py  # 域随机化、系统辨识
├── hardware/               # 真实硬件接口
│   └── real_motor_env.py   # Gymnasium 环境 + 硬件部署
├── main.py                # 主脚本
└── environment.yml        # Conda 环境配置
```

## 快速开始

### 1. 环境安装

```bash
# 创建并激活 Conda 环境
conda env create -f environment.yml
conda activate rmd_rl_control

# 或者使用 pip
pip install stable-baselines3 pybullet gymnasium torch torchvision
```

### 2. 动作数据准备

将已有的动作录制数据放在 `recorded_actions/` 目录：

```bash
ls recorded_actions/
# action_20251225_163727.json
# action_20260323_103700.json
# ...
```

### 3. 完整训练流程

```bash
cd RL_control

# 完整流程 (模仿学习 + RL + 评估)
python main.py --mode full --timesteps 500000 --algorithm PPO

# 仅仿真训练
python main.py --mode sim --timesteps 1000000

# 模仿学习预训练
python main.py --mode pretrain --il-epochs 100

# 域随机化训练
python main.py --mode dr --timesteps 500000
```

### 4. 真实硬件部署

```bash
# 部署训练好的策略
python main.py --mode deploy --model ./output/final_model.zip --serial-port COM3

# 评估
python main.py --mode eval --model ./output/final_model.zip --render
```

## 核心模块详解

### 仿真环境 (PyBullet)

电机物理模型包含：
- 位置/速度响应特性
- 摩擦力
- 死区非线性
- 可配置的电机延迟

```python
from envs.pybullet_motor_env import PyBulletMotorEnv

env = PyBulletMotorEnv(render_mode="human")
obs, info = env.reset()

for _ in range(1000):
    action = env.action_space.sample()  # 随机动作
    obs, reward, terminated, truncated, info = env.step(action)

    if terminated or truncated:
        obs, info = env.reset()

env.close()
```

### 模仿学习

使用已有的动作录制数据预训练策略网络：

```python
from imitation_learning.behavioral_cloning import BehavioralCloning
from data_preprocessing.action_dataset import prepare_il_dataset

# 准备数据
dataset, stats = prepare_il_dataset(
    data_dir="../recorded_actions",
    target_hz=60.0,
    augmentation=True,
)

# 行为克隆
bc = BehavioralCloning(state_dim=9, action_dim=3)
bc.fit(dataset, epochs=100, batch_size=64)

# 测试
action = bc.policy.get_action(np.zeros(9))
print(f"Predicted action: {action}")
```

### RL 训练 (PPO/SAC)

```python
from training.rl_trainer import RLTrainer, TrainingConfig
from envs.pybullet_motor_env import PyBulletMotorEnv

env = PyBulletMotorEnv()
config = TrainingConfig(
    algorithm="PPO",
    total_timesteps=500_000,
    eval_freq=10_000,
)

trainer = RLTrainer(env=env, config=config)
model = trainer.train()

# 评估
results = trainer.evaluate(n_episodes=10)
print(f"Mean reward: {results['mean_reward']:.2f}")
```

### 真实硬件部署

```python
from hardware.real_motor_env import RealMotorEnv, HardwareConfig

config = HardwareConfig(
    serial_port="COM3",
    baudrate=115200,
    control_frequency=60.0,
)

env = RealMotorEnv(config=config)
env.set_policy(model)

# 运行策略
env.run_policy(deterministic=True, max_steps=1000)
env.close()
```

## 算法选择指南

| 场景 | 推荐算法 | 特点 |
|------|---------|------|
| 连续动作控制 | **PPO** | 稳定、易调、采样效率高 |
| 连续动作、探索 | **SAC** | 自动熵调节、更好探索 |
| 连续动作平滑 | **TD3** | 适合精确控制 |
| 模仿学习后微调 | **PPO/SAC** | 两者皆可 |

## 关键超参数

### PPO

| 参数 | 默认值 | 说明 |
|------|-------|------|
| learning_rate | 3e-4 | 学习率 |
| n_steps | 2048 | 每次收集步数 |
| batch_size | 64 | 批大小 |
| n_epochs | 10 | 每次更新 epoch 数 |
| gamma | 0.99 | 折扣因子 |
| clip_range | 0.2 | PPO clip 范围 |

### 仿真环境

| 参数 | 默认值 | 说明 |
|------|-------|------|
| time_step | 0.001 | 仿真时间步 (秒) |
| max_speed_deg_per_sec | 120 | 最大速度 (度/秒) |
| motor_time_constant | 0.01 | 电机响应时间常数 |

## 视觉信息处理

视觉观测模块支持：

1. **RealSense L515**：RGB + 深度同步采集
2. **UNet 目标检测**：检测手术目标
3. **Depth-Anything-V2**：单目深度估计
4. **视觉编码器**：ResNet18/MobileNet 特征提取

```python
from vision.vision_processor import VisionObservationModule

vision = VisionObservationModule(
    encoder_config={"backbone": "resnet18", "feature_dim": 256},
    camera_config={"width": 640, "height": 480, "fps": 30},
)

observation = vision.get_observation()
# observation["features"]: 视觉特征
# observation["target_position"]: 目标 3D 位置
# observation["detection"]: 检测结果
```

## Sim-to-Real 迁移策略

### 1. 域随机化

随机化仿真参数，提高策略鲁棒性：

```python
from sim2real.domain_randomization import ProgressiveDomainRandomizer

randomizer = ProgressiveDomainRandomizer(
    initial_std_factor=0.5,
    final_std_factor=1.0,
    annealing_steps=500_000,
)

for step in range(0, 500_000, 1000):
    params = randomizer.randomize(step)
    apply_to_env(env, params)
```

### 2. 系统辨识

学习仿真和真实环境的差异：

```python
from sim2real.domain_randomization import SystemIdentifier

identifier = SystemIdentifier(state_dim=9, action_dim=3)
identifier.add_real_trajectory(real_states, real_actions, real_next_states)
identifier.add_sim_trajectory(sim_states, sim_actions, sim_next_states)
identifier.fit(epochs=100)
```

### 3. 在线微调

在真实环境中微调策略：

```python
from sim2real.domain_randomization import OnlineFineTuner

fine_tuner = OnlineFineTuner(
    policy=sim_policy,
    real_env=real_env,
)

fine_tuner.train(total_steps=5000)
```

## 硬件要求

### 仿真训练

- Python 3.10+
- PyTorch 2.0+
- NVIDIA GPU (推荐，但 CPU 也可)
- 8GB+ RAM

### 真实硬件部署

- RMD 伺服电机 (通过 RS485/USB 连接)
- Intel RealSense L515 (可选)
- 控制器: Windows/Linux

## 常见问题

### Q: 训练不收敛怎么办？

1. 检查奖励函数设计
2. 降低学习率
3. 增加训练步数
4. 使用模仿学习预训练
5. 启用域随机化

### Q: Sim-to-Real 差距大怎么办？

1. 增加域随机化范围
2. 使用渐进式随机化
3. 添加系统辨识
4. 在线微调

### Q: 如何处理视觉信息？

1. 使用预训练的视觉编码器
2. 初期简化视觉输入
3. 使用深度图而非 RGB

## 参考资料

- [Stable-Baselines3](https://stable-baselines3.readthedocs.io/)
- [PyBullet](https://pybullet.org/)
- [模仿学习综述](https://arxiv.org/abs/2005.05139)
- [Domain Randomization](https://arxiv.org/abs/1710.06537)
- [PPO Paper](https://arxiv.org/abs/1707.06347)

## 许可证

MIT License

## 联系方式

如有问题，请提交 Issue。
