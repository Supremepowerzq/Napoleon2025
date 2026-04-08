"""
RL 控制系统配置文件
定义所有超参数和系统参数
"""

# ============================================================
# 系统配置 (来自现有 config.py)
# ============================================================
MOTOR_CONFIG = {
    "num_motors": 3,
    "motor_names": ["motor_0", "motor_1", "motor_2"],
    # 电机 ID
    "motor_ids": [0, 1, 2],
    # 电机限位 [min, max] (度)
    "m0_limit": [-1000, 1000],   # M0: 前进/后退 (轮子或滑轨)
    "m1_limit": [-135, 135],     # M1: 水平旋转 (左右)
    "m2_limit": [-135, 135],     # M2: 垂直旋转 (上下)
    # 电机物理参数
    "max_speed_deg_per_sec": 120.0,  # 最大速度 (度/秒)
    "max_torque_nm": 1.5,             # 最大扭矩 (Nm)
    "position_resolution_deg": 0.01, # 位置分辨率 (度)
    # 串口配置
    "serial_port": "COM3",            # 需要根据实际修改
    "baudrate": 115200,
    "serial_timeout": 0.1,
}

# ============================================================
# 仿真环境配置
# ============================================================
SIM_CONFIG = {
    "engine": "pybullet",             # 仿真引擎: pybullet, mujoco, isaac
    "time_step": 0.001,               # 仿真时间步 (秒)
    "render": True,                   # 是否渲染
    "render_fps": 60,                 # 渲染帧率
    "camera_distance":1.0,            # 相机距离
    "camera_yaw": 45,                 # 相机偏航角
    "camera_pitch": -30,              # 相机俯仰角
    "camera_target": [0, 0, 0],       # 相机目标点
    # 域随机化
    "domain_randomization": {
        "enabled": True,
        "friction_range": [0.5, 1.5],       # 摩擦系数随机化
        "mass_range": [0.8, 1.2],           # 质量随机化
        "motor_delay_range": [0.0, 0.02],    # 电机响应延迟随机化 (秒)
        "vision_noise_std": 0.02,            # 视觉噪声标准差
    },
}

# ============================================================
# RL 算法配置
# ============================================================
RL_CONFIG = {
    # 算法选择: PPO, SAC, TD3, DDPG
    "algorithm": "PPO",
    # 观察空间配置
    "observation": {
        # 观测类型: "state", "vision", "multimodal"
        "type": "multimodal",
        # 低维状态 (电机状态)
        "state_dim": 9,  # [m0_angle, m0_vel, m1_angle, m1_vel, m2_angle, m2_vel, target_x, target_y, target_depth]
        # 视觉观测 (图像特征维度)
        "vision_dim": 256,
        # 目标观测
        "target_dim": 3,  # [target_x, target_y, target_depth]
        # 归一化
        "normalize": True,
        "normalize_clip": 10.0,
    },
    # 动作空间配置
    "action": {
        "num_actions": 3,               # 三轴电机
        "action_dim": 3,
        "action_type": "continuous",    # 连续动作
        # 动作范围 (归一化到 [-1, 1]，内部会映射到电机实际角度/速度)
        "action_scale": {
            "m0": (-1.0, 1.0),          # 前进/后退速度
            "m1": (-1.0, 1.0),          # 水平旋转速度
            "m2": (-1.0, 1.0),          # 垂直旋转速度
        },
    },
    # PPO 特定配置
    "ppo": {
        "learning_rate": 3e-4,
        "n_steps": 2048,                # 每次收集的步数
        "batch_size": 64,
        "n_epochs": 10,                 # 每次更新的 epoch 数
        "gamma": 0.99,                  # 折扣因子
        "gae_lambda": 0.95,             # GAE 参数
        "clip_range": 0.2,              # PPO clip 范围
        "clip_range_vf": None,          # 价值函数 clip (None 表示不 clip)
        "ent_coef": 0.01,               # 熵系数 (鼓励探索)
        "vf_coef": 0.5,                 # 价值函数系数
        "max_grad_norm": 0.5,           # 梯度裁剪
        "target_kl": None,              # 目标 KL 散度 (None 表示不限制)
    },
    # SAC 特定配置
    "sac": {
        "learning_rate": 3e-4,
        "buffer_size": 100000,          # 回放缓冲区大小
        "learning_starts": 1000,        # 开始学习前的步数
        "batch_size": 256,
        "tau": 0.005,                   # 目标网络更新系数
        "gamma": 0.99,
        "ent_coef": "auto",             # 自动调整熵系数
        "target_update_interval": 1,
    },
    # 训练配置
    "training": {
        "total_timesteps": 1_000_000,   # 总训练步数
        "eval_freq": 5000,              # 评估频率
        "eval_episodes": 10,            # 每次评估的 episodes 数
        "save_freq": 10000,             # 保存频率
        "log_freq": 100,                # 日志记录频率
        "n_eval_envs": 1,               # 并行评估环境数
    },
}

# ============================================================
# 视觉配置
# ============================================================
VISION_CONFIG = {
    # RealSense L515 配置
    "realsense": {
        "width": 640,
        "height": 480,
        "fps": 30,
        "enable_depth": True,
        "enable_color": True,
    },
    # 视觉编码器配置
    "encoder": {
        "backbone": "resnet18",         # 视觉 backbone: resnet18, resnet34, efficientnet_b0, mobilenet_v3
        "pretrained": True,            # 是否使用预训练权重
        "feature_dim": 256,             # 特征输出维度
        "freeze_bn": True,             # 是否冻结 BatchNorm
    },
    # 目标检测配置 (可选, 用于提供目标位置)
    "target": {
        "use_detection": True,          # 是否使用目标检测
        "detection_model": "unet",      # 检测模型: unet, yolo, segformer
        "detection_threshold": 0.5,     # 检测阈值
        "input_size": [640, 480],       # 输入图像尺寸
    },
    # 深度估计配置
    "depth": {
        "use_model": "depth_anything_v2",  # 深度估计模型
        "model_path": "../model_data/...",  # 预训练模型路径
        "normalize": True,
    },
}

# ============================================================
# 模仿学习配置
# ============================================================
IL_CONFIG = {
    "enabled": True,
    # 行为克隆
    "behavioral_cloning": {
        "enabled": True,
        "data_path": "../recorded_actions/",  # 动作数据路径
        "epochs": 100,
        "batch_size": 64,
        "learning_rate": 1e-4,
        "loss": "mse",                    # 或 "smooth_l1"
        "validation_split": 0.2,
    },
    # GAIL (生成对抗模仿学习)
    "gail": {
        "enabled": False,
        "discriminator_lr": 3e-4,
        "discriminator_hidden_dim": 256,
        "n_discriminator_updates": 5,
        "expert_trajectories": 1000,      # 专家轨迹数量
    },
    # 数据增强
    "augmentation": {
        "enabled": True,
        "noise_std": 0.01,
        "time_shift_max": 0.1,            # 最大时间偏移 (秒)
        "mixup_alpha": 0.2,               # MixUp 混合系数
    },
}

# ============================================================
# Sim-to-Real 配置
# ============================================================
SIM2REAL_CONFIG = {
    "enabled": True,
    # 域随机化策略
    "domain_randomization": {
        "enabled": True,
        # 物理参数随机化
        "physics": {
            "friction": {"mean": 1.0, "std": 0.2},
            "mass": {"mean": 1.0, "std": 0.1},
            "motor_constant": {"mean": 1.0, "std": 0.05},
        },
        # 视觉随机化
        "visual": {
            "light_direction": True,
            "background": True,
            "camera_noise": 0.01,
        },
        # 动力学随机化
        "dynamics": {
            "latency": [0.0, 0.005, 0.01, 0.02],  # 电机延迟
            "deadzone": [0.0, 0.1, 0.2],          # 死区
        },
    },
    # 域适应方法
    "domain_adaptation": {
        "method": "finetune",              # 方法: finetune, dropout, batch_norm_adapt
        "real_data_ratio": 0.2,           # 真实数据比例
        "adaptive_lr": True,
    },
}

# ============================================================
# 奖励函数配置
# ============================================================
REWARD_CONFIG = {
    "type": "dense",                      # 奖励类型: dense, sparse,混合
    # 密集奖励权重
    "weights": {
        "distance_to_goal": -1.0,         # 到目标距离的奖励权重
        "velocity_cost": -0.01,           # 速度成本
        "action_smoothness": -0.001,      # 动作平滑性成本
        "success": 10.0,                  # 成功奖励
        "failure": -10.0,                 # 失败惩罚
        "time_penalty": -0.01,            # 时间惩罚
    },
    # 成功条件
    "success": {
        "distance_threshold": 0.01,      # 成功距离阈值 (米)
        "time_limit": 10.0,               # 时间限制 (秒)
    },
    # 失败条件
    "failure": {
        "max_distance": 2.0,              # 最大允许距离
        "collision": True,                # 是否检测碰撞
    },
}

# ============================================================
# 实验配置
# ============================================================
EXPERIMENT_CONFIG = {
    "project_name": "RMD_Motor_RL",
    "experiment_name": "exp_001",
    "seed": 42,
    "device": "cuda",                      # 训练设备: cuda, cpu
    "cuda_deterministic": True,
    # 日志配置
    "logging": {
        "tensorboard": True,
        "wandb": False,                   # 需要 wandb 登录
        "save_video": True,
        "video_freq": 10000,
    },
    # 检查点配置
    "checkpoint": {
        "save_dir": "./checkpoints/",
        "keep_last_n": 5,
    },
}
