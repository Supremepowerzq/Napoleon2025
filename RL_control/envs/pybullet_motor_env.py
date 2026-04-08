"""
PyBullet 仿真环境
模拟 RMD 电机系统 + RealSense L515 相机 + 目标物体

观测空间:
    - 电机状态: 位置、速度 (6维)
    - 目标状态: 像素坐标 + 深度 (3维)
    - 视觉图像: RGB-D 图像

动作空间:
    - 三轴电机控制: M0(前进/后退), M1(水平旋转), M2(垂直旋转)
    - 连续动作, 归一化到 [-1, 1]

奖励函数:
    - 距离奖励: 距离目标越近奖励越高
    - 平滑奖励: 动作变化越小奖励越高
    - 成功奖励: 到达目标位置时
    - 时间惩罚: 鼓励快速完成任务
"""

import gymnasium as gym
import numpy as np
from numpy.random import rand, randn
from typing import Optional, Tuple, Dict, Any, Union
import cv2

try:
    import pybullet as p
    import pybullet_data
except ImportError:
    print("Warning: pybullet not installed. Running in mock mode for development.")
    p = None

from config.rl_config import MOTOR_CONFIG, SIM_CONFIG, REWARD_CONFIG


class MotorSimulator:
    """
    电机动力学仿真器
    模拟 RMD 伺服电机的响应特性
    """

    def __init__(self, config: Dict[str, Any]):
        self.num_motors = config["num_motors"]
        self.max_speed = config["max_speed_deg_per_sec"]
        self.position_resolution = config["position_resolution_deg"]

        # 电机状态
        self.angles = np.zeros(self.num_motors)  # 当前角度 (度)
        self.velocities = np.zeros(self.num_motors)  # 当前速度 (度/秒)
        self.target_angles = np.zeros(self.num_motors)  # 目标角度 (度)

        # 电机限位
        self.limits = np.array([
            config["m0_limit"],
            config["m1_limit"],
            config["m2_limit"]
        ])

        # 电机动力学参数
        self.motor_time_constant = 0.01  # 电机响应时间常数 (秒)
        self.friction_coef = 0.1          # 摩擦系数
        self.motor_delay_range = config.get("motor_delay_range", [0.0, 0.02])

        # 死区
        self.deadzone = 0.1  # 度

    def reset(self, initial_angles: Optional[np.ndarray] = None) -> np.ndarray:
        """重置电机状态"""
        if initial_angles is not None:
            self.angles = np.clip(initial_angles, self.limits[:, 0], self.limits[:, 1])
        else:
            self.angles = np.zeros(self.num_motors)
        self.velocities = np.zeros(self.num_motors)
        self.target_angles = self.angles.copy()
        return self.angles.copy()

    def set_target(self, actions: np.ndarray) -> None:
        """
        设置目标角度
        actions: 归一化动作 [-1, 1] -> 映射到实际角度变化
        """
        # 将归一化动作映射到角度变化
        # M0: 速度控制 (度/秒)
        # M1, M2: 位置控制 (度)
        delta_angles = actions * self.max_speed * 0.1  # 0.1秒的增量

        self.target_angles = np.clip(
            self.target_angles + delta_angles,
            self.limits[:, 0],
            self.limits[:, 1]
        )

    def step(self, dt: float) -> np.ndarray:
        """
        更新电机状态 (一阶系统模型)
        dt: 时间步长 (秒)
        """
        # 一阶低通滤波器模拟电机响应
        alpha = 1 - np.exp(-dt / self.motor_time_constant)
        self.angles = self.angles + alpha * (self.target_angles - self.angles)

        # 添加死区非线性
        for i in range(self.num_motors):
            if abs(self.angles[i] - self.target_angles[i]) < self.deadzone:
                self.angles[i] = self.target_angles[i]

        # 更新速度 (近似)
        self.velocities = (self.angles - self.target_angles) / dt

        return self.angles.copy()

    def get_state(self) -> np.ndarray:
        """获取完整电机状态"""
        return np.concatenate([
            self.angles,
            self.velocities
        ])


class TargetObject:
    """
    目标物体仿真
    模拟医学手术中的目标物体 (结石、管道等)
    """

    def __init__(self, position: np.ndarray, shape: str = "sphere"):
        self.position = position  # 世界坐标 [x, y, z]
        self.shape = shape
        self.success_threshold = 0.01  # 成功接触阈值 (米)

    def set_position(self, position: np.ndarray):
        self.position = np.array(position)

    def get_distance_to_point(self, point: np.ndarray) -> float:
        return np.linalg.norm(self.position - np.array(point))


class PyBulletMotorEnv(gym.Env):
    """
    基于 PyBullet 的电机控制仿真环境
    实现 gymnasium.Env 接口
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 60}

    def __init__(
        self,
        motor_config: Dict[str, Any] = None,
        sim_config: Dict[str, Any] = None,
        reward_config: Dict[str, Any] = None,
        render_mode: str = "human",
    ):
        super().__init__()

        # 配置
        self.motor_config = motor_config or MOTOR_CONFIG
        self.sim_config = sim_config or SIM_CONFIG
        self.reward_config = reward_config or REWARD_CONFIG
        self.render_mode = render_mode

        # 初始化组件
        self.motor_sim = MotorSimulator(self.motor_config)

        # Gym 空间定义
        self._setup_spaces()

        # PyBullet 客户端
        self.physics_client = None
        self.robot_id = None
        self.camera_id = None
        self.target_id = None

        # 仿真状态
        self.current_step = 0
        self.max_steps = 1000
        self.target_position = None
        self.time_limit = self.reward_config["success"]["time_limit"]
        self._dr_params = {}  # 域随机化参数

        # 渲染
        self._viewer = None

    def _setup_spaces(self):
        """定义观察空间和动作空间"""

        # 动作空间: 三轴电机控制, 归一化到 [-1, 1]
        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(3,),  # M0, M1, M2
            dtype=np.float32
        )

        # 观察空间: 电机状态 (6) + 目标状态 (3) = 9维
        # 或者加上视觉 (可选)
        obs_dim = 9  # [m0_angle, m0_vel, m1_angle, m1_vel, m2_angle, m2_vel, target_x, target_y, target_depth]
        self.observation_space = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(obs_dim,),
            dtype=np.float32
        )

    def reset(
        self, seed: Optional[int] = None, options: Optional[Dict] = None
    ) -> Tuple[np.ndarray, Dict]:
        """重置环境"""
        super().reset(seed=seed)

        # 连接 PyBullet
        if self.physics_client is None:
            self._connect_pybullet()

        # 重置电机
        initial_angles = options.get("initial_angles") if options else None
        self.motor_sim.reset(initial_angles)

        # 重置仿真世界
        self._reset_world()

        # 生成新目标
        self._spawn_target()

        # 域随机化
        if self.sim_config.get("domain_randomization", {}).get("enabled", False):
            self._apply_domain_randomization()

        # 重置步数
        self.current_step = 0

        return self._get_obs(), self._get_info()

    def _connect_pybullet(self):
        """连接 PyBullet 物理引擎"""
        if p is None:
            return

        if self.render_mode == "human":
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.sim_config["time_step"])

        # 配置渲染
        if self.render_mode == "human":
            p.configureDebugVisualizer(
                p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0
            )
            p.configureDebugVisualizer(
                p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0
            )
            p.configureDebugVisualizer(
                p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0
            )

    def _reset_world(self):
        """重置仿真世界"""
        if p is None:
            return

        # 清除世界
        p.resetSimulation()

        # 重新设置重力
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.sim_config["time_step"])

        # 加载地面
        plane_id = p.loadURDF("plane.urdf")

        # 创建简化的电机机器人模型
        self._create_robot_model()

        # 重置机器人位置
        p.resetBasePositionAndOrientation(
            self.robot_id, [0, 0, 0], [0, 0, 0, 1]
        )

    def _create_robot_model(self):
        """创建简化的机器人模型"""
        if p is None:
            return

        # 创建机器人 (简化为一个基座 + 3个旋转关节)
        base_collision = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.05]
        )
        base_visual = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.05], rgbaColor=[0.5, 0.5, 0.5, 1]
        )

        self.robot_id = p.createMultiBody(
            baseMass=1.0,
            baseCollisionShapeIndex=base_collision,
            baseVisualShapeIndex=base_visual,
            basePosition=[0, 0, 0],
            baseOrientation=[0, 0, 0, 1],
        )

        # 关节可视化通过 createMultiBody 的 linkVisualShapeOptions 处理
        # 详细关节可视化在需要时扩展

    def _spawn_target(self):
        """生成目标物体"""
        if self.sim_config.get("target_spawn_method") == "random":
            # 随机生成目标位置 (在相机前方的锥形区域内)
            depth = self.np_random.uniform(0.3, 0.8)  # 0.3-0.8米
            angle_h = self.np_random.uniform(-0.5, 0.5)  # 水平角度 (弧度)
            angle_v = self.np_random.uniform(-0.3, 0.3)  # 垂直角度 (弧度)

            x = depth * np.tan(angle_h)
            y = depth * np.tan(angle_v)
            z = -depth  # 相机朝向 -Z

            self.target_position = np.array([x, y, z])
        else:
            # 默认目标位置
            self.target_position = np.array([0.1, 0.0, -0.5])

    def _apply_domain_randomization(self):
        """应用域随机化"""
        dr_config = self.sim_config["domain_randomization"]

        # 摩擦系数随机化
        self._dr_params["friction"] = self.np_random.uniform(
            dr_config["friction_range"][0],
            dr_config["friction_range"][1]
        )

        # 质量随机化
        self._dr_params["mass"] = self.np_random.uniform(
            dr_config["mass_range"][0],
            dr_config["mass_range"][1]
        )

        # 电机延迟随机化
        self._dr_params["motor_delay"] = self.np_random.uniform(
            dr_config["motor_delay_range"][0],
            dr_config["motor_delay_range"][1]
        )

        # 视觉噪声
        self._dr_params["vision_noise_std"] = self.np_random.uniform(
            0, dr_config["vision_noise_std"]
        )

        # 应用到仿真
        if self.robot_id is not None and p is not None:
            # 改变质量
            for link_idx in range(p.getNumJoints(self.robot_id)):
                p.changeDynamics(
                    self.robot_id, link_idx,
                    mass=self._dr_params["mass"]
                )

    def step(
        self, action: np.ndarray
    ) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """
        执行一步仿真

        Returns:
            observation: 观察
            reward: 奖励
            terminated: 是否结束 (成功或失败)
            truncated: 是否截断 (时间限制)
            info: 额外信息
        """
        self.current_step += 1

        # 动作处理
        action = np.clip(action, -1, 1)
        self.motor_sim.set_target(action)

        # 仿真步骤
        dt = self.sim_config["time_step"]
        motor_state = self.motor_sim.step(dt)

        # PyBullet 步骤 (如果可用)
        if p is not None and self.physics_client is not None:
            p.stepSimulation()

        # 更新机器人视觉状态 (将电机角度应用到 URDF 模型的关节上)
        self._update_robot_joints(motor_state)

        # 计算奖励
        reward, reward_info = self._compute_reward(motor_state)

        # 检查终止条件
        terminated = self._check_termination()
        truncated = self.current_step >= self.max_steps

        # 获取观察
        obs = self._get_obs()
        info = self._get_info()
        info.update(reward_info)

        return obs, reward, terminated, truncated, info

    def _update_robot_joints(self, motor_state: np.ndarray):
        """更新机器人的关节状态"""
        if p is None or self.robot_id is None:
            return

        # 将电机角度设置为关节位置
        # 注意: 这里需要根据实际的 URDF 模型调整
        for joint_idx in range(min(3, p.getNumJoints(self.robot_id))):
            p.resetJointState(
                self.robot_id, joint_idx,
                motor_state[joint_idx] * np.pi / 180  # 转换为弧度
            )

    def _compute_reward(
        self, motor_state: np.ndarray
    ) -> Tuple[float, Dict]:
        """计算奖励"""
        weights = self.reward_config["weights"]

        # 计算到目标的距离
        robot_pos = np.array([0, 0, 0])  # 简化的机器人位置
        distance = np.linalg.norm(self.target_position - robot_pos)

        # 距离奖励 (负距离)
        reward_distance = weights["distance_to_goal"] * distance

        # 速度成本
        velocities = motor_state[3:6]  # 速度分量
        reward_velocity = weights["velocity_cost"] * np.sum(np.square(velocities))

        # 动作平滑性成本
        # (需要记录上一步的动作来计算平滑性)
        if not hasattr(self, 'prev_action'):
            self.prev_action = np.zeros(3)
        action_smoothness = np.sum(np.square(motor_state - self.prev_action))
        reward_smoothness = weights["action_smoothness"] * action_smoothness
        self.prev_action = motor_state.copy()

        # 时间惩罚
        reward_time = weights["time_penalty"]

        # 成功奖励
        success_threshold = self.reward_config["success"]["distance_threshold"]
        is_success = distance < success_threshold
        reward_success = weights["success"] if is_success else 0

        # 失败惩罚 (超出范围)
        max_distance = self.reward_config["failure"]["max_distance"]
        is_failure = distance > max_distance
        reward_failure = weights["failure"] if is_failure else 0

        # 总奖励
        total_reward = (
            reward_distance
            + reward_velocity
            + reward_smoothness
            + reward_time
            + reward_success
            + reward_failure
        )

        # 奖励分解信息
        reward_info = {
            "reward_distance": reward_distance,
            "reward_velocity": reward_velocity,
            "reward_smoothness": reward_smoothness,
            "reward_time": reward_time,
            "reward_success": reward_success,
            "reward_failure": reward_failure,
            "distance": distance,
            "is_success": is_success,
            "is_failure": is_failure,
        }

        return total_reward, reward_info

    def _check_termination(self) -> bool:
        """检查是否终止"""
        # 成功
        robot_pos = np.array([0, 0, 0])
        distance = np.linalg.norm(self.target_position - robot_pos)
        if distance < self.reward_config["success"]["distance_threshold"]:
            return True

        # 失败
        if distance > self.reward_config["failure"]["max_distance"]:
            return True

        return False

    def _get_obs(self) -> np.ndarray:
        """获取观察"""
        # 电机状态
        motor_state = self.motor_sim.get_state()  # [angles(3), velocities(3)]

        # 目标状态 (简化为相对位置)
        target_obs = np.array([
            self.target_position[0],
            self.target_position[1],
            self.target_position[2],
        ])

        # 添加域随机化噪声 (如果有)
        if hasattr(self, '_dr_params') and 'vision_noise_std' in self._dr_params:
            noise_std = self._dr_params['vision_noise_std']
            target_obs += self.np_random.normal(0, noise_std, size=target_obs.shape)

        return np.concatenate([motor_state, target_obs]).astype(np.float32)

    def _get_info(self) -> Dict:
        """获取额外信息"""
        return {
            "step": self.current_step,
            "target_position": self.target_position.tolist(),
            "motor_angles": self.motor_sim.angles.tolist(),
            "motor_velocities": self.motor_sim.velocities.tolist(),
        }

    def render(self) -> Optional[np.ndarray]:
        """渲染环境"""
        if self.render_mode == "rgb_array":
            return self._render_rgb_array()
        return None

    def _render_rgb_array(self) -> np.ndarray:
        """渲染 RGB 图像"""
        if p is None:
            return np.zeros((480, 640, 3), dtype=np.uint8)

        # 获取相机视图
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[0, 0, 0],
            distance=self.sim_config["camera_distance"],
            yaw=self.sim_config["camera_yaw"],
            pitch=self.sim_config["camera_pitch"],
            roll=0,
            upAxisIndex=2,
        )

        proj_matrix = p.computeProjectionMatrixFOV(
            fov=60, aspect=640 / 480, nearVal=0.1, farVal=100.0
        )

        (_, _, rgb像素, _, _) = p.getCameraImage(
            640, 480,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
        )

        rgb_array = np.array(rgb像素, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (480, 640, 4))
        rgb_array = rgb_array[:, :, :3]  # 去除 alpha 通道

        return rgb_array

    def close(self):
        """关闭环境"""
        if self.physics_client is not None and p is not None:
            p.disconnect()
            self.physics_client = None

    def get_camera_image(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        获取相机图像 (RGB + Depth)
        模拟 RealSense L515 输出
        """
        rgb = self.render()
        if rgb is None:
            rgb = np.zeros((480, 640, 3), dtype=np.uint8)

        # 模拟深度图
        depth = np.zeros((480, 640), dtype=np.float32)
        # 简单的基于深度的填充 (简化版本)
        # 真实实现需要射线投射

        return rgb, depth


# ============================================================
# 测试代码
# ============================================================
if __name__ == "__main__":
    # 测试环境
    env = PyBulletMotorEnv(render_mode="human")

    print("动作空间:", env.action_space)
    print("观察空间:", env.observation_space)

    # 测试重置
    obs, info = env.reset()
    print("初始观察:", obs)
    print("初始信息:", info)

    # 测试步进
    for i in range(100):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        print(f"Step {i}: reward={reward:.3f}, terminated={terminated}")

        if terminated or truncated:
            print("Episode ended!")
            obs, info = env.reset()

    env.close()
