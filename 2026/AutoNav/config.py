# -*- coding: utf-8 -*-
"""
AutoNav 配置文件
================
集中管理电机限位、安全参数、各层控制增益。
"""

# ── 电机物理限位（与主程序保持一致）───────────────────────────────
MOTOR_LIMITS = {
    0: (-900.0, 0.0),     # M0 前进/后退
    1: (-170.0, 170.0),   # M1 左右弯曲
    2: (-600.0, 600.0),   # M2 上下弯曲
}

# ── 导航速度控制 ───────────────────────────────────────────────────
# 序列导航最大单步角度增量（安全限幅）
PLAYBACK_MAX_DELTA = {0: 6.0, 1: 4.0, 2: 4.0}   # °/帧

# 导航时间轴速度缩放。单路径导航直接使用 JSON 原始时间戳：
# 1.0 = 与手动采集相同的真实节奏；0.8 = 慢 20%；1.2 = 快 20%。
# 无论速度如何，完整终点位移都保持不变。
PLAYBACK_SPEED = 1.0

# 原始轨迹导航的电机位置模式速度上限（需允许电机追上采集轨迹）。
NAV_MOTOR_MAX_SPEED = {0: 80.0, 1: 30.0, 2: 80.0}

# 路径结束后继续保持最终目标，等待真实电机到位。
NAV_SETTLE_TOLERANCE = {0: 3.0, 1: 2.0, 2: 3.0}
NAV_SETTLE_TIMEOUT_S = 12.0

# 目标帧率（Hz），采集时使用的基准频率
TARGET_HZ = 20.0

# 当前主程序控制循环节拍（Hz），用于自动计算速度缩放
CONTROL_LOOP_HZ = 60.0

# ── Layer 2: 岔口引导 ─────────────────────────────────────────────
# 岔口面积下限：低于此值认为岔口太远 / 不可信，不做修正
BIFUR_AREA_THRESHOLD = 0.01

# 画面中心死区：岔口中心落在中心附近时不修正，避免来回抖动。
# bifur_cx/cy 使用 [-1, 1] 归一化坐标，0.08 表示半幅画面的 8%。
BIFUR_CENTER_DEADZONE = 0.08

# P 控制增益：offset = BIFUR_GAIN × deadzone(bifur_error)
# 注意：这里得到的是相对于 Layer 1 原始路径的“总角度偏置”，不是每帧增量。
# bifur_cx ∈ [-1,1]，岔口偏右时 cx>0，需要向右修正 → +m1
BIFUR_GAIN_M1 = 2.0   # 左右总偏置增益（° per unit），全流程先采用保守微调
BIFUR_GAIN_M2 = 1.5   # 上下总偏置增益

# Layer 2 相对 Layer 1 的最大总偏置（°）。即使岔口长期远离画面中心，
# 也不会再逐帧累积并覆盖原始采集路径。
BIFUR_MAX_OFFSET_M1 = 2.0
BIFUR_MAX_OFFSET_M2 = 1.5

# EMA 响应系数：越大跟随越快，越小越平滑。
BIFUR_RESPONSE_ALPHA = 0.25

# 岔口微调使用连续权重，不做“达到某进度才突然启动”的硬门控。
# 路径起点保留少量微调，随进度增加，在 80% 时达到完整强度。
BIFUR_MIN_PROGRESS_WEIGHT = 0.25
BIFUR_FULL_EFFECT_PROGRESS = 0.80

# 只在岔口几何中心已经靠近画面中心时微调。<=0.12 为完整强度；
# 0.12~0.35 线性减弱；>=0.35 完全停止，避免远距离追踪失败时拉偏轨迹。
BIFUR_FULL_EFFECT_DISTANCE = 0.12
BIFUR_ZERO_EFFECT_DISTANCE = 0.35

# 视觉门控：视觉线程停更、候选跳变或连续帧不足时不接受新的修正。
BIFUR_VISUAL_MAX_AGE_S = 0.25
BIFUR_STABLE_FRAMES = 4
BIFUR_MAX_TRACK_JUMP = 0.12

# ── Layer 3: 阻塞物处理 ───────────────────────────────────────────
# 进入清除：检测结果、面积、视觉时效和连续新视觉帧共同确认。
# 当前实机验证表明目标占画面约 1/3 时追踪稳定，第一版取 0.30。
OBSTRUCTION_ENTER_AREA_THRESHOLD = 0.30
OBSTRUCTION_TRIGGER_FRAMES = 4
OBSTRUCTION_VISUAL_MAX_AGE_S = 0.25

# 退出清除使用更低面积阈值形成迟滞；连续 N 个“新视觉帧”满足
# 未检测到目标或面积低于阈值，才认为清除完成。
OBSTRUCTION_EXIT_AREA_THRESHOLD = 0.10
OBSTRUCTION_CLEAR_FRAMES = 15

# 追踪阻塞物的 P 控制增益（°/帧 per unit）
OBSTRUCTION_TRACK_GAIN_M1 = 4.0
OBSTRUCTION_TRACK_GAIN_M2 = 3.5
OBSTRUCTION_TRACK_GAIN_M0 = 2.0   # 前进（向阻塞物靠近）

# 前进追踪时的深度阈值：阻塞物深度 < 此值时停止前进
OBSTRUCTION_MIN_DEPTH_MM = 10.0

# 返回断点时的到位容差（°）
RETURN_TOLERANCE_DEG = 5.0
OBSTRUCTION_RETURN_STABLE_FRAMES = 3

# 返回断点时的最大电机速度（°/帧）
RETURN_MAX_DELTA = {0: 8.0, 1: 5.0, 2: 5.0}

# ── 全流程自主巡检 ─────────────────────────────────────────────────
# 两条离散轨迹在三轴电机角度空间中的“近似重复点”容差。
INSPECTION_MATCH_TOLERANCE = (20.0, 12.0, 30.0)
INSPECTION_MATCH_MAX_NORMALIZED_DISTANCE = 1.5

# 叶段演示必须具有足够的 M0 前进行程。这样可排除已经返回 TR 原点的
# 闭环/调试记录，避免把它误当作“到达叶段”的单条轨迹。
INSPECTION_MIN_FORWARD_TRAVEL_M0 = 100.0

# 全流程从 TR 固定原点出发。UI 在启动前按此容差检查真实三轴角度。
INSPECTION_ORIGIN_TOLERANCE = (5.0, 3.0, 5.0)
