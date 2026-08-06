# AutoNav — 支气管自主巡检系统

## 设计目标

在现有手动操控系统基础上，实现**支气管主气道自主巡检**：从固定起点自主导航到指定部位，沿途清除阻塞物，兼顾安全性和实时性。

---

## 三层架构

```
┌─────────────────────────────────────────────────────────┐
│                    AutoNavController                     │
│  状态机: IDLE → NAVIGATING → CLEARING → RETURNING → DONE│
├──────────────────┬──────────────────┬────────────────────┤
│  Layer 1         │  Layer 2         │  Layer 3           │
│  SequencePlayer  │  JunctionGuide   │  ObstructionHandler│
│                  │                  │                    │
│  纯电机序列导航   │  岔口识别路线修正 │  阻塞物追踪清除     │
│  (无视觉依赖)     │  (P控制)         │  (断点续航)         │
└──────────────────┴──────────────────┴────────────────────┘
```

### Layer 1：SequencePlayer（纯电机序列）

- **原理**：从 `BC/expert_demos/` 中加载指定路径标签（TR/LMB/RMB…）的所有 JSON 演示
- 将每条演示提取为相对起点的累积角度序列
- 重采样到统一帧数后取逐帧平均，得到**平均参考轨迹**
- 导航时：`target[t] = start_angles + avg_cumulative[t]`
- **优点**：无需视觉信息，直接利用 80+ 条已采集演示，无需重新训练

### Layer 2：JunctionGuide（岔口修正）

- 读取视觉线程的 `bifur_cx`, `bifur_cy`, `bifur_area`
- 多个候选中只选择几何中心距离画面中心最近的岔口
- 路径前段保留低强度微调，随进度连续增强，并在 80% 后达到完整进度权重
- 中心距离 ≤0.30 时完整响应，0.30～0.75 连续衰减，≥0.75 才停止
- 当岔口面积 > 阈值时，用 P 控制在 M1/M2 上叠加有限总偏置；M0 不接受岔口修正
- 偏置始终相对于 Layer 1 原始目标，不逐帧累积；中心死区内不修正
- 使镜头沿岔口几何中心前进，减少对气管壁的碰撞
- 增益可在 `config.py` 中独立调节

独立“视觉自主”岔口追踪先使用像素误差速度闭环验证：10px死区内停止，误差越大速度越快；M1/X与M2/Y使用不同速度曲线。该闭环不乘Layer 2导航距离权重，避免远距离输出低于电机启动阈值。

### Layer 3：ObstructionHandler（阻塞物处理）

- 检测到阻塞物 (`obstruction_detected=True`) 时：
  1. **保存断点**：记录当前电机角度 + 序列步骤索引
  2. **切换 CLEARING**：驱动镜头向阻塞物中心移动并前进
  3. 连续 N 帧（默认 15 帧）未检测到阻塞物 → **RETURNING**
  4. 电机回到断点位置后 → **恢复 NAVIGATING**，从断点步骤继续序列

---

## 文件结构

```
AutoNav/
├── README.md               # 本文件
├── __init__.py             # 公共接口导出
├── config.py               # 所有可调参数（增益、限位、阈值）
├── sequence_player.py      # Layer 1：平均轨迹计算与导航
├── junction_guide.py       # Layer 2：岔口引导修正
├── obstruction_handler.py  # Layer 3：阻塞物状态机
└── nav_controller.py       # 统一控制器（集成三层）
```

---

## 快速集成

在主程序（`main2026-Xhandwriting-AC-Auto...py`）中添加：

```python
# 1. 导入
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))  # 若从子目录运行
from AutoNav import AutoNavController, NavMode
from BC.data_collector import get_visual_features

# 2. 初始化（机器人 init 阶段）
autonav = AutoNavController(demo_dir="BC/expert_demos", motor_group=motor_group)

# 3. 启动（切换到 AI_AUTO 模式时）
autonav.start(path_label=2, current_motor_angles=motor_group.get_cached_angles())

# 4. 控制循环（每帧）
if state == "AI_AUTO":
    vis = get_visual_features()
    dec = autonav.step(motor_angles=motor_group.get_cached_angles(), visual=vis)
    if dec.active:
        motor_group.set_motor_position(0, dec.target_m0, max_speed=25)
        motor_group.set_motor_position(1, dec.target_m1, max_speed=15)
        motor_group.set_motor_position(2, dec.target_m2, max_speed=15)
    # UI 显示
    status = autonav.get_status()
    ui_label.setText(f"模式:{status['mode']}  进度:{status['progress']*100:.0f}%")

# 5. 停止
autonav.stop()
```

---

## 参数调节指南（config.py）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `PLAYBACK_SPEED` | 1.0 | 序列导航速度，建议先用 0.5 验证安全 |
| `PLAYBACK_MAX_DELTA` | {0:6, 1:4, 2:4} | 单帧最大角度增量（°），安全保护 |
| `BIFUR_AREA_THRESHOLD` | 0.01 | 岔口识别激活面积，越小越灵敏 |
| `BIFUR_GAIN_M1` | 3.0 | 岔口左右修正增益，若方向反了改为负值 |
| `BIFUR_GAIN_M2` | 2.5 | 岔口上下修正增益 |
| `OBSTRUCTION_CLEAR_FRAMES` | 15 | 确认阻塞物清除所需的连续空白帧数 |
| `OBSTRUCTION_TRACK_GAIN_M1` | 4.0 | 阻塞物追踪左右增益 |
| `RETURN_TOLERANCE_DEG` | 5.0 | 返回断点的到位容差（°） |

---

## 前提条件

1. `BC/expert_demos/` 中需有目标路径的 JSON 演示（最少 3 条，越多越稳定）
2. 每次使用**必须从固定起点**出发（电机回到初始零位）
3. 视觉线程需调用 `BC.data_collector.update_visual_features()` 更新 `bifur_*` 和 `obstruction_*` 字段
4. Layer 1 可独立运行（无需视觉），Layer 2/3 需要视觉线程在线

---

## 验证与测试

### 离线单元测试

```bash
# 进入 2026 目录
cd 2026

# 运行纯逻辑测试（无需机器人）
python test_autonav.py

# 运行模拟验证测试（模拟更多场景）
python test_autonav_sim.py
```

### 上机验证顺序

详见 [VERIFICATION_GUIDE.md](./VERIFICATION_GUIDE.md)，建议按以下顺序验证：

| 阶段 | 内容 | 前提 |
|------|------|------|
| 1 | Layer 1 纯序列导航 | 演示数据完整 |
| 2 | Layer 2 岔口修正 | Layer 1 通过 |
| 3 | Layer 3 阻塞物清除 | Layer 1+2 通过 |
| 4 | 完整巡检 | 全部通过 |

---

## 路径标签对照

| path_label | YOLO 代码 | 中文名 |
|-----------|-----------|--------|
| 1 | TR | 气管 |
| 2 | LMB | 左主支气管 |
| 3 | RMB | 右主支气管 |
| 4 | LUB | 左上叶 |
| 5 | LLB | 左下叶 |
| 6 | RUL | 右上叶 |
| 7 | BI | 中间支气管 |
| 8 | RML | 右中叶 |
| 9 | RLL | 右下叶 |
