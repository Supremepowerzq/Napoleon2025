# 支气管镜机器人三维轨迹还原

本项目根据三电机动作序列，重建柔性支气管镜机器人头端的近似三维轨迹，并生成可在浏览器中旋转、缩放和查看逐帧信息的交互式三维页面。

项目同时提供一个可选的 CT 中心线配准模块：将重建轨迹与 3D Slicer 导出的气道中心线路径放到同一坐标系中，用于显示完整中心线、相关路径、中心线约束轨迹和原始运动学轨迹之间的差异。

> **重要说明**：本项目输出的是基于电机编码器和简化运动学模型的几何估计，不是电磁定位、光学定位或形状传感器测得的真实轨迹，不应直接用于临床决策。

## 1. 电机定义

| 电机 | 机器人动作 | 模型中的作用 |
|---|---|---|
| M0 | 前进／后退 | 沿当前头端前向移动 |
| M1 | 顺时针／逆时针旋转 | 改变 M2 所在的弯曲平面 |
| M2 | 头端上下弯曲 | 改变头端前向方向 |

核心状态包含头端位置 `p`、前向单位向量 `f` 和局部参考上向 `u`。每一帧先根据 M1 确定弯曲轴，再利用 Rodrigues 旋转公式更新方向，最后根据 M0 推进量更新位置：

```text
Δs = sign0 × ΔM0 × k0
Δθ = sign2 × ΔM2 × k2
f(k+1) = Rodrigues(f(k), bend_axis, Δθ)
p(k+1) = p(k) + Δs × f(k+1)
```

其中 `k0`、`k1`、`k2` 分别对应推进、滚转和弯曲传动关系，需要通过实验标定。

## 2. 项目结构

```text
bronchoscope-trajectory-reconstruction/
├── examples/
│   └── minimal_session.json       # 不含真实实验信息的合成示例
├── src/broncho_trajectory/
│   ├── kinematics.py              # 三电机正运动学
│   ├── geometry.py                # 三维向量与 Rodrigues 旋转
│   ├── centerline.py              # 中心线读取、弧长映射和配准
│   ├── visualization.py           # 交互式三维 HTML
│   ├── io.py                      # JSON / CSV 输出
│   └── cli.py                     # 命令行接口
└── tests/                         # 单元测试
```

## 3. 环境要求

- Python 3.9 或更高版本
- 核心计算仅使用 Python 标准库
- 三维 HTML 通过 CDN 加载 Plotly，打开页面时需要网络；计算和 JSON/CSV 输出不需要网络

## 4. 安装

```bash
git clone https://github.com/Supremepowerzq/bronchoscope-trajectory-reconstruction.git
cd bronchoscope-trajectory-reconstruction
python3 -m venv .venv
source .venv/bin/activate          # Windows 使用：.venv\Scripts\activate
python -m pip install -e .
```

## 5. 输入数据格式

输入文件是一个 JSON 对象，必须包含 `frames` 数组。每帧至少包含三个电机的绝对角度：

```json
{
  "session_name": "example",
  "path_name": "左主支气管",
  "frames": [
    {
      "timestamp": 0.0,
      "m0_angle": 0.0,
      "m1_angle": 0.0,
      "m2_angle": 0.0
    },
    {
      "timestamp": 0.05,
      "m0_angle": -5.0,
      "m1_angle": 10.0,
      "m2_angle": 2.0
    }
  ]
}
```

如果帧中提供 `delta_m0` 或 `delta_m2`，程序优先使用增量字段；否则由相邻帧的绝对角度计算增量。与轨迹无关的视觉观测字段可以保留，程序会忽略它们。

## 6. 快速开始：重建三维轨迹

运行合成示例：

```bash
broncho-trajectory reconstruct examples/minimal_session.json --output-dir outputs
```

程序会生成：

```text
outputs/minimal_session_trajectory.json   # 逐帧位置、方向和累计推进量
outputs/minimal_session_trajectory.csv    # 便于 MATLAB / Excel / pandas 分析
outputs/minimal_session_trajectory.html   # 可交互三维视图
```

在 macOS 中打开三维视图：

```bash
open outputs/minimal_session_trajectory.html
```

Windows 可直接双击 HTML，Linux 可使用浏览器打开。

## 7. 标定参数

默认参数只用于演示，真实系统必须标定：

```bash
broncho-trajectory reconstruct YOUR_DATA.json \
  --output-dir outputs \
  --m0-mm-per-degree 0.10 \
  --m1-roll-deg-per-degree 1.0 \
  --m2-tip-deg-per-degree 1.0 \
  --m0-sign -1 \
  --m1-sign 1 \
  --m2-sign 1 \
  --m1-zero-degree 0
```

建议按以下顺序标定：

1. **M0 推进尺度与正负号**：在近似直线的管道中记录已知推进距离。
2. **M1 零位与传动比**：标记镜身方向，比较电机角度与实际滚转角。
3. **M2 弯曲增益**：在多个弯曲角度下测量头端切向或曲率。
4. **回差和时延**：分别采集正向、反向和不同速度的数据。

## 8. 可选：与 CT 中心线配准

中心线路径使用 3D Slicer Markups JSON。多个 `--route` 按命令行顺序拼接；`--model-vtp` 用于显示完整中心线树，可以省略。

```bash
broncho-trajectory register YOUR_DATA.json \
  --route centerline/B0_TR_entry_to_carina.mrk.json \
  --route centerline/B2_LMB_carina_to_LMB.mrk.json \
  --model-vtp centerline_model.vtp \
  --output outputs/LMB_registered.html \
  --m0-mm-per-degree 0.10
```

输出包括：

- `LMB_registered.html`：完整中心线和轨迹的交互式三维叠加视图；
- `LMB_registered.metrics.json`：平均残差和最大残差。

当前实现首先将 M0 累计推进量映射到中心线弧长，再使用 Horn 四元数方法计算带尺度的最小二乘相似变换。该残差反映模型与中心线约束之间的一致性，不能当作真实定位误差。

## 9. 运行测试

```bash
python -m unittest discover -s tests -v
```

## 10. 已知局限

- 没有显式建模柔性镜身形变、摩擦、管壁接触、滞回和机械回差；
- M1/M2 的耦合被简化为局部坐标系中的滚转和弯曲；
- 仅依赖编码器时，误差会随轨迹长度累积；
- 中心线约束会提高几何一致性，但不等于获得了真实空间定位；
- 当前 VTP 解析器针对 3D Slicer 常见的 appended/base64/zlib PolyData 格式。

后续可加入视觉分叉定位、术中图像纠偏、标定参数优化和外部定位传感器，构成“CT 全局路径规划 + 视觉局部纠偏”的闭环导航系统。

## 11. 数据与隐私

`.gitignore` 默认排除数据集、VTP/NRRD/MRB 文件和生成的 HTML。提交前请再次确认仓库中不包含患者信息、实验原始数据、设备密钥、本机绝对路径或未获授权的模型文件。

## 12. 许可证

当前版本尚未指定开源许可证。在确定代码公开范围和知识产权归属之前，请勿默认将代码用于商业或临床用途。
