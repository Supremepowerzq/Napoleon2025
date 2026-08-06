# IEEE RA-L 论文图片生成提示词 (Enhanced Prompts for Scientific Figure Generation)

> 生成日期: 2026-07-16
> 论文: An Embodied Autonomous Bronchoscope Platform with Multimodal Perception and Goal-Conditioned Imitation Learning

---

## Fig 1: System Overview (系统总架构)

### 基础描述
**功能**: 展示整个支气管镜机器人自动导航系统的四层架构
**版式**: 双栏通栏（full-column width），推荐16:9或7:4比例

### 高级提示词 (DALL-E/Midjourney格式)

```
Scientific architecture diagram for an embodied autonomous bronchoscope system, 
professional IEEE journal style, clean minimalist design, white background.

Show a hierarchical 4-layer architecture with flowing arrows indicating 
data flow between layers:

LAYER D (Top - Interaction): Three interaction modules arranged horizontally:
- Tkinter GUI with labeled buttons and status display
- Xbox Controller with joystick icon and wireless symbol
- Baidu ASR (Voice) with microphone icon and Chinese text "语音控制"

LAYER C (Policy): Two central modules:
- BronchusPolicy box (GRU icon inside) with label "0.4M params, <5ms"
- AutoNavController box with three sub-modules:
  * SequencePlayer (trajectory icon)
  * JunctionGuide (bifurcation icon)
  * ObstructionHandler (warning icon)

LAYER B (Perception): Three perception modules in a row:
- UNet Segmentation: brain/network icon with "4-class: lumen, bifurcation, mucus, background"
- Depth-Anything-V2: depth layers icon with "ViT-L, metric depth (mm)"
- DepthPathFinder: 3x5 grid icon with "left/center/right path analysis"

LAYER A (Bottom - Hardware): Robotic bronchoscope hardware:
- 3-DoF RMD Motors labeled M0 (advance), M1 (L/R deflection), M2 (U/D deflection)
- Endoscopic camera icon with USB connection
- Bronchoscope tip illustration showing 4-way deflection

CONNECTIONS:
- Vertical dashed arrows connecting layers top to bottom
- Horizontal arrows within each layer
- Circular arrows showing 20Hz feedback loop on right side
- Color-coded: Hardware blue (#1E88E5), Perception green (#43A047), 
  Policy orange (#FB8C00), Interaction purple (#8E24AA)

STYLE: Professional scientific diagram with consistent font sizes,
clean sans-serif labels, IEEE two-column compatible, 
no decorative elements, scientific illustration quality.
```

### LaTeX/TikZ 建议代码段

```latex
\begin{figure}[t]
  \centering
  \caption{Overall system architecture. The four layers communicate 
           through a thread-safe feature dictionary updated at 20\,Hz: 
           (a)~hardware actuation (3-DoF RMD motors); 
           (b)~multimodal perception (UNet, Depth-Anything-V2, DepthPathFinder); 
           (c)~policy layer (BronchusPolicy and hierarchical AutoNavController); 
           and (d)~interaction (GUI, joystick, voice ASR).}
  \label{fig:system}
\end{figure}
```

---

## Fig 2: Multimodal Perception Pipeline (多模态感知流水线)

### 基础描述
**功能**: 展示从原始内镜图像到20维观测向量的多模态感知流水线
**版式**: 单栏四列或2x2子图

### 高级提示词 (DALL-E/Midjourney格式)

```
Scientific visualization of bronchoscopy multimodal perception pipeline,
professional IEEE journal style, clean white background.

FOUR PANELS arranged in 2x2 grid with equal spacing:

PANEL A (Top-Left): Raw Endoscopic Frame
- Bronchoscopic view of bronchial tree with realistic tissue texture
- Circular ROI overlay with dashed border, labeled "788px diameter"
- Anatomical labels: "lumen", "bifurcation/carina"
- Subtle specular highlights on wet tissue surface
- Style: realistic medical imaging, slight vignette

PANEL B (Top-Right): UNet Semantic Segmentation Overlay
- Same bronchial scene as panel A
- Color-coded semantic segmentation:
  * Green (#00C853): Lumen regions, semi-transparent overlay
  * Red (#D50000): Bifurcation/carina detection
  * Orange (#FF6D00): Mucus/stone deposits
  * Black: Background
- Legend box in corner: "■ Lumen  ■ Bifurcation  ■ Mucus"
- Segmentation confidence shown as opacity

PANEL C (Bottom-Left): Depth Map from Depth-Anything-V2
- Monocular depth estimation in false color
- Color scale: cool blue (near, ~5mm) to warm red (far, ~300mm)
- Circular mask matching ROI from panel A
- Depth contours overlaid as subtle iso-lines
- Scale bar showing depth range in mm
- Style: scientific colormap visualization

PANEL D (Bottom-Right): DepthPathFinder Three-Column Analysis
- 3D-like bar chart showing path depths at bifurcation
- Three vertical bars labeled: "Left Path", "Center Path", "Right Path"
- Bar heights proportional to depth: Left=85mm, Center=120mm, Right=95mm
- Overlay of 3x5 grid on miniature bronchoscopic view
- Arrow indicating "deepest path (center)" selected by policy
- Axes labeled: "Path Depth (mm)" and "Left | Center | Right"

SUBPLOT LABELS: (a), (b), (c), (d) in top-left corner of each panel
ARROWS: Curved arrow from (a)→(b) labeled "UNet", (a)→(c) labeled "DAM-V2",
        (c)→(d) labeled "Grid Analysis"

STYLE: Consistent panel sizes, professional typography,
IEEE-compatible margins, scientific figure standards.
```

### LaTeX 子图结构建议

```latex
\begin{figure}[t]
  \centering
  \begin{subfigure}[b]{0.23\textwidth}
    \centering
    \includegraphics[]{panel_a_raw.png}
    \caption{Raw frame}
    \label{fig:perception:a}
  \end{subfigure}
  \hfill
  \begin{subfigure}[b]{0.23\textwidth}
    \centering
    \includegraphics[]{panel_b_seg.png}
    \caption{UNet overlay}
    \label{fig:perception:b}
  \end{subfigure}
  \hfill
  \begin{subfigure}[b]{0.23\textwidth}
    \centering
    \includegraphics[]{panel_c_depth.png}
    \caption{Depth map}
    \label{fig:perception:c}
  \end{subfigure}
  \hfill
  \begin{subfigure}[b]{0.23\textwidth}
    \centering
    \includegraphics[]{panel_d_path.png}
    \caption{Path analysis}
    \label{fig:perception:d}
  \end{subfigure}
  \caption{Perception pipeline.}
  \label{fig:perception}
\end{figure}
```

---

## Fig 3: BronchusPolicy Network Architecture (策略网络架构)

### 基础描述
**功能**: 展示BronchusPolicy的双头GRU网络结构
**版式**: 单栏，居中布局

### 高级提示词 (DALL-E/Midjourney格式)

```
Scientific neural network architecture diagram for goal-conditioned policy,
professional IEEE journal style, clean white background, left-to-right data flow.

CENTER-LEFT: INPUT MODULE (8 frames + goal token)
- 8 small rectangular boxes stacked vertically, each labeled "o_t" with t=1...8
- Inside each box: mini vector representation [7 visual + 6 depth + 3 pos + 3 vel + 1 dt]
- Arrow pointing right from stack to ObsEncoder
- Below stack: Goal Token box labeled "g ∈ {0,...,9}" with arrow to Goal Embedding

CENTER: TWO PARALLEL PATHS

PATH 1 - ObsEncoder:
- Rectangle labeled "MLP Encoder" with internal layers: 20→128→128
- LayerNorm and ELU labels inside
- Output: 128-dim vector per frame

PATH 2 - Goal Embedding:
- Rectangle labeled "Goal Embed" 
- Internal: lookup table [10×16]
- Output: 16-dim vector

CONVERGENCE:
- Bracket grouping ObsEncoder outputs (8×128) with Goal Embedding (16)
- Arrow labeled "Concat → 144-dim" pointing to GRU

CENTER-RIGHT: TEMPORAL PROCESSING
- Large rectangle: "2-Layer GRU"
- Hidden size = 256
- Dropout = 0.1 between layers (shown as dashed line)
- Hidden state arrows flowing through: h_{t-1} → GRU → h_t
- Label: "T=8 time steps"

RIGHT SIDE: DUAL OUTPUT HEADS (from final h_T)

HEAD 1 - Action Head:
- Rectangle: "MLP_act: 256→128→3"
- Tanh activation icon
- Output: a_t ∈ [-1,1]³ labeled "Δm₀, Δm₁, Δm₂"
- Arrow labeled "×[5°,3°,3°]/frame" pointing to motor commands

HEAD 2 - Task Head:
- Rectangle: "MLP_task: 256→64→2"
- Output: z_t ∈ ℝ² labeled "navigate / clear"
- Softmax icon

BOXES AND CONNECTIONS:
- Rounded rectangles for modules with drop shadow
- Straight arrows with arrowheads for data flow
- Dimension annotations in gray text
- Color coding:
  * Input: Light blue (#BBDEFB)
  * Encoder/Embedding: Light green (#C8E6C9)
  * GRU: Light orange (#FFE0B2)
  * Heads: Light purple (#E1BEE7)

ANNOTATIONS:
- Top: "BronchusPolicy (~0.4M parameters)"
- Bottom: "Inference <5ms on RTX 3090"

STYLE: Clean network diagram, IEEE scientific illustration,
balanced whitespace, professional typography.
```

### PyTorch 简化代码参考

```python
# For reference, actual architecture:
class BronchusPolicy(nn.Module):
    def __init__(self, obs_dim=20, goal_dim=10, hidden_dim=256):
        super().__init__()
        self.obs_encoder = nn.Sequential(
            nn.Linear(obs_dim, 128), nn.LayerNorm(128), nn.ELU(),
            nn.Linear(128, 128), nn.LayerNorm(128), nn.ELU()
        )  # 128-dim output per frame
        self.goal_embed = nn.Embedding(goal_dim, 16)  # 10 goals × 16-dim
        self.gru = nn.GRU(input_size=144, hidden_size=hidden_dim, 
                         num_layers=2, dropout=0.1, batch_first=True)
        self.action_head = nn.Sequential(
            nn.Linear(hidden_dim, 128), nn.ELU(),
            nn.Linear(128, 3), nn.Tanh()
        )
        self.task_head = nn.Sequential(
            nn.Linear(hidden_dim, 64), nn.ELU(),
            nn.Linear(64, 2)
        )
```

---

## Fig 4: Experimental Results (实验结果)

### 基础描述
**功能**: 展示导航成功率对比、轨迹可视化、和电机控制时序图
**版式**: 双栏通栏，三个子图

### 高级提示词 (DALL-E/Midjourney格式)

```
Scientific results visualization for autonomous bronchoscopy navigation,
professional IEEE journal style, white background, three-panel layout.

PANEL A (Top-Left, ~40% width): Navigation Success Rate Comparison
- Grouped bar chart comparing 4 methods across 4 target bronchi
- X-axis: 4 targets (LMB, RMB, LUB, RML) — Left/Right Main Bronchi, Upper/Lower Lobe
- Y-axis: Success Rate (%) from 0 to 100
- 4 grouped bars per target:
  * Expert: solid gray (#757575), diagonal stripe pattern
  * Reactive: solid blue (#1E88E5)
  * BC (no-goal): solid orange (#FB8C00)
  * BC+goal (ours): solid green (#43A047) with bold border
- Error bars showing 95% confidence intervals
- Dashed horizontal line at y=80% labeled "Clinical threshold"
- Legend box: "■ Expert  ■ Reactive  ■ BC (no-goal)  ■ BC+goal (ours)"
- Y-axis label: "Navigation Success Rate (%)"
- Significant improvement annotations: arrow with "*" above BC+goal vs Reactive

PANEL B (Top-Right, ~55% width): Phantom Trajectory Visualization
- Top-down schematic of bronchial phantom
- Trachea at top, branching to LMB/RMB, then further to LUB/LLB/RUL/RML/RLL
- Two overlaid trajectories:
  * Reactive baseline: orange (#FB8C00) dashed line with small circles at waypoints
  * BC+goal (ours): green (#43A047) solid line with trajectory smoothness
- Start point: blue circle labeled "Start" at trachea entrance
- Target endpoints: star markers with target labels (LMB, RMB, etc.)
- Collision points: red X markers on orange trajectory only
- Phantom outline: light gray (#E0E0E0) with anatomical labels
- Subtle depth shading suggesting 3D structure

PANEL C (Bottom, full width): Motor Angle Time Series
- Three synchronized subplots sharing X-axis (Time, frames):
  
  Subplot M0: Advancement/Withdrawal
  - Y-axis: Motor angle (degrees), range [-100, 50]
  - Two lines: Reactive (orange dashed), BC+goal (green solid)
  - Shaded regions showing commanded velocity
  - Label: "M₀ (advancement) [°]"

  Subplot M1: Left-Right Deflection
  - Y-axis: Motor angle (degrees), range [-180, 180]
  - Same two lines as M0
  - Bifurcation events marked with vertical dashed lines and annotations
  - Label: "M₁ (L/R deflection) [°]"

  Subplot M2: Up-Down Deflection
  - Y-axis: Motor angle (degrees), range [-60, 60]
  - Same two lines as M0
  - Label: "M₂ (U/D deflection) [°]"

- X-axis: "Time (frames @ 20 Hz)" with tick marks every 100 frames
- Shared time annotations: vertical dashed lines at key events
- Jerk smoothness annotation: "BC+goal smoother trajectories (lower jerk)"

STYLE NOTES:
- Consistent color scheme throughout
- Professional axis labels and tick marks
- Clear panel labeling: (a), (b), (c)
- IEEE publication quality figures
- High resolution (300+ DPI equivalent)
```

---

## 通用规范 (Unified Specifications)

### 颜色系统 (Color Palette)
| 元素 | 颜色代码 | 用途 |
|------|----------|------|
| Layer 1 (导航) | #1E88E5 (蓝色) | 基础视觉伺服 |
| Layer 2 (修正) | #FB8C00 (橙色) | 进度加权修正 |
| Layer 3 (清除) | #43A047 (绿色) | 黏液清除功能 |
| 人工基线 | #757575 (灰色) | Expert/人工操作 |
| 失败/警告 | #D50000 (红色) | 错误/碰撞标记 |
| 背景 | #FFFFFF | 论文白底 |
| 边框 | #212121 | 文字和边框 |

### 字体规范
- 主标题: 12pt bold
- 子图标题: 10pt bold
- 轴标签: 9pt regular
- 注释文字: 8pt italic

### 分辨率和导出
- 主图: 300 DPI minimum
- 矢量格式优先: PDF, SVG, EPS
- 位图备份: PNG with alpha

### 关键设计原则
1. **科学准确性**: 图形元素必须准确反映论文描述
2. **清晰层次**: 使用颜色、大小、位置区分信息优先级
3. **一致性**: 同一论文中所有图片风格统一
4. **可读性**: 在黑白打印下也能清晰区分
5. **信息密度**: 展示关键数据，删除冗余装饰
