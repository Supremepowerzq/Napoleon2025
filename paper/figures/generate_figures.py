"""
Scientific Figure Generator for IEEE RA-L Paper
================================================
生成支气管镜自动导航系统的科研论文图片

使用方法:
    python generate_figures.py [--figure {1,2,3,4,all}] [--output_dir ./output]

依赖安装:
    pip install matplotlib numpy scipy

作者: Z.Q. (Napoleon2025)
日期: 2026-07-16
"""

import argparse
import os
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle, Rectangle, FancyArrow
import matplotlib.lines as mlines
import numpy as np
from matplotlib.collections import LineCollection
import matplotlib.gridspec as gridspec

# 设置中文字体支持（如果系统有中文字体）
plt.rcParams['font.family'] = ['DejaVu Sans', 'SimHei', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['figure.dpi'] = 150
plt.rcParams['savefig.dpi'] = 300
plt.rcParams['savefig.bbox'] = 'tight'

# 科研配色方案
COLORS = {
    'hardware': '#1E88E5',      # 蓝色 - 硬件层
    'perception': '#43A047',    # 绿色 - 感知层
    'policy': '#FB8C00',        # 橙色 - 策略层
    'interaction': '#8E24AA',   # 紫色 - 交互层
    'expert': '#757575',        # 灰色 - 专家基线
    'reactive': '#1E88E5',      # 蓝色 - 视觉伺服
    'bc_no_goal': '#FB8C00',    # 橙色 - 无目标BC
    'bc_goal': '#43A047',       # 绿色 - BC+目标(ours)
    'failure': '#D50000',       # 红色 - 失败
    'background': '#FFFFFF',    # 白色背景
    'text': '#212121',          # 深灰文字
    'grid': '#E0E0E0',          # 浅灰网格
}


def generate_fig1_system_overview(ax=None, save_path=None):
    """
    生成 Fig 1: 系统总架构图
    
    展示四层架构:
    - Layer D: 交互层 (GUI, 游戏手柄, 语音)
    - Layer C: 策略层 (BronchusPolicy, AutoNavController)
    - Layer B: 感知层 (UNet, DAM-V2, DepthPathFinder)
    - Layer A: 硬件层 (3-DoF电机, 内镜相机)
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(12, 8))
    
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 8)
    ax.axis('off')
    
    # 层的位置定义
    layer_y = {
        'D': 6.5,  # 交互层
        'C': 4.8,  # 策略层
        'B': 3.0,  # 感知层
        'A': 1.2,  # 硬件层
    }
    layer_height = 1.2
    layer_width = 9.0
    
    # 绘制各层背景
    for name, y in layer_y.items():
        box = FancyBboxPatch(
            (0.5, y - layer_height/2), layer_width, layer_height,
            boxstyle="round,pad=0.05,rounding_size=0.1",
            facecolor=COLORS.get(name.lower() if name != 'D' else 'interaction', '#F5F5F5'),
            edgecolor='#BDBDBD', linewidth=1.5, alpha=0.3
        )
        ax.add_patch(box)
    
    # Layer A: 硬件层
    ax.text(0.7, layer_y['A'], 'A', fontsize=14, fontweight='bold', va='center')
    ax.text(1.0, layer_y['A'], 'Hardware Layer (20 Hz)', fontsize=11, fontweight='bold', va='center')
    
    # 电机模块
    for i, (motor, label) in enumerate([
        ('M₀', 'Adv/Ret'), ('M₁', 'L/R Deflect'), ('M₂', 'U/D Deflect')
    ]):
        x = 3.0 + i * 2.0
        rect = FancyBboxPatch((x - 0.7, y - 0.4), 1.4, 0.8,
                              boxstyle="round,pad=0.03",
                              facecolor=COLORS['hardware'], alpha=0.8)
        ax.add_patch(rect)
        ax.text(x, y + 0.05, motor, ha='center', va='center', fontsize=10, 
                fontweight='bold', color='white')
        ax.text(x, y - 0.2, label, ha='center', va='center', fontsize=7, color='white')
    
    # 内镜相机
    cam_x = 8.5
    circle = Circle((cam_x, layer_y['A']), 0.4, facecolor='#424242', alpha=0.8)
    ax.add_patch(circle)
    # 使用简化的图标表示相机（避免emoji字体问题）
    ax.text(cam_x, layer_y['A'], 'o', ha='center', va='center', fontsize=14, 
            color='white', fontweight='bold')
    ax.text(cam_x, layer_y['A'] - 0.6, 'USB Camera', ha='center', va='center', fontsize=7)
    
    # Layer B: 感知层
    ax.text(0.7, layer_y['B'], 'B', fontsize=14, fontweight='bold', va='center')
    ax.text(1.0, layer_y['B'], 'Perception Layer', fontsize=11, fontweight='bold', va='center')
    
    perception_modules = [
        ('UNet', 'Segmentation\n4-class'),
        ('DAM-V2', 'Depth Est.\nViT-L, metric'),
        ('DepthPath\nFinder', '3×5 Grid\nPath Analysis')
    ]
    for i, (name, desc) in enumerate(perception_modules):
        x = 3.0 + i * 2.2
        rect = FancyBboxPatch((x - 0.9, layer_y['B'] - 0.45), 1.8, 0.9,
                              boxstyle="round,pad=0.03",
                              facecolor=COLORS['perception'], alpha=0.8)
        ax.add_patch(rect)
        ax.text(x, layer_y['B'] + 0.1, name, ha='center', va='center', 
                fontsize=9, fontweight='bold', color='white')
        ax.text(x, layer_y['B'] - 0.2, desc, ha='center', va='center', 
                fontsize=7, color='white', linespacing=1.2)
    
    # 20维观测向量输出
    obs_box = FancyBboxPatch((8.0, layer_y['B'] - 0.35), 1.2, 0.7,
                             boxstyle="round,pad=0.03",
                             facecolor='#FFC107', alpha=0.9)
    ax.add_patch(obs_box)
    ax.text(8.6, layer_y['B'] + 0.1, 'obs', ha='center', va='center', 
            fontsize=9, fontweight='bold', color='black')
    ax.text(8.6, layer_y['B'] - 0.15, '20-dim', ha='center', va='center', 
            fontsize=7, color='black')
    
    # Layer C: 策略层
    ax.text(0.7, layer_y['C'], 'C', fontsize=14, fontweight='bold', va='center')
    ax.text(1.0, layer_y['C'], 'Policy Layer', fontsize=11, fontweight='bold', va='center')
    
    # BronchusPolicy
    policy_rect = FancyBboxPatch((2.5, layer_y['C'] - 0.45), 2.2, 0.9,
                                  boxstyle="round,pad=0.03",
                                  facecolor=COLORS['policy'], alpha=0.8)
    ax.add_patch(policy_rect)
    ax.text(3.6, layer_y['C'] + 0.1, 'BronchusPolicy', ha='center', va='center', 
            fontsize=9, fontweight='bold', color='white')
    ax.text(3.6, layer_y['C'] - 0.2, 'GRU, 0.4M, <5ms', ha='center', va='center', 
            fontsize=7, color='white')
    
    # AutoNavController
    controller_rect = FancyBboxPatch((5.2, layer_y['C'] - 0.45), 3.5, 0.9,
                                    boxstyle="round,pad=0.03",
                                    facecolor=COLORS['policy'], alpha=0.8)
    ax.add_patch(controller_rect)
    ax.text(6.95, layer_y['C'] + 0.15, 'AutoNavController', ha='center', va='center', 
            fontsize=9, fontweight='bold', color='white')
    
    # 子模块
    submodules = ['Sequence\nPlayer', 'Junction\nGuide', 'Obstruction\nHandler']
    for i, sub in enumerate(submodules):
        sx = 5.5 + i * 1.1
        sub_rect = Rectangle((sx - 0.4, layer_y['C'] - 0.35), 0.8, 0.5,
                              facecolor='white', alpha=0.9, edgecolor='white')
        ax.add_patch(sub_rect)
        ax.text(sx, layer_y['C'] - 0.1, sub, ha='center', va='center', 
                fontsize=6, color='#E65100', linespacing=1.1)
    
    # Layer D: 交互层
    ax.text(0.7, layer_y['D'], 'D', fontsize=14, fontweight='bold', va='center')
    ax.text(1.0, layer_y['D'], 'Interaction Layer', fontsize=11, fontweight='bold', va='center')
    
    interaction_modules = [
        ('Tkinter GUI', 'GUI'),
        ('Xbox Controller', 'JOY'),
        ('Baidu ASR\n(Voice)', 'ASR')
    ]
    for i, (name, icon) in enumerate(interaction_modules):
        x = 3.0 + i * 2.2
        rect = FancyBboxPatch((x - 0.9, layer_y['D'] - 0.4), 1.8, 0.8,
                              boxstyle="round,pad=0.03",
                              facecolor=COLORS['interaction'], alpha=0.8)
        ax.add_patch(rect)
        ax.text(x, layer_y['D'] + 0.05, icon, ha='center', va='center', fontsize=10)
        ax.text(x, layer_y['D'] - 0.2, name, ha='center', va='center', 
                fontsize=7, color='white', linespacing=1.1)
    
    # 右侧反馈回路标注
    ax.annotate('', xy=(9.8, layer_y['D']), xytext=(9.8, layer_y['A']),
                arrowprops=dict(arrowstyle='<->', color='#9E9E9E', lw=1.5))
    ax.text(9.9, 3.8, '20 Hz\nFeedback', ha='left', va='center', fontsize=7, 
            color='#757575', rotation=90)
    
    # 层级连接箭头
    for y1, y2 in [(layer_y['A'] + 0.6, layer_y['B'] - 0.6),
                   (layer_y['B'] + 0.6, layer_y['C'] - 0.6),
                   (layer_y['C'] + 0.6, layer_y['D'] - 0.6)]:
        ax.annotate('', xy=(5, y2), xytext=(5, y1),
                    arrowprops=dict(arrowstyle='->', color='#757575', lw=1.2))
    
    # 标题
    ax.text(5, 7.5, 'Overall System Architecture', fontsize=14, fontweight='bold',
            ha='center', va='center')
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight', 
                   facecolor='white', edgecolor='none')
        print(f"Fig 1 saved to: {save_path}")
    
    return ax


def generate_fig2_perception(ax=None, save_path=None):
    """
    生成 Fig 2: 多模态感知流水线
    
    四个子图:
    (a) 原始内镜图像
    (b) UNet语义分割结果
    (c) Depth-Anything-V2深度图
    (d) DepthPathFinder三列路径分析
    """
    if ax is None:
        fig, ax = plt.subplots(2, 2, figsize=(12, 10))
        ax = ax.flatten()
    else:
        ax = np.asarray(ax).flatten()
    
    # 模拟支气管树图像数据
    np.random.seed(42)
    
    # Panel (a): 原始内镜图像
    ax[0].set_title('(a) Raw Endoscopic Frame', fontsize=11, fontweight='bold', pad=10)
    
    # 创建模拟支气管树图像
    img = np.zeros((300, 300, 3))
    center = (150, 150)
    
    # 背景（组织颜色渐变）
    for i in range(300):
        for j in range(300):
            dist = np.sqrt((i - center[0])**2 + (j - center[1])**2)
            base = 80 + 40 * (1 - dist/150)
            img[i, j] = [base * 0.9, base * 0.85, base * 0.8]
    
    # 添加管腔结构
    for angle in [0, 60, 120, 180, 240, 300]:
        rad = np.radians(angle)
        for r in range(30, 120):
            x = int(center[0] + r * np.sin(rad) * 0.4)
            y = int(center[1] - r * np.cos(rad))
            if 0 <= x < 300 and 0 <= y < 300:
                brightness = 140 + 20 * np.sin(r * 0.1)
                img[y, x] = [brightness * 0.95, brightness * 0.9, brightness * 0.85]
    
    ax[0].imshow(img.astype(np.uint8))
    
    # 圆形ROI
    circle = Circle(center, 140, fill=False, edgecolor='#2196F3', linewidth=2, 
                    linestyle='--', label='ROI (788px)')
    ax[0].add_patch(circle)
    ax[0].text(10, 20, 'Circular ROI', fontsize=8, color='#2196F3', 
               fontweight='bold', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # 解剖标注
    ax[0].text(100, 250, 'Lumen', fontsize=9, color='#4CAF50', fontweight='bold')
    ax[0].text(200, 150, 'Bifurcation', fontsize=9, color='#F44336', fontweight='bold')
    ax[0].set_axis_off()
    
    # Panel (b): UNet语义分割
    ax[1].set_title('(b) UNet Segmentation Overlay', fontsize=11, fontweight='bold', pad=10)
    ax[1].imshow(img.astype(np.uint8), alpha=0.5)
    
    # 添加分割掩码
    seg_mask = np.zeros((300, 300, 4))
    
    # 腔体掩码（绿色）
    for angle in [0, 60, 120, 180, 240, 300]:
        rad = np.radians(angle)
        for r in range(25, 100):
            x = int(center[0] + r * np.sin(rad) * 0.4)
            y = int(center[1] - r * np.cos(rad))
            if 0 <= x < 300 and 0 <= y < 300:
                seg_mask[y, x] = [0, 0.78, 0, 0.5]  # 绿色半透明
    
    # 分岔处掩码（红色）
    for i in range(center[0]-20, center[0]+20):
        for j in range(center[1]-20, center[1]+20):
            dist = np.sqrt((i - center[0])**2 + (j - center[1])**2)
            if dist < 25:
                seg_mask[j, i] = [0.83, 0, 0, 0.6]  # 红色半透明
    
    # 黏液掩码（橙色）
    mucus_pos = [(180, 220), (200, 200), (190, 210)]
    for mx, my in mucus_pos:
        for i in range(mx-15, mx+15):
            for j in range(my-15, my+15):
                if 0 <= i < 300 and 0 <= j < 300:
                    dist = np.sqrt((i - mx)**2 + (j - my)**2)
                    if dist < 12:
                        seg_mask[j, i] = [1.0, 0.43, 0, 0.5]  # 橙色半透明
    
    ax[1].imshow(seg_mask)
    
    # 图例
    legend_elements = [
        mpatches.Patch(color='#00C853', alpha=0.7, label='Lumen'),
        mpatches.Patch(color='#D50000', alpha=0.7, label='Bifurcation'),
        mpatches.Patch(color='#FF6D00', alpha=0.7, label='Mucus'),
    ]
    ax[1].legend(handles=legend_elements, loc='upper right', fontsize=8)
    ax[1].set_axis_off()
    
    # Panel (c): 深度图
    ax[2].set_title('(c) Depth-Anything-V2 Depth Map', fontsize=11, fontweight='bold', pad=10)
    
    # 创建模拟深度图
    depth_img = np.zeros((300, 300))
    for i in range(300):
        for j in range(300):
            dist = np.sqrt((i - center[0])**2 + (j - center[1])**2)
            depth = min(300, max(5, dist * 1.5 + 30 + np.random.randn() * 10))
            depth_img[i, j] = depth
    
    depth_img_display = (depth_img - 5) / (300 - 5)  # 归一化
    im = ax[2].imshow(depth_img_display, cmap='coolwarm', vmin=0, vmax=1)
    
    # 颜色条
    cbar = plt.colorbar(im, ax=ax[2], fraction=0.046, pad=0.04)
    cbar.set_label('Depth (mm)', fontsize=8)
    
    # 添加等深线
    contour_levels = [50, 100, 150, 200]
    cs = ax[2].contour(depth_img_display, levels=contour_levels, 
                        colors='white', linewidths=0.5, alpha=0.5)
    ax[2].clabel(cs, inline=True, fontsize=7, fmt='%.0f')
    
    ax[2].set_axis_off()
    
    # Panel (d): DepthPathFinder三列分析
    ax[3].set_title('(d) DepthPathFinder Path Analysis', fontsize=11, fontweight='bold', pad=10)
    
    # 创建三列路径深度条形图
    paths = ['Left\nPath', 'Center\nPath', 'Right\nPath']
    depths = [85, 142, 98]  # 模拟深度值
    colors_bar = ['#1E88E5', '#43A047', '#1E88E5']
    
    bars = ax[3].bar(paths, depths, color=colors_bar, edgecolor='black', linewidth=1.5)
    
    # 在条上添加数值标签
    for bar, depth in zip(bars, depths):
        height = bar.get_height()
        ax[3].text(bar.get_x() + bar.get_width()/2., height + 3,
                   f'{depth}mm', ha='center', va='bottom', fontsize=10, fontweight='bold')
    
    # 高亮最深处（中心路径）
    ax[3].annotate('Selected\n(Deepest)', xy=(1, 142), xytext=(1.5, 170),
                   fontsize=9, ha='center',
                   arrowprops=dict(arrowstyle='->', color='#43A047', lw=1.5),
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='#43A047', 
                            edgecolor='none', alpha=0.2))
    
    ax[3].set_ylabel('Path Depth (mm)', fontsize=10)
    ax[3].set_xlabel('Bifurcation Paths', fontsize=10)
    ax[3].set_ylim(0, 200)
    ax[3].grid(axis='y', alpha=0.3)
    
    # 添加3x5网格示意
    grid_ax = ax[3].inset_axes([0.65, 0.5, 0.3, 0.4])
    grid_ax.set_xlim(0, 3)
    grid_ax.set_ylim(0, 5)
    
    for i in range(3):
        for j in range(5):
            rect = Rectangle((i, j), 1, 1, fill=False, edgecolor='#BDBDBD', linewidth=0.5)
            grid_ax.add_patch(rect)
    
    grid_ax.set_title('3×5 Grid', fontsize=7)
    grid_ax.set_xticks([0.5, 1.5, 2.5])
    grid_ax.set_xticklabels(['L', 'C', 'R'], fontsize=6)
    grid_ax.set_yticks([])
    grid_ax.set_facecolor('#F5F5F5')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight',
                   facecolor='white', edgecolor='none')
        print(f"Fig 2 saved to: {save_path}")
    
    return ax


def generate_fig3_network(ax=None, save_path=None):
    """
    生成 Fig 3: BronchusPolicy网络架构
    
    展示:
    - 8帧观测输入 + 目标token
    - MLP编码器
    - 目标嵌入
    - 双层GRU
    - 动作头 + 任务头
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(14, 8))
    
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 8)
    ax.axis('off')
    
    # ========== 左侧: 输入 ==========
    # 8帧观测窗口
    ax.text(1.5, 6.5, 'Observation Window', fontsize=11, fontweight='bold', ha='center')
    ax.text(1.5, 6.1, 'T=8 frames', fontsize=9, ha='center', color='#757575')
    
    for i in range(8):
        y = 5.6 - i * 0.55
        rect = FancyBboxPatch((0.8, y - 0.2), 1.4, 0.4,
                              boxstyle="round,pad=0.02",
                              facecolor='#BBDEFB', edgecolor='#1976D2', linewidth=1)
        ax.add_patch(rect)
        ax.text(1.5, y, f'o_{{{i+1}}}', ha='center', va='center', fontsize=8)
        # 内部维度标注
        ax.text(2.3, y, '[20]', ha='left', va='center', fontsize=6, color='#757575')
    
    # 目标Token
    goal_box = FancyBboxPatch((0.8, 0.8), 1.4, 0.6,
                               boxstyle="round,pad=0.02",
                               facecolor='#FFE0B2', edgecolor='#FB8C00', linewidth=2)
    ax.add_patch(goal_box)
    ax.text(1.5, 1.1, 'Goal Token', ha='center', va='center', fontsize=8, fontweight='bold')
    ax.text(1.5, 0.9, 'g ∈ {0,...,9}', ha='center', va='center', fontsize=7, color='#E65100')
    
    # ========== 中间: 编码层 ==========
    # ObsEncoder
    enc_box = FancyBboxPatch((3.5, 1.5), 2.0, 5.5,
                              boxstyle="round,pad=0.05",
                              facecolor='#C8E6C9', edgecolor='#43A047', linewidth=2)
    ax.add_patch(enc_box)
    ax.text(4.5, 6.5, 'ObsEncoder', ha='center', va='center', fontsize=10, fontweight='bold')
    ax.text(4.5, 6.1, '(MLP Encoder)', ha='center', va='center', fontsize=8, color='#2E7D32')
    ax.text(4.5, 5.5, '20 → 128 → 128', ha='center', va='center', fontsize=7)
    ax.text(4.5, 5.0, '+ LayerNorm + ELU', ha='center', va='center', fontsize=7, color='#757575')
    
    # 输出维度
    ax.text(5.6, 4.0, '128-dim', ha='center', va='center', fontsize=7, color='#757575')
    
    # Goal Embedding
    goal_enc_box = FancyBboxPatch((3.5, 0.4), 2.0, 0.8,
                                   boxstyle="round,pad=0.02",
                                   facecolor='#FFE0B2', edgecolor='#FB8C00', linewidth=1)
    ax.add_patch(goal_enc_box)
    ax.text(4.5, 0.8, 'Goal Embed', ha='center', va='center', fontsize=8, fontweight='bold')
    ax.text(4.5, 0.55, '10 × 16', ha='center', va='center', fontsize=7, color='#E65100')
    
    # 连接箭头 (Obs → Encoder)
    for i in range(8):
        y_obs = 5.6 - i * 0.55
        ax.annotate('', xy=(3.6, 5.0), xytext=(2.2, y_obs),
                    arrowprops=dict(arrowstyle='->', color='#1976D2', lw=1, alpha=0.6))
    
    # Goal → Goal Embed
    ax.annotate('', xy=(4.5, 1.2), xytext=(1.5, 1.2),
                arrowprops=dict(arrowstyle='->', color='#FB8C00', lw=1.5))
    
    # ========== Concatenation ==========
    concat_box = FancyBboxPatch((5.8, 1.0), 0.8, 5.5,
                                 boxstyle="round,pad=0.02",
                                 facecolor='#E1BEE7', edgecolor='#7B1FA2', linewidth=1)
    ax.add_patch(concat_box)
    ax.text(6.2, 3.75, 'Concat', ha='center', va='center', fontsize=7, fontweight='bold')
    ax.text(6.2, 3.4, '144-dim', ha='center', va='center', fontsize=7, color='#757575')
    
    # 箭头 Encoder → Concat
    ax.annotate('', xy=(5.9, 4.0), xytext=(5.6, 4.0),
                 arrowprops=dict(arrowstyle='->', color='#43A047', lw=1.5))
    
    # 箭头 Goal Embed → Concat
    ax.annotate('', xy=(5.9, 1.5), xytext=(5.5, 0.8),
                 arrowprops=dict(arrowstyle='->', color='#FB8C00', lw=1.5))
    
    # ========== GRU层 ==========
    gru_box = FancyBboxPatch((7.0, 1.5), 2.5, 5.0,
                              boxstyle="round,pad=0.05",
                              facecolor='#FFE0B2', edgecolor='#FB8C00', linewidth=2)
    ax.add_patch(gru_box)
    ax.text(8.25, 6.0, '2-Layer GRU', ha='center', va='center', fontsize=11, fontweight='bold')
    ax.text(8.25, 5.5, 'Hidden Size = 256', ha='center', va='center', fontsize=9)
    ax.text(8.25, 5.0, 'Dropout = 0.1', ha='center', va='center', fontsize=8, color='#757575')
    
    # GRU内部状态
    ax.text(8.25, 3.5, 'h_{t-1}', ha='center', va='center', fontsize=9, 
            color='#757575', fontfamily='monospace')
    ax.text(8.25, 2.8, 'GRU', ha='center', va='center', fontsize=10, fontweight='bold')
    ax.text(8.25, 2.3, 'h_t', ha='center', va='center', fontsize=9,
            color='#757575', fontfamily='monospace')
    ax.text(8.25, 1.8, 'T=8 steps', ha='center', va='center', fontsize=8, color='#757575')
    
    # 箭头 Concat → GRU
    ax.annotate('', xy=(7.1, 4.0), xytext=(6.6, 4.0),
                arrowprops=dict(arrowstyle='->', color='#7B1FA2', lw=1.5))
    
    # ========== 输出头 ==========
    # Action Head
    action_box = FancyBboxPatch((10.0, 3.5), 2.2, 2.2,
                                  boxstyle="round,pad=0.03",
                                  facecolor='#B3E5FC', edgecolor='#0288D1', linewidth=2)
    ax.add_patch(action_box)
    ax.text(11.1, 5.3, 'Action Head', ha='center', va='center', fontsize=10, fontweight='bold')
    ax.text(11.1, 4.8, 'MLP: 256→128→3', ha='center', va='center', fontsize=8)
    ax.text(11.1, 4.4, '+ Tanh', ha='center', va='center', fontsize=8, color='#757575')
    ax.text(11.1, 3.9, 'a_t ∈ [-1,1]³', ha='center', va='center', fontsize=9,
            fontfamily='monospace')
    
    # Action输出标签
    ax.text(12.4, 3.8, 'Δm₀,\nΔm₁,\nΔm₂', ha='left', va='center', fontsize=8, 
            fontweight='bold', color='#0288D1')
    ax.text(12.5, 3.3, '×[5°,3°,3°]/frame', ha='left', va='center', fontsize=7, 
            color='#757575')
    
    # Task Head
    task_box = FancyBboxPatch((10.0, 0.5), 2.2, 1.5,
                                boxstyle="round,pad=0.03",
                                facecolor='#FFCDD2', edgecolor='#D32F2F', linewidth=2)
    ax.add_patch(task_box)
    ax.text(11.1, 1.65, 'Task Head', ha='center', va='center', fontsize=10, fontweight='bold')
    ax.text(11.1, 1.2, 'MLP: 256→64→2', ha='center', va='center', fontsize=8)
    ax.text(11.1, 0.8, 'navigate / clear', ha='center', va='center', fontsize=8,
            fontstyle='italic')
    
    # 箭头 GRU → Heads
    ax.annotate('', xy=(10.1, 4.6), xytext=(9.5, 4.0),
                arrowprops=dict(arrowstyle='->', color='#0288D1', lw=1.5))
    ax.annotate('', xy=(10.1, 1.25), xytext=(9.5, 2.5),
                arrowprops=dict(arrowstyle='->', color='#D32F2F', lw=1.5))
    
    # ========== 标注 ==========
    ax.text(4.5, 0.1, '~0.4M parameters', ha='center', va='center', fontsize=9,
            fontweight='bold', color='#43A047')
    ax.text(8.5, 0.1, '<5ms inference on RTX 3090', ha='center', va='center', 
            fontsize=9, fontweight='bold', color='#FB8C00')
    
    # 标题
    ax.text(7, 7.5, 'BronchusPolicy Architecture', fontsize=14, fontweight='bold',
            ha='center', va='center')
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight',
                   facecolor='white', edgecolor='none')
        print(f"Fig 3 saved to: {save_path}")
    
    return ax


def generate_fig4_results(ax=None, save_path=None):
    """
    生成 Fig 4: 实验结果
    
    三个子图:
    (a) 导航成功率对比
    (b) 支气管仿体轨迹可视化
    (c) 电机角度时序图
    """
    if ax is None:
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        ax = axes.flatten()
    else:
        ax = ax.flatten() if hasattr(ax, 'flatten') else ax
    
    # 模拟实验数据
    np.random.seed(42)
    
    targets = ['LMB', 'RMB', 'LUB', 'RML']
    methods = ['Expert', 'Reactive', 'BC\n(no-goal)', 'BC+goal\n(ours)']
    
    # 成功率数据 (模拟)
    success_rates = {
        'Expert': [95, 93, 90, 88],
        'Reactive': [65, 60, 55, 58],
        'BC\n(no-goal)': [72, 68, 62, 65],
        'BC+goal\n(ours)': [85, 82, 78, 80]
    }
    
    colors = ['#757575', '#1E88E5', '#FB8C00', '#43A047']
    
    # Panel (a): 成功率对比
    ax[0].set_title('(a) Navigation Success Rate Comparison', fontsize=11, fontweight='bold')
    
    x = np.arange(len(targets))
    width = 0.2
    
    for i, (method, rates) in enumerate(success_rates.items()):
        offset = (i - 1.5) * width
        bars = ax[0].bar(x + offset, rates, width, label=method, color=colors[i],
                         edgecolor='black' if method == 'BC+goal\n(ours)' else 'none',
                         linewidth=2 if method == 'BC+goal\n(ours)' else 0.5)
        
        # 添加误差棒
        errors = [np.random.uniform(3, 8) for _ in range(4)]
        ax[0].errorbar(x + offset, rates, yerr=errors, fmt='none', 
                       color='black', capsize=3, capthick=1)
    
    ax[0].set_xlabel('Target Bronchus', fontsize=10)
    ax[0].set_ylabel('Success Rate (%)', fontsize=10)
    ax[0].set_xticks(x)
    ax[0].set_xticklabels(targets)
    ax[0].set_ylim(0, 105)
    ax[0].legend(loc='upper right', fontsize=8)
    ax[0].grid(axis='y', alpha=0.3)
    
    # 临床阈值线
    ax[0].axhline(y=80, color='#D50000', linestyle='--', linewidth=1, alpha=0.7)
    ax[0].text(3.5, 82, 'Clinical threshold (80%)', fontsize=7, color='#D50000')
    
    # Panel (b): 支气管树轨迹
    ax[1].set_title('(b) Phantom Trajectory Visualization', fontsize=11, fontweight='bold')
    
    # 绘制支气管树结构
    # 气管到主支气管
    trunk_x = [0, 0, -1, -2, -3]
    trunk_y = [5, 4, 3, 2, 1]
    ax[1].plot(trunk_x, trunk_y, 'k-', linewidth=3, solid_capstyle='round')
    
    # 左主支气管
    lmb_x = [-3, -4.5, -5.5, -6]
    lmb_y = [1, 0.5, 0, -0.5]
    ax[1].plot(lmb_x, lmb_y, 'k-', linewidth=2.5, solid_capstyle='round')
    
    # 右主支气管
    rmb_x = [-3, -1.5, -0.5, 0.5]
    rmb_y = [1, 0.5, 0, -0.5]
    ax[1].plot(rmb_x, rmb_y, 'k-', linewidth=2.5, solid_capstyle='round')
    
    # 上叶支气管
    lub_x = [-5.5, -6.5, -7]
    lub_y = [0, 0.5, 1]
    ax[1].plot(lub_x, lub_y, 'k-', linewidth=2)
    
    # 下叶支气管
    llb_x = [-6, -7, -7.5, -8]
    llb_y = [-0.5, -1, -1.5, -2]
    ax[1].plot(llb_x, llb_y, 'k-', linewidth=2)
    
    # 中间支气管
    rml_x = [0.5, 1.5, 2]
    rml_y = [-0.5, -0.5, -1]
    ax[1].plot(rml_x, rml_y, 'k-', linewidth=2)
    
    # 标注解剖结构
    ax[1].text(0, 5.3, 'Trachea', fontsize=9, ha='center', fontweight='bold')
    ax[1].text(-3.3, 1.3, 'Carina', fontsize=8, ha='right', color='#D50000')
    ax[1].text(-6.5, -0.3, 'LMB', fontsize=8, ha='center')
    ax[1].text(0.8, -0.3, 'RMB', fontsize=8, ha='left')
    ax[1].text(-7.5, 0.8, 'LUB', fontsize=7, ha='center')
    ax[1].text(-8, -1.8, 'LLB', fontsize=7, ha='center')
    ax[1].text(2.3, -0.8, 'RML', fontsize=7, ha='left')
    
    # 起始点
    start = Circle((0, 5.5), 0.15, facecolor='#1E88E5', edgecolor='black')
    ax[1].add_patch(start)
    ax[1].text(0.3, 5.7, 'Start', fontsize=8, color='#1E88E5', fontweight='bold')
    
    # Reactive轨迹（橙色虚线）
    reactive_path_x = [0, 0, -0.8, -2, -2.5, -4, -5, -4.5, -4.8]
    reactive_path_y = [5, 4.5, 3.5, 2.5, 1.5, 1, 0.5, 0, -0.3]
    ax[1].plot(reactive_path_x, reactive_path_y, '--', color='#FB8C00', 
               linewidth=1.5, alpha=0.8, label='Reactive')
    
    # 碰撞点标记
    collision = Circle((-2.5, 1.5), 0.12, facecolor='#D50000', edgecolor='black')
    ax[1].add_patch(collision)
    ax[1].text(-2.3, 1.7, 'X', fontsize=8, color='#D50000', fontweight='bold')
    
    # BC+goal轨迹（绿色实线）
    bc_path_x = [0, 0, -1, -2, -3, -4, -5, -5.5, -6, -6.5]
    bc_path_y = [5, 4.3, 3.2, 2.2, 1.3, 0.8, 0.3, 0, -0.2, 0.3]
    ax[1].plot(bc_path_x, bc_path_y, '-', color='#43A047', 
               linewidth=2, alpha=0.9, label='BC+goal')
    
    # 终点标记
    bc_end = Circle((-6.5, 0.3), 0.12, facecolor='#43A047', edgecolor='black')
    ax[1].add_patch(bc_end)
    ax[1].text(-6.7, 0.5, '✓', fontsize=10, color='#43A047', fontweight='bold')
    
    ax[1].set_xlim(-9, 3)
    ax[1].set_ylim(-2.5, 6)
    ax[1].set_aspect('equal')
    ax[1].legend(loc='upper right', fontsize=8)
    ax[1].axis('off')
    
    # Panel (c): 电机角度时序图
    ax[2].set_title('(c) Motor Angle Time Series', fontsize=11, fontweight='bold')
    
    time = np.arange(0, 200)
    
    # 生成模拟数据
    # Reactive数据（波动较大）
    reactive_m0 = -50 + 10 * np.sin(time * 0.1) + 5 * np.random.randn(len(time))
    reactive_m1 = 30 * np.sin(time * 0.15) + 15 * np.cos(time * 0.08)
    reactive_m2 = 15 * np.sin(time * 0.12 + 1) + 8 * np.random.randn(len(time))
    
    # BC+goal数据（更平滑）
    bc_m0 = -50 + 8 * np.sin(time * 0.1 + 0.3) + 2 * np.random.randn(len(time))
    bc_m1 = 28 * np.sin(time * 0.15 + 0.5) + 8 * np.cos(time * 0.08)
    bc_m2 = 12 * np.sin(time * 0.12 + 1.5) + 3 * np.random.randn(len(time))
    
    # M0: 推进/回撤
    ax[2].plot(time, reactive_m0, '--', color='#FB8C00', linewidth=1, alpha=0.7, label='Reactive')
    ax[2].plot(time, bc_m0, '-', color='#43A047', linewidth=1.5, label='BC+goal')
    ax[2].set_ylabel('M₀ (advancement) [°]', fontsize=9)
    ax[2].set_ylim(-80, 20)
    ax[2].grid(True, alpha=0.3)
    ax[2].legend(loc='upper right', fontsize=7)
    
    # 添加分岔事件标记
    for t in [50, 100, 150]:
        ax[2].axvline(x=t, color='#BDBDBD', linestyle=':', alpha=0.7)
    
    # Panel (d): M1和M2
    ax[3].set_title('Motor Deflection Angles', fontsize=11, fontweight='bold')
    
    ax[3].plot(time, reactive_m1, '--', color='#FB8C00', linewidth=1, alpha=0.7, label='Reactive M₁')
    ax[3].plot(time, bc_m1, '-', color='#43A047', linewidth=1.5, label='BC+goal M₁')
    ax[3].plot(time, reactive_m2, '--', color='#1E88E5', linewidth=1, alpha=0.7, label='Reactive M₂')
    ax[3].plot(time, bc_m2, '-', color='#7E57C2', linewidth=1.5, label='BC+goal M₂')
    
    ax[3].set_ylabel('Angle [°]', fontsize=9)
    ax[3].set_xlabel('Time (frames @ 20 Hz)', fontsize=9)
    ax[3].set_ylim(-60, 60)
    ax[3].grid(True, alpha=0.3)
    ax[3].legend(loc='upper right', fontsize=7, ncol=2)
    
    # 添加分岔事件
    for t in [50, 100, 150]:
        ax[3].axvline(x=t, color='#BDBDBD', linestyle=':', alpha=0.7)
        ax[3].text(t, 55, '|', fontsize=10, ha='center', color='#757575')
    
    # 标注BC+goal更平滑
    ax[3].annotate('BC+goal:\nSmoother\ntrajectories', 
                   xy=(120, 0), xytext=(140, 30),
                   fontsize=8, ha='center',
                   arrowprops=dict(arrowstyle='->', color='#43A047', lw=1),
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='#E8F5E9', 
                            edgecolor='#43A047', alpha=0.8))
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight',
                   facecolor='white', edgecolor='none')
        print(f"Fig 4 saved to: {save_path}")
    
    return ax


def generate_all_figures(output_dir='./output'):
    """生成所有图片"""
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    print("=" * 60)
    print("Generating Scientific Figures for IEEE RA-L Paper")
    print("=" * 60)
    
    # Fig 1: System Overview
    print("\n[1/4] Generating Fig 1: System Overview...")
    fig1, ax1 = plt.subplots(figsize=(12, 8))
    generate_fig1_system_overview(ax1, output_path / 'fig1_system_overview.png')
    plt.close(fig1)
    
    # Fig 2: Perception Pipeline
    print("[2/4] Generating Fig 2: Perception Pipeline...")
    fig2, ax2 = plt.subplots(2, 2, figsize=(12, 10))
    generate_fig2_perception(ax2, output_path / 'fig2_perception.png')
    plt.close(fig2)
    
    # Fig 3: Network Architecture
    print("[3/4] Generating Fig 3: Network Architecture...")
    fig3, ax3 = plt.subplots(figsize=(14, 8))
    generate_fig3_network(ax3, output_path / 'fig3_network.png')
    plt.close(fig3)
    
    # Fig 4: Results
    print("[4/4] Generating Fig 4: Experimental Results...")
    fig4, axes = plt.subplots(2, 2, figsize=(14, 10))
    generate_fig4_results(axes, output_path / 'fig4_results.png')
    plt.close(fig4)
    
    print("\n" + "=" * 60)
    print("All figures generated successfully!")
    print(f"Output directory: {output_path.absolute()}")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='Generate scientific figures for IEEE RA-L paper'
    )
    parser.add_argument(
        '--figure', '-f',
        type=str,
        default='all',
        choices=['1', '2', '3', '4', 'all'],
        help='Figure number to generate (default: all)'
    )
    parser.add_argument(
        '--output_dir', '-o',
        type=str,
        default='./output',
        help='Output directory for generated figures'
    )
    
    args = parser.parse_args()
    
    output_path = Path(args.output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    figure_map = {
        '1': ('Fig 1: System Overview', generate_fig1_system_overview, 'fig1_system_overview.png'),
        '2': ('Fig 2: Perception Pipeline', generate_fig2_perception, 'fig2_perception.png'),
        '3': ('Fig 3: Network Architecture', generate_fig3_network, 'fig3_network.png'),
        '4': ('Fig 4: Experimental Results', generate_fig4_results, 'fig4_results.png'),
    }
    
    if args.figure == 'all':
        generate_all_figures(args.output_dir)
    else:
        name, func, filename = figure_map[args.figure]
        print(f"Generating {name}...")
        
        if args.figure == '1':
            fig, ax = plt.subplots(figsize=(12, 8))
        elif args.figure == '2':
            fig, ax = plt.subplots(2, 2, figsize=(12, 10))
        elif args.figure == '3':
            fig, ax = plt.subplots(figsize=(14, 8))
        else:
            fig, ax = plt.subplots(2, 2, figsize=(14, 10))
        
        func(ax, output_path / filename)
        plt.close(fig)
        print(f"Saved to: {output_path / filename}")


if __name__ == '__main__':
    main()
