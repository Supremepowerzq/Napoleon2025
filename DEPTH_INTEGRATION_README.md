# Depth-Anything-V2 深度推理集成说明

## 概述

已成功将 `Depth-Anything-V2` 的深度图功能集成到 `predict_2026.py` 中，保留了原有的所有功能。

## 新增功能

### 深度推理窗口
- 新增 `Depth` 窗口显示实时深度图
- 使用彩色可视化（可通过参数调整为灰度）
- 与原有 `Original` 和 `video` 窗口并存

### 关键特性
- **输入源**: 使用 `predict_2026.py` 中 'Original' 窗口的 circular 图像帧作为深度推理输入
- **无额外裁剪**: 按照用户要求，不再进行额外的圆形裁剪处理
- **保留原有功能**: UNet目标检测、运动控制等功能完全不变

## 使用方法

### 启用深度推理
在 `predict_2026.py` 主程序中，`UnetPackage` 实例化时设置：
```python
unet_package = UnetPackage(
    mode='video',
    video_path=0,
    video_fps=30,
    # 深度推理相关参数
    enable_depth=True,  # 启用深度推理功能
    depth_encoder='vits',  # 模型类型: vits/vitb/vitl/vitg
    depth_input_size=518,  # 模型输入尺寸
    depth_grayscale=False,  # 是否使用灰度深度图
)
```

### 深度推理参数说明
- `enable_depth`: 是否启用深度推理（需要Depth-Anything-V2可用）
- `depth_encoder`: 深度模型编码器类型
  - 'vits': 最轻量级，速度最快
  - 'vitb': 平衡性能
  - 'vitl': 高精度
  - 'vitg': 最高精度，速度最慢
- `depth_input_size`: 模型输入分辨率（建议518）
- `depth_grayscale`: True=灰度深度图，False=彩色深度图

## 显示窗口

运行程序后将显示三个窗口：
1. **Original**: 圆形裁剪后的原始图像
2. **video**: UNet检测结果和控制界面
3. **Depth**: 实时深度图可视化（新增）

## 系统要求

### 依赖包
- PyTorch
- OpenCV
- NumPy
- Matplotlib
- Depth-Anything-V2 (已集成在项目中)

### 硬件要求
- CUDA兼容GPU（推荐）或CPU
- 至少4GB RAM
- 深度模型权重文件位于 `2026/Depth-Anything-V2-main/checkpoints/`

## 故障排除

### 深度推理不可用
如果看到提示"深度推理功能将被禁用"，可能原因：
1. Depth-Anything-V2模块未正确安装
2. 缺少必要的依赖包
3. 模型权重文件不存在

### 性能问题
- 使用 'vits' 模型获得最佳性能
- 如遇内存不足，可降低 `depth_input_size`
- CPU模式下性能会显著下降

## 技术实现

### 集成方式
- 在 `UnetPackage` 类中添加深度推理相关方法
- 使用 'Original' 窗口的 circular 图像帧直接作为深度推理输入
- 深度推理结果在独立线程中处理，不影响原有视频处理流程

### 代码结构
```python
class UnetPackage:
    def _init_depth_model(self):  # 深度模型初始化
    def infer_depth(self, image):  # 深度推理方法
    def extract_circular_roi(self, ...):  # 圆形区域提取
    def resize_with_padding(self, ...):  # 宽高比调整
```

## 注意事项

1. **功能独立性**: 深度推理功能可独立启用/禁用，不影响原有功能
2. **资源占用**: 深度推理会增加GPU/CPU使用率和内存占用
3. **实时性**: 深度推理可能略微降低整体帧率，取决于硬件性能
4. **兼容性**: 保留了所有原有参数和配置选项的兼容性