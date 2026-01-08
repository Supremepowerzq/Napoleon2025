import cv2
import torch
import numpy as np
import argparse
import matplotlib
from depth_anything_v2.dpt import DepthAnythingV2
import os

# ---------------- 视频帧预处理函数（核心修复：匹配摄像头画面特性） ----------------
def preprocess_video_frame(frame, target_resolution=(1280, 720)):
    """
    预处理视频帧：统一分辨率+去压缩失真+降噪，模拟实时摄像头的纯净输入
    :param frame: 视频原始帧
    :param target_resolution: 目标分辨率（和实时摄像头一致，默认1280×720）
    :return: 预处理后的纯净帧
    """
    # 1. 统一缩放到目标分辨率（消除分辨率不一致问题）
    frame = cv2.resize(frame, target_resolution)
    # 2. 去压缩失真（高斯模糊+锐化，恢复高频细节）
    frame = cv2.GaussianBlur(frame, (3, 3), 0)
    frame = cv2.addWeighted(frame, 1.5, np.zeros_like(frame), 0, 0)  # 锐化增强细节
    # 3. 降噪（消除视频压缩产生的噪点）
    frame = cv2.fastNlMeansDenoisingColored(frame, None, 10, 10, 7, 21)
    return frame

# ---------------- 提取纯圆形有效像素（适配视频+实时参数对齐） ----------------
def extract_circular_roi(image):
    """
    对视频帧自动提取纯圆形有效像素（圆心=帧中心，半径=最小边的一半）
    :param image: 预处理后的视频单帧图像
    :return: 
        - roi_image: 纯圆形有效像素组成的图像（无黑边，正方形）
        - crop_mask: 圆形掩码（用于后续过滤深度图）
        - crop_size: 裁剪后的正方形尺寸（用于还原）
        - original_roi: 原始圆形区域画面（无中性灰填充，用于显示）
    """
    # 1. 自动计算圆心和半径（视频帧中心 + 最小边的一半）
    height, width = image.shape[:2]
    center_x, center_y = width // 2, height // 2
    radius = min(width, height) // 2  # 半径=最小边的一半
    
    # 2. 创建圆形掩码（软掩码，边缘渐变减少突变干扰）
    mask = np.zeros((height, width), dtype=np.float32)
    cv2.circle(mask, (center_x, center_y), radius, 1.0, -1)
    mask = cv2.GaussianBlur(mask, (21, 21), 0)  # 边缘渐变，避免特征突变
    
    # 3. 保留全局弱上下文（用于推理，兼顾精度+无干扰）
    blurred_image = cv2.GaussianBlur(image, (51, 51), 0)  # 全局模糊
    roi_image = image * mask[..., np.newaxis] + blurred_image * (1 - mask[..., np.newaxis]) * 0.1
    roi_image = roi_image.astype(np.uint8)
    
    # 4. 提取原始圆形区域画面（无中性灰，仅用于显示）
    original_mask = np.zeros_like(image)
    cv2.circle(original_mask, (center_x, center_y), radius, (255, 255, 255), -1)
    original_roi = np.where(original_mask == 255, image, 0)  # 仅保留圆形区域，其余黑
    
    # 5. 裁剪出圆形所在的正方形区域
    x1 = max(0, center_x - radius)
    y1 = max(0, center_y - radius)
    x2 = min(width, center_x + radius)
    y2 = min(height, center_y + radius)
    roi_image = roi_image[y1:y2, x1:x2]
    original_roi = original_roi[y1:y2, x1:x2]  # 裁剪原始圆形画面
    crop_mask = (mask[y1:y2, x1:x2] > 0.5).astype(np.uint8) * 255  # 硬掩码用于后处理
    crop_size = (roi_image.shape[1], roi_image.shape[0])  # (width, height)
    
    return roi_image, crop_mask, crop_size, original_roi

# ---------------- 视频深度推理主函数 ----------------
def main(args):
    # 1. 设备初始化
    DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
    print(f"========== 初始化信息 ==========")
    print(f"使用设备：{DEVICE}")
    print(f"目标分辨率：{args.target_resolution}")
    print(f"平滑系数：{args.smooth_alpha}")
    print(f"模型输入尺寸：{args.input_size}")

    # 2. 模型配置与加载
    model_configs = {
        'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
        'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
        'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
        'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
    }

    depth_anything = DepthAnythingV2(**model_configs[args.encoder])
    weight_path = f'checkpoints/depth_anything_v2_{args.encoder}.pth'
    if not os.path.exists(weight_path):
        print(f"\n错误：权重文件不存在 → {weight_path}")
        print(f"请确保 checkpoints 文件夹下有 {os.path.basename(weight_path)}")
        exit(1)
    
    depth_anything.load_state_dict(torch.load(weight_path, map_location=DEVICE))
    depth_anything = depth_anything.to(DEVICE).eval()
    print("模型加载成功！")

    # 3. 视频读取配置
    cap = cv2.VideoCapture(args.video_path)
    if not cap.isOpened():
        print(f"\n错误：无法打开视频文件 → {args.video_path}")
        exit(1)
    
    # 获取视频基本信息
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    original_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    original_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"\n========== 视频信息 ==========")
    print(f"原始分辨率：{original_width}×{original_height}")
    print(f"帧率：{fps} | 总帧数：{total_frames}")

    # 4. 输出视频配置（左右拼接：原始圆形 + 深度图）
    os.makedirs(args.outdir, exist_ok=True)
    # 先读取第一帧，计算拼接后的输出尺寸
    ret, first_frame = cap.read()
    if not ret:
        print("错误：无法读取视频第一帧！")
        exit(1)
    first_frame = preprocess_video_frame(first_frame, args.target_resolution)
    _, _, crop_size, original_roi = extract_circular_roi(first_frame)
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # 重置视频读取指针到第一帧
    
    # 拼接后的尺寸：宽度=2*crop_size[0]，高度=crop_size[1]（左右各一个圆形）
    output_size = (crop_size[0] * 2, crop_size[1])
    # 设置输出视频编码器（MP4格式）
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    output_path = os.path.join(args.outdir, 'circular_depth_video.mp4')
    out = cv2.VideoWriter(
        output_path,
        fourcc,
        fps,
        output_size,  # 拼接后的尺寸
        isColor=True  # 始终彩色（左侧原始图+右侧深度图）
    )
    print(f"\n========== 输出信息 ==========")
    print(f"输出视频路径：{output_path}")
    print(f"输出分辨率：{output_size[0]}×{output_size[1]}（左：原始圆形 | 右：深度图）")
    print(f"开始处理视频...")

    # 5. 颜色映射（用于彩色深度图）
    cmap = matplotlib.colormaps.get_cmap('Spectral_r')

    # 6. 时序平滑参数初始化
    prev_depth = None
    smooth_alpha = args.smooth_alpha

    # 7. 逐帧处理（纯圆形区域推理+修复优化+左右拼接）
    processed_frames = 0
    with torch.no_grad():
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            # ========== 步骤1：视频帧预处理（核心修复） ==========
            frame = preprocess_video_frame(frame, args.target_resolution)

            # ========== 步骤2：提取纯圆形有效像素 ==========
            roi_image, crop_mask, crop_size, original_roi = extract_circular_roi(frame)

            # ========== 步骤3：仅对圆形区域做深度推理（尺寸对齐） ==========
            # 先缩放圆形区域到模型最优输入尺寸，提升精度
            roi_image_resized = cv2.resize(roi_image, (args.input_size, args.input_size))
            depth = depth_anything.infer_image(roi_image_resized, args.input_size)
            # 再缩放回原圆形尺寸
            depth = cv2.resize(depth, crop_size)

            # ========== 步骤4：深度图后处理（仅保留圆形区域） ==========
            # 归一化到0-255
            depth = (depth - depth.min()) / (depth.max() - depth.min()) * 255.0
            depth = depth.astype(np.uint8)
            
            # 仅保留圆形掩码内的深度值，其余设为0（黑色）
            depth_masked = np.zeros_like(depth)
            depth_masked[crop_mask == 255] = depth[crop_mask == 255]

            # ========== 步骤5：时序平滑（解决帧间跳变） ==========
            if prev_depth is not None:
                depth_masked = (smooth_alpha * depth_masked + (1 - smooth_alpha) * prev_depth).astype(np.uint8)
            prev_depth = depth_masked.copy()

            # ========== 步骤6：深度图可视化（彩色/灰度） ==========
            if args.grayscale:
                depth_vis = np.repeat(depth_masked[..., np.newaxis], 3, axis=-1)
            else:
                # 先生成彩色图，再过滤出圆形区域
                depth_color = (cmap(depth)[:, :, :3] * 255)[:, :, ::-1].astype(np.uint8)
                depth_vis = np.zeros_like(depth_color)
                depth_vis[crop_mask == 255] = depth_color[crop_mask == 255]

            # ========== 步骤7：左右拼接（原始圆形 + 深度图） ==========
            # 确保原始roi和深度图尺寸一致
            original_roi_resized = cv2.resize(original_roi, crop_size)
            depth_vis_resized = cv2.resize(depth_vis, crop_size)
            # 拼接：左=原始圆形，右=深度图
            combined_frame = cv2.hconcat([original_roi_resized, depth_vis_resized])

            # ========== 步骤8：写入输出视频 ==========
            out.write(combined_frame)

            # 进度提示
            processed_frames += 1
            if processed_frames % 10 == 0:
                progress = (processed_frames / total_frames) * 100
                print(f"已处理：{processed_frames}/{total_frames} 帧 | 进度：{progress:.1f}%", end='\r')

    # 8. 释放资源
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    print(f"\n\n========== 处理完成 ==========")
    print(f"输出视频已保存到：{output_path}")
    print(f"总处理帧数：{processed_frames}")
    print(f"输出格式：左侧=原始圆形画面 | 右侧=圆形深度图")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Depth-Anything-V2 视频纯圆形区域深度推理（修复版+左右拼接显示）')
    # 核心参数
    parser.add_argument('--video-path', type=str, required=True, help='输入视频文件路径（如.mp4/.avi）')
    parser.add_argument('--outdir', type=str, default='./video_output', help='输出视频保存目录')
    parser.add_argument('--encoder', type=str, default='vits', choices=['vits', 'vitb', 'vitl', 'vitg'],
                        help='模型编码器版本（vits最快，vitg精度最高）')
    parser.add_argument('--input-size', type=int, default=384, help='模型输入尺寸（和实时版本对齐，推荐384）')
    parser.add_argument('--grayscale', dest='grayscale', action='store_true', help='深度图使用灰度显示（默认彩色）')
    
    # 修复相关参数
    parser.add_argument('--target-resolution', type=tuple, default=(1280, 720), help='预处理目标分辨率（和实时摄像头一致）')
    parser.add_argument('--smooth-alpha', type=float, default=0.7, help='时序平滑系数（0-1，越大越稳定）')
    
    args = parser.parse_args()
    main(args)