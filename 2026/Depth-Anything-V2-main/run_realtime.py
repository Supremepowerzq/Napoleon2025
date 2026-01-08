import cv2
import torch
import numpy as np
import argparse
import matplotlib
from depth_anything_v2.dpt import DepthAnythingV2
import os

# ---------------- 提取纯圆形有效像素（核心修改） ----------------
def extract_circular_roi(image, center_x, center_y, radius):
    """
    仅提取圆形内的有效像素（彻底去掉黑边），返回纯圆形区域的图像
    :param image: 已拉伸的输入图像
    :param center_x/y: 圆心坐标（拉伸后画面）
    :param radius: 圆半径
    :return: 
        - roi_image: 纯圆形有效像素组成的图像（无黑边，尺寸=2*radius×2*radius）
        - mask: 圆形掩码（用于后续还原）
    """
    # 1. 创建圆形掩码
    mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
    cv2.circle(mask, (center_x, center_y), radius, 255, -1)
    
    # 2. 裁剪出圆形所在的正方形区域（仅包含圆形的最小正方形）
    x1 = max(0, center_x - radius)
    y1 = max(0, center_y - radius)
    x2 = min(image.shape[1], center_x + radius)
    y2 = min(image.shape[0], center_y + radius)
    crop_image = image[y1:y2, x1:x2]
    crop_mask = mask[y1:y2, x1:x2]  # 裁剪后的掩码
    
    # 3. 仅保留掩码内的有效像素（黑边设为中性灰，避免干扰）
    roi_image = np.copy(crop_image)
    roi_image[crop_mask == 0] = 128  # 中性灰（模型对中性灰无响应）
    
    return roi_image, crop_mask

# ---------------- 强制宽高比处理函数（保留） ----------------
def resize_with_padding(image, target_ratio=16/9):
    orig_height, orig_width = image.shape[:2]
    orig_ratio = orig_width / orig_height
    
    if orig_ratio > target_ratio:
        new_height = orig_height
        new_width = int(new_height * target_ratio)
        resized = cv2.resize(image, (new_width, new_height))
        pad_width = (orig_width - new_width) // 2
        padded = cv2.copyMakeBorder(resized, 0, 0, pad_width, pad_width, cv2.BORDER_CONSTANT, value=[0, 0, 0])
    else:
        new_width = orig_width
        new_height = int(new_width / target_ratio)
        resized = cv2.resize(image, (new_width, new_height))
        pad_height = (orig_height - new_height) // 2
        padded = cv2.copyMakeBorder(resized, pad_height, pad_height, 0, 0, cv2.BORDER_CONSTANT, value=[0, 0, 0])
    
    return padded

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Depth-Anything-V2 纯圆形区域深度推理')
    parser.add_argument('--encoder', type=str, default='vits', choices=['vits', 'vitb', 'vitl', 'vitg'])
    parser.add_argument('--input-size', type=int, default=518, help='模型输入尺寸')
    parser.add_argument('--grayscale', dest='grayscale', action='store_true', help='灰度深度图')
    
    # 你的原始圆形参数（完全保留）
    parser.add_argument('--circle-center-x', type=int, default=400, help='圆心X（拉伸后画面）')
    parser.add_argument('--circle-center-y', type=int, default=230, help='圆心Y（拉伸后画面）')
    parser.add_argument('--circle-radius', type=int, default=115, help='圆半径')
    
    parser.add_argument('--target-ratio', type=float, default=16/9, help='目标宽高比')
    
    args = parser.parse_args()

    # 设备/模型初始化（保留）
    DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
    model_configs = {
        'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
        'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
        'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
        'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
    }

    print(f"使用设备：{DEVICE} | 圆形参数：({args.circle_center_x},{args.circle_center_y}) 半径{args.circle_radius}")

    # 加载模型（保留）
    depth_anything = DepthAnythingV2(**model_configs[args.encoder])
    weight_path = f'checkpoints/depth_anything_v2_{args.encoder}.pth'
    if not os.path.exists(weight_path):
        print(f"错误：权重文件不存在 → {weight_path}")
        exit(1)
    depth_anything.load_state_dict(torch.load(weight_path, map_location=DEVICE))
    depth_anything = depth_anything.to(DEVICE).eval()

    # 打开摄像头（保留）
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("错误：无法打开摄像头！")
        exit(1)

    cmap = matplotlib.colormaps.get_cmap('Spectral_r')
    margin_width = 0

    with torch.no_grad():
        while True:
            ret, raw_frame = cap.read()
            if not ret:
                break

            # ========== 步骤1：拉伸画面（和你原代码一致） ==========
            stretched_frame = resize_with_padding(raw_frame, target_ratio=args.target_ratio)

            # ========== 步骤2：提取纯圆形有效像素（核心！） ==========
            # 只保留圆形内的画面，黑边替换为中性灰（模型不响应）
            roi_image, crop_mask = extract_circular_roi(
                stretched_frame,
                center_x=args.circle_center_x,
                center_y=args.circle_center_y,
                radius=args.circle_radius
            )

            # ========== 步骤3：仅对纯圆形区域推理（无黑边干扰） ==========
            depth = depth_anything.infer_image(roi_image, args.input_size)

            # ========== 步骤4：深度图后处理（仅保留圆形区域的深度） ==========
            # 归一化
            depth = (depth - depth.min()) / (depth.max() - depth.min()) * 255.0
            depth = depth.astype(np.uint8)
            
            # 仅保留圆形掩码内的深度值，其余设为0（黑色）
            depth_masked = np.zeros_like(depth)
            depth_masked[crop_mask == 255] = depth[crop_mask == 255]  # 只保留圆形深度
            
            # 可视化
            if args.grayscale:
                depth_vis = np.repeat(depth_masked[..., np.newaxis], 3, axis=-1)
            else:
                # 先生成彩色图，再掩码过滤
                depth_color = (cmap(depth)[:, :, :3] * 255)[:, :, ::-1].astype(np.uint8)
                depth_vis = np.zeros_like(depth_color)
                depth_vis[crop_mask == 255] = depth_color[crop_mask == 255]

            # ========== 步骤5：显示（纯圆形，无正方形黑边） ==========
            # 统一缩放（保持尺寸一致）
            crop_size = max(roi_image.shape[0], roi_image.shape[1])
            roi_image_resized = cv2.resize(roi_image, (crop_size, crop_size))
            depth_vis_resized = cv2.resize(depth_vis, (crop_size, crop_size))

            # 拼接显示（纯圆形原始帧 + 纯圆形深度帧）
            split_region = np.ones((crop_size, margin_width, 3), dtype=np.uint8) * 255
            combined_frame = cv2.hconcat([roi_image_resized, split_region, depth_vis_resized])

            cv2.imshow('纯圆形区域深度推理（无黑边干扰）', combined_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()
    print("程序退出成功！")