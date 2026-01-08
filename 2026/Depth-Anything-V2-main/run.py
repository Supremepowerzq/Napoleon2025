import argparse
import cv2
import glob
import matplotlib
import numpy as np
import os
import torch

from depth_anything_v2.dpt import DepthAnythingV2

# ---------------- 提取纯圆形有效像素（适配图片+自动计算圆心/半径） ----------------
def extract_circular_roi(image):
    """
    对图片自动提取纯圆形有效像素（圆心=图像中心，半径=最小边的一半）
    :param image: 输入原始图片
    :return: 
        - roi_image: 纯圆形有效像素组成的图像（无黑边，正方形，用于推理）
        - crop_mask: 圆形掩码（用于过滤深度图）
        - crop_size: 裁剪后的正方形尺寸
        - original_roi: 原始圆形区域画面（用于显示）
    """
    # 1. 自动计算圆心和半径（图像中心 + 最小边的一半）
    height, width = image.shape[:2]
    center_x, center_y = width // 2, height // 2
    radius = min(width, height) // 2  # 半径=最小边的一半
    
    # 2. 创建圆形掩码（软掩码，边缘渐变减少干扰）
    mask = np.zeros((height, width), dtype=np.float32)
    cv2.circle(mask, (center_x, center_y), radius, 1.0, -1)
    mask = cv2.GaussianBlur(mask, (21, 21), 0)  # 边缘渐变，避免特征突变
    
    # 3. 保留全局弱上下文（用于推理，兼顾精度+无干扰）
    blurred_image = cv2.GaussianBlur(image, (51, 51), 0)
    roi_image = image * mask[..., np.newaxis] + blurred_image * (1 - mask[..., np.newaxis]) * 0.1
    roi_image = roi_image.astype(np.uint8)
    
    # 4. 提取原始圆形区域画面（仅用于显示，无中性灰）
    original_mask = np.zeros_like(image)
    cv2.circle(original_mask, (center_x, center_y), radius, (255, 255, 255), -1)
    original_roi = np.where(original_mask == 255, image, 0)  # 仅保留圆形，其余黑
    
    # 5. 裁剪出圆形所在的正方形区域
    x1 = max(0, center_x - radius)
    y1 = max(0, center_y - radius)
    x2 = min(width, center_x + radius)
    y2 = min(height, center_y + radius)
    roi_image = roi_image[y1:y2, x1:x2]
    original_roi = original_roi[y1:y2, x1:x2]
    crop_mask = (mask[y1:y2, x1:x2] > 0.5).astype(np.uint8) * 255
    crop_size = (roi_image.shape[1], roi_image.shape[0])  # (width, height)
    
    return roi_image, crop_mask, crop_size, original_roi

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Depth Anything V2 - 仅圆形区域图片深度推理')
    
    parser.add_argument('--img-path', type=str, required=True, help='图片文件/文件夹路径，或txt文件（含图片列表）')
    parser.add_argument('--input-size', type=int, default=518, help='模型输入尺寸')
    parser.add_argument('--outdir', type=str, default='./vis_depth_circular', help='输出目录')
    
    parser.add_argument('--encoder', type=str, default='vitl', choices=['vits', 'vitb', 'vitl', 'vitg'])
    
    parser.add_argument('--pred-only', dest='pred_only', action='store_true', help='仅保存深度图（不拼接原图）')
    parser.add_argument('--grayscale', dest='grayscale', action='store_true', help='灰度深度图（默认彩色）')
    
    args = parser.parse_args()
    
    # 设备初始化
    DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
    print(f"使用设备：{DEVICE}")
    
    # 模型加载
    model_configs = {
        'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
        'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
        'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
        'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
    }
    
    depth_anything = DepthAnythingV2(**model_configs[args.encoder])
    weight_path = f'checkpoints/depth_anything_v2_{args.encoder}.pth'
    if not os.path.exists(weight_path):
        print(f"错误：权重文件不存在 → {weight_path}")
        exit(1)
    depth_anything.load_state_dict(torch.load(weight_path, map_location='cpu'))
    depth_anything = depth_anything.to(DEVICE).eval()
    
    # 解析图片列表
    if os.path.isfile(args.img_path):
        if args.img_path.endswith('txt'):
            with open(args.img_path, 'r') as f:
                filenames = f.read().splitlines()
        else:
            filenames = [args.img_path]
    else:
        filenames = glob.glob(os.path.join(args.img_path, '**/*'), recursive=True)
    # 过滤非图片文件
    img_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.tiff']
    filenames = [f for f in filenames if os.path.splitext(f)[1].lower() in img_extensions]
    
    if not filenames:
        print("错误：未找到有效图片文件！")
        exit(1)
    
    os.makedirs(args.outdir, exist_ok=True)
    cmap = matplotlib.colormaps.get_cmap('Spectral_r')
    
    # 逐张图片处理
    for k, filename in enumerate(filenames):
        print(f'Progress {k+1}/{len(filenames)}: {filename}')
        
        # 读取原始图片
        raw_image = cv2.imread(filename)
        if raw_image is None:
            print(f"警告：无法读取图片 {filename}，跳过")
            continue
        
        # ========== 核心：提取圆形区域（仅对该区域推理） ==========
        roi_image, crop_mask, crop_size, original_roi = extract_circular_roi(raw_image)
        
        # ========== 仅对圆形区域做深度推理 ==========
        depth = depth_anything.infer_image(roi_image, args.input_size)
        
        # 归一化+掩码过滤（仅保留圆形区域深度值）
        depth = (depth - depth.min()) / (depth.max() - depth.min()) * 255.0
        depth = depth.astype(np.uint8)
        # 缩放回圆形尺寸（匹配crop_size）
        depth = cv2.resize(depth, crop_size)
        # 仅保留圆形掩码内的深度值，其余设为0（黑色）
        depth_masked = np.zeros_like(depth)
        depth_masked[crop_mask == 255] = depth[crop_mask == 255]
        
        # 深度图可视化（灰度/彩色）
        if args.grayscale:
            depth_vis = np.repeat(depth_masked[..., np.newaxis], 3, axis=-1)
        else:
            depth_color = (cmap(depth)[:, :, :3] * 255)[:, :, ::-1].astype(np.uint8)
            depth_vis = np.zeros_like(depth_color)
            depth_vis[crop_mask == 255] = depth_color[crop_mask == 255]
        
        # 保存结果
        img_basename = os.path.splitext(os.path.basename(filename))[0]
        if args.pred_only:
            # 仅保存圆形深度图
            cv2.imwrite(os.path.join(args.outdir, f"{img_basename}_depth.png"), depth_vis)
        else:
            # 拼接：左=原始圆形区域，右=圆形深度图（和视频/实时版本一致）
            split_region = np.ones((crop_size[1], 50, 3), dtype=np.uint8) * 255  # 白色分隔条
            combined_result = cv2.hconcat([original_roi, split_region, depth_vis])
            cv2.imwrite(os.path.join(args.outdir, f"{img_basename}_combined.png"), combined_result)
    
    print(f"\n处理完成！结果已保存到：{args.outdir}")