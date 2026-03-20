import argparse
import os
import cv2
import numpy as np
import torch
import matplotlib

from depth_anything_v2.dpt import DepthAnythingV2


def make_circle_mask(side: int) -> np.ndarray:
    """side x side 的圆形mask，圆内=1，圆外=0（float32）"""
    cx = side // 2
    cy = side // 2
    r = side // 2
    Y, X = np.ogrid[:side, :side]
    mask = ((X - cx) ** 2 + (Y - cy) ** 2) <= (r ** 2)
    return mask.astype(np.float32)


def apply_roi_circle_preprocess(roi_bgr: np.ndarray, mask: np.ndarray, blur_ksize: int = 51,
                                outside_strength: float = 0.15) -> np.ndarray:
    """
    圆外不参与推理：把圆外区域变成“灰度+弱化的背景”，减少突变边缘干扰。
    outside_strength 越小，圆外越暗/越不显著。
    """
    roi = roi_bgr.copy()

    # 灰度 -> 3通道
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray3 = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    # 轻微模糊的灰度背景，避免噪声影响
    if blur_ksize is not None and blur_ksize >= 3:
        if blur_ksize % 2 == 0:
            blur_ksize += 1
        gray3 = cv2.GaussianBlur(gray3, (blur_ksize, blur_ksize), 0)

    m = mask[..., None]  # (H,W,1)

    # 圆内保留原彩色；圆外使用灰度背景并弱化
    roi_proc = roi * m + gray3 * (1.0 - m) * outside_strength
    roi_proc = np.clip(roi_proc, 0, 255).astype(np.uint8)
    return roi_proc


def normalize_depth_to_uint8(depth: np.ndarray) -> np.ndarray:
    """把模型输出深度归一化到 0~255"""
    dmin = float(depth.min())
    dmax = float(depth.max())
    if dmax - dmin < 1e-6:
        return np.zeros_like(depth, dtype=np.uint8)
    depth_n = (depth - dmin) / (dmax - dmin) * 255.0
    return depth_n.astype(np.uint8)


def main():
    parser = argparse.ArgumentParser(description="Depth Anything V2 Metric Depth - Realtime ROI(788) + Circle Mask")
    parser.add_argument("--camera", type=int, default=0, help="camera index")
    parser.add_argument("--input-size", type=int, default=518, help="model input size")
    parser.add_argument("--outdir", type=str, default="./vis_depth_realtime", help="save directory")
    parser.add_argument("--encoder", type=str, default="vitl", choices=["vits", "vitb", "vitl", "vitg"])
    parser.add_argument("--load-from", type=str, default="E:\\wqx\\Depth-Anything-V2-main\\checkpoints\\depth_anything_v2_metric_hypersim_vitl.pth")
    parser.add_argument("--max-depth", type=float, default=20.0)
    parser.add_argument("--grayscale", action="store_true", help="show depth in grayscale (default: color)")
    parser.add_argument("--pred-only", action="store_true", help="only show depth (no concat)")
    parser.add_argument("--use-dshow", action="store_true", help="use DirectShow on Windows")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    DEVICE = "cuda" if torch.cuda.is_available() else "mps" if torch.backends.mps.is_available() else "cpu"
    print(f"[INFO] Device: {DEVICE}")

    # ===== Model =====
    model_configs = {
        "vits": {"encoder": "vits", "features": 64, "out_channels": [48, 96, 192, 384]},
        "vitb": {"encoder": "vitb", "features": 128, "out_channels": [96, 192, 384, 768]},
        "vitl": {"encoder": "vitl", "features": 256, "out_channels": [256, 512, 1024, 1024]},
        "vitg": {"encoder": "vitg", "features": 384, "out_channels": [1536, 1536, 1536, 1536]},
    }

    depth_anything = DepthAnythingV2(**{**model_configs[args.encoder], "max_depth": args.max_depth})
    depth_anything.load_state_dict(torch.load(args.load_from, map_location="cpu"))
    depth_anything = depth_anything.to(DEVICE).eval()
    print("[INFO] Model loaded.")

    # ===== Camera =====
    api = cv2.CAP_DSHOW if args.use_dshow else 0
    cap = cv2.VideoCapture(args.camera, api) if api != 0 else cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError("Failed to open camera")

    # 尝试设置成 1920x1080
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[INFO] Camera resolution in use: {w} x {h}")

    # ===== ROI params (你指定的) =====
    x0, y0, side = 803, 146, 788

    # 圆形 mask（直径=side）
    circle_mask = make_circle_mask(side)

    # colormap
    cmap = matplotlib.colormaps.get_cmap("Spectral")

    save_idx = 0
    cv2.namedWindow("DepthAnythingV2 ROI", cv2.WINDOW_NORMAL)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 防越界裁 ROI
        H, W = frame.shape[:2]
        x = max(0, min(x0, W - side))
        y = max(0, min(y0, H - side))
        roi = frame[y:y + side, x:x + side]

        # 圆外不参与推理：灰度弱化（不resize，不模糊 ROI 本身）
        roi_for_model = apply_roi_circle_preprocess(
            roi, circle_mask,
            blur_ksize=51,
            outside_strength=0.10  # 圆外更弱一点
        )

        # ===== infer =====
        depth = depth_anything.infer_image(roi_for_model, args.input_size)  # (side, side) float

        # 可视化 depth（0~255）
        depth_u8 = normalize_depth_to_uint8(depth)

        # 圆外直接置 0（不显示，也保持一致）
        depth_u8 = (depth_u8 * circle_mask).astype(np.uint8)

        if args.grayscale:
            depth_vis = cv2.cvtColor(depth_u8, cv2.COLOR_GRAY2BGR)
        else:
            depth_color = (cmap(depth_u8)[:, :, :3] * 255).astype(np.uint8)[:, :, ::-1]  # RGB->BGR
            depth_vis = (depth_color * circle_mask[..., None]).astype(np.uint8)

        # 拼接显示
        if args.pred_only:
            show = depth_vis
        else:
            split = np.ones((side, 30, 3), dtype=np.uint8) * 255
            show = cv2.hconcat([roi_for_model, split, depth_vis])

        cv2.imshow("DepthAnythingV2 ROI", show)
        key = cv2.waitKey(1) & 0xFF

        # q 退出
        if key == ord("q"):
            break

        # s 保存当前帧（ROI原图/深度图）
        if key == ord("s"):
            save_idx += 1
            cv2.imwrite(os.path.join(args.outdir, f"roi_{save_idx:04d}.png"), roi_for_model)
            cv2.imwrite(os.path.join(args.outdir, f"depth_{save_idx:04d}.png"), depth_vis)
            print(f"[SAVE] {save_idx:04d} saved to {args.outdir}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
