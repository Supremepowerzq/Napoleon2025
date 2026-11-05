import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

SITE_ROOTS = [
    ROOT / '2026',
    ROOT / '2026' / 'Depth-Anything-V2-main',
    ROOT / '2026' / 'Depth-Anything-V2-main' / 'metric_depth',
]
for p in SITE_ROOTS:
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

import cv2
import numpy as np
import torch
from PIL import Image
from ultralytics import YOLO

from DLpredict.unet import Unet
from metric_depth.depth_anything_v2.dpt import DepthAnythingV2

from BC.data_collector import update_visual_features
from BC.inference import BCRunner


ROOT = Path(__file__).resolve().parents[3]
IMAGE_PATH = ROOT / '2026' / 'full_frame.jpg'
UNET_WEIGHT = ROOT / 'model_data' / '2.23-noon-best_epoch_weights.pth'
YOLO_WEIGHT = ROOT / 'model_data' / 'bronchus_yolo' / 'best.pt'
DEPTH_WEIGHT = Path(r'G:/zq/Depth-Anything-V2-main/checkpoints/depth_anything_v2_metric_hypersim_vitl.pth')
BC_WEIGHT = ROOT / '2026' / 'BC' / 'checkpoints' / 'bc_best.pth'


class DummyMotor:
    def get_cached_angles(self):
        return [0.0, 0.0, 0.0]


def sync():
    if torch.cuda.is_available():
        torch.cuda.synchronize()


def benchmark(fn, warmup=2, runs=10):
    times = []
    for i in range(warmup + runs):
        sync()
        t0 = time.perf_counter()
        fn()
        sync()
        t1 = time.perf_counter()
        if i >= warmup:
            times.append((t1 - t0) * 1000.0)
    return float(np.mean(times)), float(np.std(times)), len(times)


def load_image():
    img = cv2.imread(str(IMAGE_PATH))
    if img is None:
        raise FileNotFoundError(f'Cannot read image: {IMAGE_PATH}')
    crop = img[:582, :582].copy()
    return crop


def run():
    print('ROOT', ROOT)
    img = load_image()
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    pil_img = Image.fromarray(img_rgb)

    # UNet
    print('Loading UNet...')
    unet = Unet(model_path=str(UNET_WEIGHT), cuda=torch.cuda.is_available(), num_classes=3)
    if not unet.cuda:
        print('Warning: UNet not using CUDA')
    print('Warm-up UNet...')
    unet.detect_image(pil_img)
    mean_unet, std_unet, n_unet = benchmark(lambda: unet.detect_image(pil_img), warmup=2, runs=8)
    print(f'UNet latency ms: {mean_unet:.2f} ± {std_unet:.2f} over {n_unet} runs')

    # YOLO
    print('Loading YOLO...')
    yolo = YOLO(str(YOLO_WEIGHT), task='detect')
    yolo.to('cuda' if torch.cuda.is_available() else 'cpu')
    print('Warm-up YOLO...')
    _ = yolo(img, verbose=False)
    mean_yolo, std_yolo, n_yolo = benchmark(lambda: yolo(img, verbose=False), warmup=2, runs=8)
    print(f'YOLO latency ms: {mean_yolo:.2f} ± {std_yolo:.2f} over {n_yolo} runs')
    print('YOLO names:', yolo.names)

    # Depth
    print('Loading DepthAnythingV2...')
    model_cfg = {'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]}}
    depth = DepthAnythingV2(**{**model_cfg['vitl'], 'max_depth': 20.0})
    state = torch.load(str(DEPTH_WEIGHT), map_location='cpu')
    try:
        depth.load_state_dict(state)
    except TypeError:
        depth.load_state_dict(state, strict=False)
    depth_device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    depth = depth.to(depth_device).eval()
    depth_input = cv2.resize(img, (518, 518))
    print('Warm-up Depth...')
    try:
        _ = depth.infer_image(depth_input, 518)
    except Exception as exc:
        print('Depth warmup failed:', exc)
        raise
    mean_depth, std_depth, n_depth = benchmark(
        lambda: depth.infer_image(depth_input, 518), warmup=2, runs=8)
    print(f'Depth latency ms: {mean_depth:.2f} ± {std_depth:.2f} over {n_depth} runs')

    # BC GRU
    print('Loading BC model...')
    bc = BCRunner(str(BC_WEIGHT), goal_id=0, device='cuda' if torch.cuda.is_available() else 'cpu')
    motor = DummyMotor()
    update_visual_features(
        obstruction_cx=0.0, obstruction_cy=0.0, obstruction_area=0.0,
        obstruction_detected=False,
        bifur_cx=0.0, bifur_cy=0.0, bifur_area=0.0,
        bifur_detected=False,
        depth_center_mm=0.0, depth_mean_mm=0.0, depth_max_mm=0.0,
        path_left_mm=0.0, path_center_mm=0.0, path_right_mm=0.0,
    )
    print('Warm-up BC...')
    bc.step(motor)
    mean_bc, std_bc, n_bc = benchmark(lambda: bc.step(motor), warmup=2, runs=8)
    print(f'BC step latency ms: {mean_bc:.2f} ± {std_bc:.2f} over {n_bc} runs')

    results = {
        'unet_ms': mean_unet,
        'unet_std_ms': std_unet,
        'yolo_ms': mean_yolo,
        'yolo_std_ms': std_yolo,
        'depth_ms': mean_depth,
        'depth_std_ms': std_depth,
        'bc_ms': mean_bc,
        'bc_std_ms': std_bc,
    }
    print('RESULTS', results)
    return results


if __name__ == '__main__':
    run()
