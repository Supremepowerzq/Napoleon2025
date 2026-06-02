import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import matplotlib
from PIL import Image

# =========================
# 你的工程：Unet + config（电机控制用）
# =========================
parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_path)

from DLpredict import Unet
from config import update_config

# =========================
# 0) 路径设置（放在最前，避免 ROOT 未定义）
# =========================
ROOT = Path(__file__).resolve().parent

# ★改成你真实的 Depth-Anything-V2-main 路径（使用仓库内自带版本）
REPO_ROOT = Path(r"G:\zq\Depth-Anything-V2-main")
METRIC_ROOT = REPO_ROOT / "metric_depth"
sys.path.insert(0, str(REPO_ROOT))  # 添加主目录到路径
sys.path.insert(0, str(METRIC_ROOT))  # 添加metric_depth目录到路径（优先）

from metric_depth.depth_anything_v2.dpt import DepthAnythingV2

# ★改成你真实的权重路径（pth 文件在哪里就写哪里）
LOAD_FROM = Path(r"G:\zq\Depth-Anything-V2-main\checkpoints\depth_anything_v2_metric_hypersim_vitl.pth")

# =========================
# 1) 参数区（你主要改这里）
# =========================
# ---- 统一 ROI（788 ROI）----
x0, y0, side = 400, 146, 788

# ---- 去畸变 maps（必须与 ROI 一致生成）----
# 直接定义绝对路径字符串
MAPS_NPZ = r"G:\zq\Depth-Anything-V2-main\undistort_maps_roi788.npz"
# 验证文件是否存在
import os
assert os.path.exists(MAPS_NPZ), f"Missing: {MAPS_NPZ}"

# ---- DAV2 metric 参数 ----
ENCODER = "vitl"
assert LOAD_FROM.exists(), f"Missing checkpoint: {LOAD_FROM}"
INPUT_SIZE = 518
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

# ---- 你的 Z 标定（你说：预测1000mm实际10mm → 先用0.01）----
A_SCALE = 0.01
B_BIAS_MM = 0.0

# ---- 圆外弱化（给深度模型用）----
BLUR_KSIZE = 51
OUTSIDE_STRENGTH = 0.10

# ---- 深度门控（防跳）----
Z_MIN_MM = 5.0
Z_MAX_MM = 300.0
Z_JUMP_MM = 30.0
INLIER_MIN = 0.25

# ---- 推进逻辑：>15mm 前进，<=10mm 停止（中间也停，保守）----
Z_ADVANCE_MM = 15.0
Z_STOP_MM = 10.0

# ---- 安全限速（非常保守，先小一点）----
SAFE_MAX_FORWARD = 8   # 推进最大值（正号前进）
SAFE_MAX_TURN = 8      # 转向最大值

# ---- 键盘控制 ----
KEY_ESTOP = ord('x')    # 急停
KEY_PAUSE = ord('p')    # 暂停/恢复
KEY_CLEAR = ord('c')    # 清空深度状态并停
KEY_QUIT_Q = ord('q')   # 退出（ESC 也退出）

# ---- 转向控制参数 ----
PixelR = 5
Multiple = 8
M = Multiple

# ---- HSV 阈值（用于从 Unet 渲染图里抠结石轮廓）----
lower_stone = np.array([98, 157, 88])
upper_stone = np.array([125, 255, 255])

# =========================
# 2) 工具函数
# =========================
cmap = matplotlib.colormaps.get_cmap("Spectral")


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def make_circle_mask(side: int) -> np.ndarray:
    cx = side // 2
    cy = side // 2
    r = side // 2
    Y, X = np.ogrid[:side, :side]
    return (((X - cx) ** 2 + (Y - cy) ** 2) <= r * r).astype(np.uint8)


def apply_roi_circle_preprocess(img, mask, blur_ksize=51, outside_strength=0.1):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray3 = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if blur_ksize % 2 == 0:
        blur_ksize += 1
    gray3 = cv2.GaussianBlur(gray3, (blur_ksize, blur_ksize), 0)
    m = mask[..., None].astype(np.float32)
    out = img * m + gray3 * (1 - m) * outside_strength
    return np.clip(out, 0, 255).astype(np.uint8)


def normalize_depth_to_uint8(depth):
    dmin, dmax = float(depth.min()), float(depth.max())
    if dmax - dmin < 1e-6:
        return np.zeros_like(depth, dtype=np.uint8)
    return ((depth - dmin) / (dmax - dmin) * 255).astype(np.uint8)


def robust_depth_from_mask(depth_m, mask, erode_r=3):
    m = (mask > 0).astype(np.uint8)
    if erode_r > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * erode_r + 1, 2 * erode_r + 1))
        m = cv2.erode(m, k, iterations=1)

    vals = depth_m[m > 0]
    if vals.size < 80:
        return None, 0.0

    z_med = float(np.median(vals))
    mad = float(np.median(np.abs(vals - z_med))) + 1e-6
    inliers = vals[np.abs(vals - z_med) < 3.5 * 1.4826 * mad]
    if inliers.size < 30:
        return z_med, 0.0

    return float(np.median(inliers)), float(inliers.size / vals.size)


def linear_interpolation(value, low, high, low_speed, high_speed):
    if abs(value) <= abs(low):
        return low_speed
    elif abs(value) >= abs(high):
        return high_speed
    else:
        return low_speed + (high_speed - low_speed) * (value - low) / (high - low)


def apply_motor_command(speed_pf, vx, vy):
    """把速度写入 config，并返回实际写入值（已限幅）"""
    speed_pf = int(clamp(speed_pf, -SAFE_MAX_FORWARD, SAFE_MAX_FORWARD))
    vx = int(clamp(vx, -SAFE_MAX_TURN, SAFE_MAX_TURN))
    vy = int(clamp(vy, -SAFE_MAX_TURN, SAFE_MAX_TURN))

    update_config("speed_pf", speed_pf)
    update_config("speed_pt", [vx, vy])
    return speed_pf, vx, vy


# =========================
# 3) 加载去畸变 maps & 内参（ROI 坐标系）
# =========================
maps = np.load(str(MAPS_NPZ))
map1, map2 = maps["map1"], maps["map2"]
Knew = maps["Knew"]
fx, fy = float(Knew[0, 0]), float(Knew[1, 1])
cx_k, cy_k = float(Knew[0, 2]), float(Knew[1, 2])

# =========================
# 4) 加载 DepthAnythingV2 metric
# =========================
model_cfg = {
    "vitl": {"encoder": "vitl", "features": 256, "out_channels": [256, 512, 1024, 1024]}
}
depth_anything = DepthAnythingV2(**model_cfg[ENCODER], max_depth=20.0)
depth_anything.load_state_dict(torch.load(str(LOAD_FROM), map_location="cpu", weights_only=True))
depth_anything = depth_anything.to(DEVICE).eval()
torch.backends.cudnn.benchmark = True
torch.set_float32_matmul_precision("high")

# =========================
# 5) Unet 初始化
# =========================
unet = Unet()

# =========================
# 6) 摄像头
# =========================
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

circle_mask = make_circle_mask(side)

# =========================
# 7) 状态变量（用于Z门控/防抖）
# =========================
Z_prev_mm = None
Z_filt_mm = None  # 额外再做一层简单滤波

estop = False
pause_control = False

fps_smooth = 0.0
frame_count = 0

print("Keys: x=ESTOP | p=Pause/Resume | c=Clear&Stop | q/ESC=Quit")
print(f"SAFE_MAX_FORWARD={SAFE_MAX_FORWARD}, SAFE_MAX_TURN={SAFE_MAX_TURN}")

# 启动先停
apply_motor_command(0, 0, 0)

# =========================
# 8) 主循环
# =========================
while True:
    t1 = time.time()
    ok, frame_full = cap.read()
    if not ok:
        print("Failed to read camera")
        break

    # ---- ROI 裁剪 ----
    if frame_count == 0:
        print(f"[DIAG] frame_full.shape={frame_full.shape}  x0={x0} y0={y0} side={side}")
        frame_count = 1

    # ---- 调试：全帧显示（缩小一半），红框标当前ROI，按 'd' 关闭 ----
    if not getattr(globals().get('_hide_debug', type('', (), {'val': False})()), 'val', False):
        dbg = cv2.resize(frame_full, (960, 540))
        sx, sy = 960 / frame_full.shape[1], 540 / frame_full.shape[0]
        cv2.rectangle(dbg,
                      (int(x0 * sx), int(y0 * sy)),
                      (int((x0 + side) * sx), int((y0 + side) * sy)),
                      (0, 0, 255), 2)
        cv2.putText(dbg, f"RED=current ROI x0={x0} y0={y0}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imshow("full_frame_debug", dbg)

    roi_raw = frame_full[y0:y0 + side, x0:x0 + side]
    if roi_raw.shape[0] != side or roi_raw.shape[1] != side:
        cv2.imshow("full", frame_full)
        key = cv2.waitKey(1) & 0xFF
        if key in [27, KEY_QUIT_Q]:
            break
        continue

    # ---- 去畸变 ----
    und = cv2.remap(roi_raw, map1, map2, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    # ---- 深度推理（米）----
    und_for_model = apply_roi_circle_preprocess(und, circle_mask, BLUR_KSIZE, OUTSIDE_STRENGTH)
    depth_m = depth_anything.infer_image(und_for_model, INPUT_SIZE).astype(np.float32)

    # ---- depth 可视化 ----
    depth_u8 = normalize_depth_to_uint8(depth_m)
    depth_color = (cmap(depth_u8)[:, :, :3] * 255).astype(np.uint8)[:, :, ::-1]
    depth_vis = (depth_color * circle_mask[..., None]).astype(np.uint8)

    # ---- Unet 分割（在 und 上跑，坐标一致）----
    und_rgb = cv2.cvtColor(und, cv2.COLOR_BGR2RGB)
    pil_img = Image.fromarray(und_rgb)
    unet_vis_rgb = np.array(unet.detect_image(pil_img))
    unet_vis = cv2.cvtColor(unet_vis_rgb, cv2.COLOR_RGB2BGR)

    # ---- 从 Unet 渲染图抠结石轮廓 ----
    hsv = cv2.cvtColor(unet_vis, cv2.COLOR_BGR2HSV)
    mask_stone = cv2.inRange(hsv, lower_stone, upper_stone)
    contours, _ = cv2.findContours(mask_stone.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # ---- 默认速度（必须先定义，避免 Pylance 未定义）----
    speed_FR = 0
    vx = 0
    vy = 0

    # ---- 模式字符串 ----
    status = "ESTOP" if estop else ("PAUSE" if pause_control else "RUN")

    center_x, center_y = side // 2, side // 2

    if len(contours) > 0:
        cnt = max(contours, key=cv2.contourArea)

        Mmt = cv2.moments(cnt)
        if Mmt["m00"] != 0:
            u = int(Mmt["m10"] / Mmt["m00"])
            v = int(Mmt["m01"] / Mmt["m00"])
        else:
            u, v = center_x, center_y

        cv2.drawContours(und, [cnt], -1, (255, 255, 0), 2, cv2.LINE_AA)
        cv2.circle(und, (u, v), 4, (0, 0, 255), -1, cv2.LINE_AA)

        # --- 用轮廓做 mask 去取稳健深度 ---
        stone_mask = np.zeros((side, side), dtype=np.uint8)
        cv2.drawContours(stone_mask, [cnt], -1, 255, thickness=cv2.FILLED, lineType=cv2.LINE_AA)
        stone_mask = (stone_mask * circle_mask).astype(np.uint8)

        Z_pred_m, inlier = robust_depth_from_mask(depth_m, stone_mask, erode_r=3)

        z_ok = False
        Z_pred_mm = None
        Z_mm = None

        if Z_pred_m is not None:
            Z_pred_mm = Z_pred_m * 1000.0
            Z_mm = A_SCALE * Z_pred_mm + B_BIAS_MM

            # 门控：范围 + inlier + 跳变
            if (Z_MIN_MM <= Z_mm <= Z_MAX_MM) and (inlier >= INLIER_MIN):
                if Z_prev_mm is None or abs(Z_mm - Z_prev_mm) <= Z_JUMP_MM:
                    z_ok = True
                else:
                    # 跳变太大 → 用上一帧
                    Z_mm = Z_prev_mm
                    z_ok = True

        if z_ok:
            Z_prev_mm = Z_mm

            # 额外防抖：一阶低通（让它别一直抖）
            if Z_filt_mm is None:
                Z_filt_mm = Z_mm
            else:
                alpha = 0.2  # 越小越稳，响应越慢
                Z_filt_mm = (1 - alpha) * Z_filt_mm + alpha * Z_mm

            # ===== 推进逻辑（只用 Z_filt_mm）=====
            if Z_filt_mm > Z_ADVANCE_MM:
                speed_FR = +SAFE_MAX_FORWARD
            elif Z_filt_mm <= Z_STOP_MM:
                speed_FR = 0
            else:
                speed_FR = 0  # 保守

            # ===== 转向逻辑（居中）=====
            Dx = center_x - u
            Dy = center_y - v

            # 这里直接用 SAFE_MAX_TURN 做上限（比 TURN_COEFF 更可控）
            TC = SAFE_MAX_TURN
            TC_S = max(1, SAFE_MAX_TURN // 3)

            red_left = Dx > PixelR
            red_right = Dx < -PixelR
            red_up = Dy < -PixelR
            red_down = Dy > PixelR

            vx = 0
            vy = 0

            if red_left:
                if PixelR < abs(Dx) < M * PixelR:
                    vx = -TC_S
                elif M * PixelR < abs(Dx) < 2 * M * PixelR:
                    vx = linear_interpolation(Dx, M * PixelR, 2 * M * PixelR, -TC_S, -TC)
                else:
                    vx = -TC

            if red_right:
                if PixelR < abs(Dx) < M * PixelR:
                    vx = +TC_S
                elif M * PixelR < abs(Dx) < 2 * M * PixelR:
                    vx = linear_interpolation(Dx, -M * PixelR, -2 * M * PixelR, TC_S, TC)
                else:
                    vx = +TC

            if red_up:
                if PixelR < abs(Dy) < M * PixelR:
                    vy = -TC_S
                elif M * PixelR < abs(Dy) < 2 * M * PixelR:
                    vy = linear_interpolation(Dy, -M * PixelR, -2 * M * PixelR, -TC_S, -TC)
                else:
                    vy = -TC

            if red_down:
                if PixelR < abs(Dy) < M * PixelR:
                    vy = +TC_S
                elif M * PixelR < abs(Dy) < 2 * M * PixelR:
                    vy = linear_interpolation(Dy, M * PixelR, 2 * M * PixelR, TC_S, TC)
                else:
                    vy = +TC

            # ===== 图上显示 =====
            cv2.putText(
                und,
                f"Zpred={Z_pred_mm:.1f}mm  Zcal={Z_mm:.1f}mm  Zfilt={Z_filt_mm:.1f}mm  inlier={inlier:.2f}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA
            )
        else:
            # 深度不可信 → 必须停
            speed_FR = 0
            vx = 0
            vy = 0
            cv2.putText(
                und,
                "Depth invalid (gated) -> motors=0",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA
            )
    else:
        # 没检测到 → 停
        speed_FR = 0
        vx = 0
        vy = 0
        cv2.putText(
            und,
            "No stone detected -> motors=0",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA
        )

    # =========================
    # 急停/暂停 优先级最高
    # =========================
    if estop or pause_control:
        speed_FR = 0
        vx = 0
        vy = 0

    # 写入 config（并显示最终写入值）
    pf_applied, vx_applied, vy_applied = apply_motor_command(speed_FR, vx, vy)

    # =========================
    # FPS + 速度显示（这里一定在 frame/速度都存在之后）
    # =========================
    dt = time.time() - t1
    fps = 1.0 / max(dt, 1e-6)
    fps_smooth = fps if fps_smooth == 0.0 else 0.9 * fps_smooth + 0.1 * fps

    cv2.putText(
        und,
        f"FPS {fps_smooth:.1f}  MODE={status}",
        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA
    )
    cv2.putText(
        und,
        f"cmd_pf={pf_applied}  turn(vx,vy)=({vx_applied},{vy_applied})",
        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA
    )

    # =========================
    # 显示窗口
    # =========================
    cv2.imshow("undist_roi", und)
    cv2.imshow("unet_vis_roi", unet_vis)
    cv2.imshow("depth_vis_roi", depth_vis)

    # =========================
    # 键盘控制
    # =========================
    key = cv2.waitKey(1) & 0xFF

    if key == KEY_ESTOP:
        estop = True
        pause_control = True
        apply_motor_command(0, 0, 0)
        print("[ESTOP] Motors set to 0 and locked (pause).")

    if key == KEY_PAUSE:
        pause_control = not pause_control
        if pause_control:
            apply_motor_command(0, 0, 0)
            print("[PAUSE] Control paused. Motors=0.")
        else:
            print("[PAUSE] Control resumed.")

    if key == KEY_CLEAR:
        Z_prev_mm = None
        Z_filt_mm = None
        apply_motor_command(0, 0, 0)
        print("[CLEAR] Z state cleared. Motors=0.")

    if key in [27, KEY_QUIT_Q]:
        apply_motor_command(0, 0, 0)
        break

# 退出前务必停
apply_motor_command(0, 0, 0)
cap.release()
cv2.destroyAllWindows()
