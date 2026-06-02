# real_time_bronchoscopy_detection.py
from ultralytics import YOLO
import cv2
import time
import numpy as np
import os
from PIL import Image, ImageDraw, ImageFont

# 全局变量
detection_active = False
recording = False
video_writer = None
recorded_frames = 0
actual_width = 0
actual_height = 0

# 置信度阈值
CONFIDENCE_THRESHOLD = 0.30

# ============================================================
# 文字绘制工具（PIL，避免 cv2 字体问题）
# ============================================================
def put_text(img, text, pos, font_size=20, color=(0, 255, 0)):
    """Draw text on image using PIL to avoid cv2 font rendering issues."""
    pil_img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil_img)
    try:
        font = ImageFont.truetype("msyh.ttc", font_size)
    except Exception:
        font = ImageFont.load_default()
    draw.text(pos, text, font=font, fill=color + (255,) if len(color) == 3 else color)
    return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)

# ============================================================
# Bronchial tree anatomy state machine
# Hierarchy: TR -> RMB/LMB -> RUL/BI -> RML/RLL
# Rule: can only go deeper, must retreat before switching branches
# ============================================================
HIGH_CONF_THRESHOLD = 0.75   # 高置信度阈值：触发从父节点回退
RETREAT_KEYWORDS = {
    # 同父节点的子节点互为回退关键词（如 RUL 和 BI 共父 RMB）
    'RUL': {'BI'},
    'BI':  {'RUL'},
    'LUB': {'LLB'},
    'LLB': {'LUB'},
}

BRONCHIAL_TREE = {
    'TR':  ['RMB', 'LMB'],          # Trachea -> left/right main bronchus
    'RMB': ['RUL', 'BI'],           # Right main -> upper lobe or bronchus intermedius
    'BI':  ['RML', 'RLL'],          # Bronchus intermedius -> middle/lower lobe
    'LMB': ['LUB', 'LLB'],          # Left main -> upper/lower lobe
    # 子节点只能回到父节点
    'RUL': ['RMB'], 'RML': ['BI'], 'RLL': ['BI'],
    'LUB': ['LMB'], 'LLB': ['LMB'],
}

# 父节点映射：用于高置信度回退
PARENT_MAP = {child: parent for parent, children in BRONCHIAL_TREE.items()
              for child in children}

# Required confirm frames per transition (critical bifurcations are faster)
CONFIRM_FRAMES = {
    ('TR', 'RMB'): 3, ('TR', 'LMB'): 3,       # Main bifurcation: stable
    ('RMB', 'RUL'): 1, ('RMB', 'BI'): 1,       # RMB->RUL/BI: fast, BI is short
    ('LMB', 'LUB'): 2, ('LMB', 'LLB'): 2,      # Left main: moderate
    ('BI', 'RML'): 2, ('BI', 'RLL'): 2,         # BI: stable
    # 回退转移也需要确认
    ('RUL', 'RMB'): 2, ('BI', 'RMB'): 2,
    ('RML', 'BI'): 2, ('RLL', 'BI'): 2,
    ('LUB', 'LMB'): 2, ('LLB', 'LMB'): 2,
}
DEFAULT_CONFIRM_FRAMES = 3

current_position = 'TR'           # Current anatomical position
position_confirm_count = {}        # Confirm frame counts per position
last_confirmed_position = 'TR'    # Last confirmed position (for display)


def validate_transition(from_pos, to_pos):
    """Validate if the anatomical transition is allowed."""
    if from_pos == to_pos:
        return True
    allowed = BRONCHIAL_TREE.get(from_pos, [])
    return to_pos in allowed


def get_valid_next_positions(current_pos):
    """Get valid next positions from current position."""
    return BRONCHIAL_TREE.get(current_pos, [])


def filter_by_anatomy(raw_detections, current_pos, confidence_threshold):
    """Filter detection results using the anatomy state machine.

    Args:
        raw_detections: list of (class_name, confidence) tuples from model
        current_pos: current anatomical position string
        confidence_threshold: minimum confidence to consider

    Returns:
        (best_position, best_confidence) or (None, None)
    """
    if not raw_detections:
        return None, None

    # 标准候选：当前位置 + 直接子节点
    valid_next = set(get_valid_next_positions(current_pos))
    valid_next.add(current_pos)

    # 高置信度扩展候选：父节点 + 关键词兄弟节点
    parent = PARENT_MAP.get(current_pos)
    extra_allowed = set()
    if parent:
        extra_allowed.add(parent)
        extra_allowed.update(BRONCHIAL_TREE.get(parent, []))
    keyword_siblings = RETREAT_KEYWORDS.get(current_pos, set())
    extra_allowed.update(keyword_siblings)

    candidates = []
    for cls, conf in raw_detections:
        if cls not in valid_next and cls not in extra_allowed:
            continue
        if conf < confidence_threshold:
            continue
        candidates.append((cls, conf))

    if not candidates:
        return None, None

    candidates.sort(key=lambda x: x[1], reverse=True)
    best = candidates[0]

    if best[0] != current_pos:
        # 检查是否可以用高置信度触发扩展转移
        is_extended = (best[0] in extra_allowed) and (best[0] not in valid_next)
        if is_extended and best[1] >= HIGH_CONF_THRESHOLD:
            required = CONFIRM_FRAMES.get((current_pos, best[0]), DEFAULT_CONFIRM_FRAMES)
            cnt = position_confirm_count.get(best[0], 0) + 1
            position_confirm_count[best[0]] = cnt
            if cnt < required:
                return current_pos, None
            else:
                position_confirm_count.clear()
                return best[0], best[1]

        # 标准转移
        required = CONFIRM_FRAMES.get((current_pos, best[0]), DEFAULT_CONFIRM_FRAMES)
        cnt = position_confirm_count.get(best[0], 0) + 1
        position_confirm_count[best[0]] = cnt
        if cnt < required:
            return current_pos, None
        else:
            position_confirm_count.clear()
            position_confirm_count[current_pos] = 0
            return best[0], best[1]
    else:
        position_confirm_count[current_pos] = 999  # 持续确认
        return best[0], best[1]


def reset_state_machine():
    """Reset the state machine when detection stops."""
    global current_position, position_confirm_count, last_confirmed_position
    current_position = 'TR'
    position_confirm_count = {}
    last_confirmed_position = 'TR'

# ROI selection variables
drawing = False
start_point = None
end_point = None
crop_roi = None  # (x, y, w, h) - selected ROI in original frame coords
roi_win_w = 1280  # ROI selection window width
roi_win_h = 720   # ROI selection window height

def mouse_callback(event, x, y, flags, param):
    """Mouse callback for ROI selection."""
    global drawing, start_point, end_point, crop_roi

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        start_point = (x, y)
        end_point = (x, y)
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            end_point = (x, y)
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        if start_point and end_point:
            x1, y1 = start_point
            x2, y2 = end_point
            roi_x = min(x1, x2)
            roi_y = min(y1, y2)
            roi_w = abs(x2 - x1)
            roi_h = abs(y2 - y1)
            if roi_w > 10 and roi_h > 10:
                crop_roi = (roi_x, roi_y, roi_w, roi_h)
                print(f"[ROI] Selected: x={roi_x}, y={roi_y}, w={roi_w}, h={roi_h}")

def detect_capture_card():
    """Auto-detect available video capture devices."""
    print("Detecting video capture devices...")
    connection_methods = [
        (0, "Device 0"), (1, "Device 1"), (2, "Device 2"), (3, "Device 3")
    ]
    available_devices = []
    tested_devices = set()

    for method in connection_methods:
        index, description = method
        device_key = f"{index}"
        if device_key in tested_devices:
            continue
        tested_devices.add(device_key)

        try:
            cap = cv2.VideoCapture(index)
            if cap.isOpened():
                ret = False
                frame = None
                for attempt in range(3):
                    ret, frame = cap.read()
                    if ret and frame is not None and frame.size > 0:
                        if np.mean(frame) > 10:
                            break
                    time.sleep(0.1)

                if ret and frame is not None and frame.size > 0:
                    available_devices.append((index, description, frame.shape))
                    print(f"[OK] Found device: {description} - {frame.shape[1]}x{frame.shape[0]}")
                else:
                    print(f"[Warn] Device {description} returned invalid frame")
                cap.release()
            else:
                print(f"[Warn] Cannot open {description}")
        except Exception as e:
            print(f"[Error] Failed to probe {description}: {e}")

    return available_devices


def real_time_bronchoscopy_simple(model_path, device_index=0, use_dshow=True):
    """简化的内窥镜实时检测 - 支持交互式框选显示区域"""

    # 使用全局变量
    global detection_active, recording, video_writer, recorded_frames
    global actual_width, actual_height
    global drawing, start_point, end_point, crop_roi
    global current_position, position_confirm_count, last_confirmed_position

    # 强制使用 CPU，绕过 CUDA 依赖缺失问题
    model = YOLO(model_path, task='detect')
    model.to('cuda')
    print(f"[Model] Loaded: {model_path} (GPU)")

    # 打开视频采集卡
    print(f"[Camera] Opening device {device_index}...")
    if use_dshow:
        cap = cv2.VideoCapture(device_index, cv2.CAP_DSHOW)
    else:
        cap = cv2.VideoCapture(device_index)

    if not cap.isOpened():
        print(f"[Camera] Failed to open device: {device_index}")
        return

    # 给设备充足的初始化时间
    print("[Camera] Waiting for device init...")
    time.sleep(2)
    
    # 丢弃前几帧以确保获得有效画面
    for _ in range(10):
        ret, _ = cap.read()
        if ret:
            time.sleep(0.05)
    
    # 验证设备是否真的能读取帧
    ret, test_frame = cap.read()
    if not ret or test_frame is None:
        print(f"[Camera] Device opened but cannot read frames")
        cap.release()
        return

    print(f"[Camera] Device ready")

    # 设置采集卡参数 - 使用较高分辨率以获得完整画面
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # 获取实际参数
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"[Camera] Connected - Resolution: {actual_width}x{actual_height}")

    # 创建窗口用于框选
    window_name = 'Bronchoscopy ROI Selection'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 720)
    cv2.setMouseCallback(window_name, mouse_callback)

    print("\n[ROI] Draw a rectangle to select the detection area:")
    print("   - Hold left mouse button and drag")
    print("   - Press ENTER to confirm and start detection")
    print("   - Press 'q' to quit")

    # 框选阶段 - 让用户选择显示区域
    roi_selection_active = True
    frame_count_init = 0
    selected_roi_final = None
    
    while roi_selection_active:
        ret, frame = cap.read()
        if not ret:
            print("[Camera] Failed to read frame")
            break

        frame_count_init += 1

        # 把原始帧 resize 到窗口尺寸，保证鼠标坐标和画面坐标一致
        display_frame = cv2.resize(frame, (roi_win_w, roi_win_h))

        # 绘制正在框选的矩形（直接用窗口坐标）
        if drawing and start_point and end_point:
            sx, sy = start_point
            ex, ey = end_point
            cv2.rectangle(display_frame,
                         (int(min(sx, ex)), int(min(sy, ey))),
                         (int(max(sx, ex)), int(max(sy, ey))),
                         (0, 255, 0), 2)

        # 绘制已框选的矩形（直接用窗口坐标）
        if crop_roi:
            roi_x, roi_y, roi_w, roi_h = crop_roi
            cv2.rectangle(display_frame,
                         (roi_x, roi_y),
                         (roi_x + roi_w, roi_y + roi_h),
                         (0, 255, 0), 3)
            display_frame = put_text(display_frame, "Press ENTER to confirm", (20, 40), 24, (0, 255, 0))

        # 显示提示信息
        display_frame = put_text(display_frame, "Draw ROI to select detection area", (20, 30), 24, (255, 255, 0))
        display_frame = put_text(display_frame, f"Camera: {actual_width}x{actual_height}", (20, 75), 20, (255, 255, 0))

        cv2.imshow(window_name, display_frame)

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):  # 退出
            print("[Quit] User cancelled")
            cap.release()
            cv2.destroyAllWindows()
            return
        elif key == 13:  # 回车键 - 确认框选
            if crop_roi:
                selected_roi_final = crop_roi
                roi_selection_active = False
                print("[ROI] Confirmed, starting detection...")
                time.sleep(1)
            else:
                print("[ROI] Please draw a rectangle first")

    if not selected_roi_final:
        print("[Error] No valid ROI selected")
        cap.release()
        cv2.destroyAllWindows()
        return

    camera_roi = selected_roi_final
    roi_x, roi_y, roi_w, roi_h = camera_roi
    print(f"[ROI] Detection area set: x={roi_x}, y={roi_y}, w={roi_w}, h={roi_h}")

    # 重新创建窗口用于检测（保持原始分辨率）
    window_name = 'Bronchoscopy Detection'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, actual_width, actual_height)

    print("\n[Controls]")
    print("   - ENTER: start/stop detection")
    print("   - 'v': start/stop recording")
    print("   - 's': save current frame")
    print("   - 'q': quit")
    print(f"   - Confidence threshold: {CONFIDENCE_THRESHOLD}")

    frame_count = 0
    fps_times = []

    try:
        while True:
            start_time = time.time()

            ret, frame = cap.read()
            if not ret:
                print("[Camera] Failed to read frame")
                break

            display_frame = frame.copy()
            display_width, display_height = actual_width, actual_height

            # 在检测窗口中标注ROI区域（把窗口坐标映射回原始坐标）
            if camera_roi:
                roi_x_win, roi_y_win, roi_w_win, roi_h_win = camera_roi
                scale_x = actual_width / roi_win_w
                scale_y = actual_height / roi_win_h
                roi_x = int(roi_x_win * scale_x)
                roi_y = int(roi_y_win * scale_y)
                roi_w = int(roi_w_win * scale_x)
                roi_h = int(roi_h_win * scale_y)
                cv2.rectangle(display_frame, (roi_x, roi_y),
                             (roi_x + roi_w, roi_y + roi_h), (0, 255, 255), 2)

            # 如果识别激活，从ROI区域裁剪后送入模型推理
            # crop_roi 存的是窗口坐标(1280x720)，需要映射回原始坐标(1920x1080)
            validated_position = None
            validated_confidence = None
            validated_box = None

            if detection_active and camera_roi:
                roi_x_win, roi_y_win, roi_w_win, roi_h_win = camera_roi
                scale_x = actual_width / roi_win_w
                scale_y = actual_height / roi_win_h
                roi_x = int(roi_x_win * scale_x)
                roi_y = int(roi_y_win * scale_y)
                roi_w = int(roi_w_win * scale_x)
                roi_h = int(roi_h_win * scale_y)
                roi_frame = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
                results = model(roi_frame)
                result = results[0]

                if result.boxes is not None and len(result.boxes) > 0:
                    raw = []
                    all_boxes = []
                    for box in result.boxes:
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        cls = int(box.cls)
                        conf = box.conf.item()
                        class_name = model.names[cls]
                        raw.append((class_name, conf))
                        all_boxes.append((x1, y1, x2, y2))

                    print(f"[Debug] Raw detections: {raw}")
                    print(f"[Debug] Current position: {current_position}, confirm_counts: {position_confirm_count}")

                    validated_position, validated_confidence = filter_by_anatomy(
                        raw, current_position, CONFIDENCE_THRESHOLD
                    )

                    print(f"[Debug] After filter -> pos={validated_position}, conf={validated_confidence}")

                    # 找到了解剖学上有效的检测结果
                    if validated_position is not None and validated_position != current_position:
                        prev_pos = current_position
                        current_position = validated_position
                        last_confirmed_position = validated_position
                        print(f"[State] {prev_pos} -> {validated_position} (conf: {validated_confidence:.2f})")

                    # 找到当前状态下对应的检测框并映射坐标
                    validated_box = None
                    if validated_position:
                        for i, (cls, _) in enumerate(raw):
                            if cls == validated_position:
                                x1, y1, x2, y2 = all_boxes[i]
                                abs_x1 = x1 + roi_x
                                abs_y1 = y1 + roi_y
                                abs_x2 = x2 + roi_x
                                abs_y2 = y2 + roi_y
                                validated_box = (abs_x1, abs_y1, abs_x2, abs_y2)
                                break

                    # 在完整画面上绘制检测框和标签
                    if validated_box and validated_confidence is not None:
                        abs_x1, abs_y1, abs_x2, abs_y2 = [int(v) for v in validated_box]
                        cv2.rectangle(display_frame, (abs_x1, abs_y1), (abs_x2, abs_y2), (0, 255, 0), 3)
                        label = f"{validated_position} {validated_confidence:.2f}"
                        cv2.putText(display_frame, label, (abs_x1, abs_y1 - 10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)

            # 录制
            if recording and video_writer is not None:
                video_writer.write(display_frame)
                recorded_frames += 1

            # 计算FPS
            inference_time = time.time() - start_time
            current_fps = 1.0 / inference_time if inference_time > 0 else 0
            fps_times.append(current_fps)

            # 状态文字
            status_text = "DETECTING" if detection_active else "STANDBY"
            record_status = "REC" if recording else "IDLE"

            # 在画面左上角叠加状态信息
            cv2.rectangle(display_frame, (5, 5), (400, 155), (0, 0, 0), -1)
            display_frame = put_text(display_frame, f'FPS: {current_fps:.1f}', (10, 30), 22, (0, 255, 0))
            display_frame = put_text(display_frame, f'Status: {status_text}', (10, 60), 22, (0, 255, 0))
            display_frame = put_text(display_frame, f'Record: {record_status}', (10, 90), 22, (0, 255, 0))
            display_frame = put_text(display_frame, f'Position: {last_confirmed_position}', (10, 120), 22, (0, 255, 0))
            if detection_active and validated_position:
                display_frame = put_text(display_frame, f'Detected: {validated_position}', (10, 150), 22, (255, 200, 0))

            cv2.imshow(window_name, display_frame)

            # 键盘输入处理
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):  # 退出
                break
            if key == ord('s'):  # 保存当前帧
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(f'capture_{timestamp}.jpg', display_frame)
                print(f"[Capture] Saved: capture_{timestamp}.jpg")
            elif key == 13:  # 回车键 (ENTER) - 开始/停止识别
                detection_active = not detection_active
                status = "Started" if detection_active else "Stopped"
                print(f"[Detection] {status}")
                if not detection_active:
                    reset_state_machine()
            elif key == ord('v'):  # 'v'键 - 开始/停止录制
                if not recording:
                    # 创建专门的录制文件夹
                    recordings_dir = "bronchoscopy_recordings"
                    os.makedirs(recordings_dir, exist_ok=True)

                    # 开始录制
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    output_filename = f'{recordings_dir}/record_{timestamp}.avi'
                    fourcc = cv2.VideoWriter_fourcc(*'XVID')
                    video_writer = cv2.VideoWriter(output_filename, fourcc, 20.0, (display_width, display_height))
                    recording = True
                    recorded_frames = 0
                    print(f"[Record] Started: {output_filename}")
                else:
                    # 停止录制
                    recording = False
                    if video_writer is not None:
                        video_writer.release()
                        video_writer = None
                    print(f"[Record] Stopped, saved {recorded_frames} frames")

            frame_count += 1

            if frame_count % 100 == 0:
                avg_fps = sum(fps_times[-100:]) / min(100, len(fps_times))
                print(f"[Stats] Frames: {frame_count}, Avg FPS: {avg_fps:.1f}")

    except KeyboardInterrupt:
        print("\n[Quit] Interrupted by user")
    except Exception as e:
        print(f"[Error] {e}")

    finally:
        # 释放资源
        cap.release()
        if video_writer is not None:
            video_writer.release()
        cv2.destroyAllWindows()
        print("[Cleanup] All resources released")

        if fps_times:
            avg_fps = sum(fps_times) / len(fps_times)
            print(f"\n[Summary]")
            print(f"   Total frames: {frame_count}")
            print(f"   Avg FPS: {avg_fps:.1f}")

# 使用示例
if __name__ == '__main__':
    available_devices = detect_capture_card()
    
    if available_devices:
        print(f"\n[Device] Found {len(available_devices)} available device(s)")
        for i, (index, desc, resolution) in enumerate(available_devices):
            print(f"  {i}: {desc} - {resolution[1]}x{resolution[0]}")

        # 只使用第一个设备
        device_index = available_devices[0][0]
        device_desc = available_devices[0][1]
        print(f"\n[Device] Selected: {device_desc}")

        # 构建模型路径（刚刚导出的ONNX模型）
        model_path = r'G:\zck\yolov11\runs\train\yolo11_4-14_2_weak_aug\weights\best.pt'

        real_time_bronchoscopy_simple(
            model_path=model_path,
            device_index=device_index,
            use_dshow=True
        )
    else:
        print("[Error] No video capture device found")