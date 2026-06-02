import cv2

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

print("正在读取摄像头...")
for _ in range(5):
    cap.read()

ok, frame = cap.read()
cap.release()

if not ok:
    print("读取失败")
    exit()

actual_h, actual_w = frame.shape[:2]
print(f"摄像头实际输出分辨率: {actual_w} x {actual_h}")

# 保存原始全图，用图片查看器量精确坐标
save_path = r"G:\zq\Napoleon2025\2026\full_frame.jpg"
cv2.imwrite(save_path, frame)
print(f"\n已保存全图: {save_path}")
print("请用画图/PS等工具打开，将鼠标移到圆形区域的四个边界点，记录坐标")
print("然后告诉我：左上角(x,y) 和 右下角(x,y)\n")

# 同时也做一次鼠标点击交互获取坐标
clicks = []

def on_mouse(event, x, y, _flags, _param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # 还原到原始坐标
        real_x = int(x / scale)
        real_y = int(y / scale)
        clicks.append((real_x, real_y))
        print(f"点击 {len(clicks)}: 原始坐标 ({real_x}, {real_y})")
        if len(clicks) == 2:
            x0 = min(clicks[0][0], clicks[1][0])
            y0 = min(clicks[0][1], clicks[1][1])
            x1 = max(clicks[0][0], clicks[1][0])
            y1 = max(clicks[0][1], clicks[1][1])
            w = x1 - x0
            h = y1 - y0
            side = min(w, h)
            print(f"\n========== ROI参数 ==========")
            print(f"x0, y0, side = {x0}, {y0}, {side}")
            print(f"宽={w}, 高={h}")
            print(f"==============================")

scale = 0.35
preview = cv2.resize(frame, (int(actual_w * scale), int(actual_h * scale)))

cv2.namedWindow("点击左上角，再点右下角（ESC退出）")
cv2.setMouseCallback("点击左上角，再点右下角（ESC退出）", on_mouse)
print("在预览窗口中：先点击圆形区域左上角，再点右下角")

while True:
    disp = preview.copy()
    for c in clicks:
        cx = int(c[0] * scale)
        cy = int(c[1] * scale)
        cv2.circle(disp, (cx, cy), 8, (0, 255, 0), -1)
    if len(clicks) == 2:
        pt1 = (int(clicks[0][0]*scale), int(clicks[0][1]*scale))
        pt2 = (int(clicks[1][0]*scale), int(clicks[1][1]*scale))
        cv2.rectangle(disp, pt1, pt2, (0, 255, 0), 2)
    cv2.imshow("点击左上角，再点右下角（ESC退出）", disp)
    key = cv2.waitKey(1) & 0xFF
    if key == 27 or len(clicks) >= 2:
        break

if len(clicks) == 2:
    x0 = min(clicks[0][0], clicks[1][0])
    y0 = min(clicks[0][1], clicks[1][1])
    x1 = max(clicks[0][0], clicks[1][0])
    y1 = max(clicks[0][1], clicks[1][1])
    side = min(x1-x0, y1-y0)
    crop = frame[y0:y0+side, x0:x0+side]
    cv2.imshow("ROI验证（任意键退出）", cv2.resize(crop, (800, 800)))
    cv2.waitKey(0)

cv2.destroyAllWindows()
