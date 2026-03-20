import cv2

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # Windows建议用 CAP_DSHOW
if not cap.isOpened():
    raise RuntimeError("Cannot open camera 0")

ret, frame = cap.read()
print("Actual frame shape (H,W,C):", frame.shape)  # 这个最真实
print("CAP_PROP W,H:", cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

cap.release()
