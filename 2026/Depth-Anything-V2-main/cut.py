import cv2
import os

cap = cv2.VideoCapture(0)

# 设置分辨率
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
ret, frame = cap.read()
print("Actual:", frame.shape[1], frame.shape[0]) 
# 可选：关闭自动对焦/曝光
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"Resolution in use: {int(w)} x {int(h)}")

cv2.namedWindow("img", cv2.WINDOW_NORMAL)

count = 1
while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow("img", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('c'):
        cv2.imwrite(f"{count}.jpg", frame)
        print(f"Saved {count}.jpg")
        count += 1

    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
