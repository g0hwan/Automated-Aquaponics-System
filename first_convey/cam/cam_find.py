import cv2

cap = None
for idx in range(0, 4):
    tmp = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
    if tmp.isOpened():
        ret, _ = tmp.read()
        if ret:
            cap = tmp
            print("사용할 카메라 인덱스:", idx)
            break
    tmp.release()

if cap is None:
    print("카메라를 찾지 못했습니다. (줌/OBS/카메라 앱 종료 후 재시도)")
    exit()