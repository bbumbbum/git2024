import cv2 as cv
import numpy as np

# 이미지 불러오기
img = cv.imread('/home/pi/hello-git/log-git/week10/imgs/4.jpg')
if img is None:
    print("Error: Unable to load image.")
else:
    # BGR 이미지를 HSV 색상 공간으로 변환
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # 흰색 범위 설정 (HSV 색상 공간에서)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([500, 30, 255])

    # 노란색 범위 설정
    lower_yellow = np.array([20, 150, 150])
    upper_yellow = np.array([30, 255, 255])

    # 흰색과 노란색 마스크 생성
    mask_white = cv.inRange(hsv, lower_white, upper_white)
    mask_yellow = cv.inRange(hsv, lower_yellow, upper_yellow)

    # 두 마스크를 합쳐서 흰색과 노란색만 남기기
    mask_combined = cv.bitwise_or(mask_white, mask_yellow)

    # 마스크를 사용해 흰색과 노란색 부분만 남기고 나머지는 검게 처리
    result = cv.bitwise_and(img, img, mask=mask_combined)

    # 결과 이미지 출력
    cv.imshow('Filtered Image (White and Yellow)', result)
    cv.waitKey(0)
    cv.destroyAllWindows()
