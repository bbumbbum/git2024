# 예시: 모델 예측 테스트
import numpy as np
import cv2
from tensorflow.keras.models import load_model

# 이미지 로드
image = cv2.imread('/home/pi/hello-git/log-git/week12/crop_img/crop_train_00070_135.png')
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # 모델 입력에 맞게 색상 변환

# 이미지 리사이즈 (모델 입력 크기에 맞게)
image = cv2.resize(image, (200, 66))  # 모델 훈련 시 사용한 크기

# 배치 차원 추가
image = np.expand_dims(image, axis=0)


# 저장된 모델 경로
model_path = '/home/pi/hello-git/log-git/week12/lane_navigation_20241202_1207.h5'

# 모델 로드
model = load_model(model_path)

# 예측
prediction = model.predict(image)
print("Prediction:", prediction)
