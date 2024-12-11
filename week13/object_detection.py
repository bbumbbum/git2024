import cv2 as cv
import numpy as np

# 클래스 이름을 텍스트 파일에서 불러오기
class_names = []
with open('/home/pi/hello-git/log-git/week13/object_detection_classes_coco.txt', 'r') as f:
    class_names = f.read().splitlines()

# 각 클래스에 대해 고유한 색상 생성
COLORS = np.random.uniform(0, 255, size=(len(class_names), 3))

# 모델 파일과 설정 파일 경로
model = cv.dnn.readNetFromTensorflow(model='/home/pi/hello-git/log-git/week13/frozen_inference_graph.pb',
                                     config='/home/pi/hello-git/log-git/week13/ssd_mobilenet_v2_coco_2018_03_29.pbtxt')

# 실시간 비디오 스트림 캡처 (웹캠 사용, 다른 소스는 번호를 변경)
cap = cv.VideoCapture(0)  # 0은 기본 웹캠, 다른 번호로 다른 카메라나 파일을 지정 가능
frame_skip = 2  # 2프레임마다 객체 검출 수행 (프레임 건너뛰기)

while True:
    # 비디오에서 프레임 읽기
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break

    frame_skip -= 1
    if frame_skip > 0:
        continue
    frame_skip = 2  # 다음 프레임에서는 객체 검출 수행
    
    frame = cv.resize(frame, (320, 240)) 
    frame = cv.flip(frame, 0)

    # 이미지 크기 가져오기
    image_height, image_width, _ = frame.shape

    # 이미지를 Blob 형태로 변환
    blob = cv.dnn.blobFromImage(frame, size=(300, 300), swapRB=True)

    # 모델에 Blob 입력
    model.setInput(blob)

    # 모델을 통과하여 객체 검출
    output = model.forward()  
    print('output shape', output.shape)

    # 검출된 객체들을 순차적으로 처리
    for detection in output[0, 0, :, :]:
        # 검출된 객체의 신뢰도(Confidence) 추출
        confidence = detection[2]

        # 신뢰도가 일정 기준 이상인 객체만 처리
        if confidence > 0.4:
            # 클래스 ID 추출
            class_id = detection[1]
            # 클래스 ID를 클래스 이름으로 변환
            class_name = class_names[int(class_id) - 1]
            # 클래스 ID에 맞는 색상 선택
            color = COLORS[int(class_id)]

            # 객체의 위치 추출 (정규화된 값)
            box_x = detection[3] * image_width
            box_y = detection[4] * image_height
            box_width = detection[5] * image_width
            box_height = detection[6] * image_height

            # 경계 상자 그리기
            cv.rectangle(frame, (int(box_x), int(box_y)), (int(box_width), int(box_height)), color, thickness=2)

            # 객체 이름 텍스트 출력
            cv.putText(frame, class_name, (int(box_x), int(box_y - 5)), cv.FONT_HERSHEY_SIMPLEX, 1, color, 2)

    # 실시간 영상 출력
    cv.imshow('Object Detection - Real-time', frame)

    # 'q' 키를 누르면 종료
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# 비디오 캡처 객체 해제 및 창 닫기
cap.release()
cv.destroyAllWindows()
