import cv2
import numpy as np
import SDcar

# 전역 변수 설정
moment = np.array([0, 0, 0])
v_x = 320
v_y = 240
v_x_grid = [int(v_x * i / 20) for i in range(1, 20)]  # 10등분된 수평선
epsilon = 1e-5  # Zero division 방지
enable_linetracing = False  # 라인트레이싱 활성화 변수

# 속도 정의 (car.motor_go, motor_left 등에서 사용)
speed = 30  # 적당한 속도 값을 설정

def detect_white_line(frame):
    """
    흰색 선을 탐지하는 함수. HSV 색 공간에서 흰색 범위를 설정하여 마스크 생성.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 흰색 HSV 범위 설정
    lower_white = np.array([0, 0, 200])  # H, S, V의 최소값
    upper_white = np.array([180, 50, 255])  # H, S, V의 최대값

    # 흰색 영역 마스크 생성
    mask = cv2.inRange(hsv, lower_white, upper_white)
    return mask

# 그리드 표시 함수
def show_grid(img):
    h, _, _ = img.shape
    for x in v_x_grid:
        cv2.line(img, (x, 0), (x, h), (0, 255, 0), 1, cv2.LINE_4)  # 초록색 그리드

# 노란색 선 검출 및 모멘트 계산 함수
def detect_white_and_calculate_moment():
    print("라인트레이싱 동작 시작")
    global enable_linetracing  # 전역 변수 참조
    print(f"라인트레이싱 시작 전 enable_linetracing 상태: {enable_linetracing}")
    if enable_linetracing:
        print(f"라인트레이싱 활성화됨, enable_linetracing 상태: {enable_linetracing}")
    else:
        print(f"라인트레이싱 비활성화됨, enable_linetracing 상태: {enable_linetracing}")

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("카메라 프레임을 읽을 수 없습니다.")
                break

            frame = cv2.flip(frame, 0)
            frame = frame[180:,100 :-100]  

            # 그리드 표시
            show_grid(frame)

            # 흰색 영역 추출
            mask = detect_white_line(frame)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                m = cv2.moments(c)

                # 모멘트 계산
                cx = int(m['m10'] / (m['m00'] + epsilon))  # cx 계산
                cy = int(m['m01'] / (m['m00'] + epsilon))  # cy 계산

                # 빨간색 모멘트 위치 표시
                cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)  # 빨간색 점

                # 초록색 컨투어 표시
                cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)  # 초록색 윤곽선 그리기

                # X좌표 출력
                cv2.putText(frame, str(cx), (10, 10), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0))

                if enable_linetracing:
                    line_tracing(cx)  # 라인트레이싱 수행

            cv2.imshow('Result', frame)  # 화면에 결과 표시

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()

# 라인트레이싱 수행 함수
def line_tracing(cx):
    global moment
    global v_x
    tolerance = 0.1
    diff = 0

    if moment[0] != 0 and moment[1] != 0 and moment[2] != 0:
        avg_m = np.mean(moment)
        diff = np.abs(avg_m - cx) / v_x

    if diff <= tolerance:
        moment[0] = moment[1]
        moment[1] = moment[2]
        moment[2] = cx
  
        # 라인트레이싱 조건
        if v_x_grid[4] <= cx <= v_x_grid[5]:
            car.motor_go(speed)  
            print("go")
        elif v_x_grid[0] <= cx <= v_x_grid[3]:
            car.motor_right(speed)  
            print('turn right')
        elif v_x_grid[6] <= cx <= v_x_grid[9]:
            car.motor_left(speed) 
            print('turn left')

    else:
        # 라인이 중앙에서 벗어나면 직진
        car.motor_go(speed)
        print('go')
        moment = [0, 0, 0]  # moment 초기화



if __name__ == '__main__':
    car = SDcar.Drive()  # 자동차 객체 생성
    #car.motor_go(50)
    print("SDcar 인스턴스 생성 완료")
    enable_linetracing=True
    detect_white_and_calculate_moment()  # 라인트레이싱 시작

    
    

    
