import cv2 as cv
import numpy as np
import threading, time
import SDcar 
import sys
import time
from tensorflow.keras.models import load_model
import RPi.GPIO as GPIO

classNames = {
    0: 'background', 1: 'person', 2: 'bicycle', 3: 'car', 4: 'motorbike', 5: 'aeroplane',
    6: 'bus', 7: 'train', 8: 'truck', 9: 'boat', 10: 'traffic light', 11: 'fire hydrant',
    12: 'N/A', 13: 'stop sign', 14: 'parking meter', 15: 'bench', 16: 'bird', 17: 'cat',
    18: 'dog', 19: 'horse', 20: 'sheep', 21: 'cow', 22: 'elephant', 23: 'bear',
    24: 'zebra', 25: 'giraffe', 26: 'N/A', 27: 'backpack', 28: 'umbrella', 29: 'N/A',
    30: 'handbag', 31: 'tie', 32: 'suitcase', 33: 'frisbee', 34: 'skis', 35: 'snowboard',
    36: 'sports ball', 37: 'kite', 38: 'baseball bat', 39: 'baseball glove', 40: 'skateboard', 41: 'surfboard',
    42: 'tennis racket', 43: 'bottle', 44: 'wine glass', 45: 'cup', 46: 'fork', 47: 'knife',
    48: 'spoon', 49: 'bowl', 50: 'banana', 51: 'apple', 52: 'sandwich', 53: 'orange',
    54: 'broccoli', 55: 'carrot', 56: 'hot dog', 57: 'pizza', 58: 'donut', 59: 'cake',
    60: 'chair', 61: 'couch', 62: 'potted plant', 63: 'bed', 64: 'dining table', 65: 'toilet',
    66: 'tv', 67: 'laptop', 68: 'mouse', 69: 'remote', 70: 'keyboard', 71: 'cell phone',
    72: 'microwave', 73: 'oven', 74: 'toaster', 75: 'sink', 76: 'refrigerator', 77: 'book',
    78: 'clock', 79: 'vase', 80: 'scissors', 81: 'teddy bear', 82: 'hair drier', 83: 'toothbrush'
}



def id_class_name(class_id, classes) :
    for key, value in classes.items():
        if class_id == key:
            return  value

speed = 80
epsilon = 0.0001

def object_detection_thread():
    global frame, object_detection_enabled, class_name
    
    model = cv.dnn.readNet('/home/pi/hello-git/log-git/week13/frozen_inference_graph.pb',
                                     '/home/pi/hello-git/log-git/week13/ssd_mobilenet_v2_coco_2018_03_29.pbtxt')
    cnt = 0
    num_skip = 10 #지연시간 조절 
    mul_f = 1
    size_img = 200
    # COLORS 배열 정의



    while True:
        cnt += 1
        if object_detection_enabled is True and cnt % num_skip == 0:

            print('cnt', cnt)

            lock.acquire()
            imagednn = frame.copy()
            #lock.release()
            print('frame id', id(frame))
            #print('image dnn id', id(imagednn))
            lock.release()

            image_height, image_width, _ = imagednn.shape

            #print('size img : ',size_img)
            
            starttime = time.time()
            model.setInput(cv.dnn.blobFromImage(imagednn, size=(size_img, size_img), swapRB=True))
            output = model.forward()
            
            class_name = -1
            for detection in output[0, 0, :, :] :
                confidence = detection[2]
                if confidence > .3:
                    class_id = detection[1]
                    class_name = id_class_name(class_id, classNames)
                    print(str(str(class_id) + " " + str(detection[2]) + " " + class_name))

                    box_x = detection[3] * image_width
                    box_y = detection[4] * image_height
                    box_width = detection[5] * image_width
                    box_height = detection[6] * image_height
                    
    
                    cv.rectangle(imagednn, (int(box_x), int(box_y)), (int(box_width), int(box_height)), (23, 230, 5), thickness=2)
                    cv.putText(imagednn, class_name, (int(box_x), int(box_y+.05*image_height)), cv.FONT_HERSHEY_SIMPLEX, 1, (23, 230,5 ), 2)

            elapsed = time.time() - starttime

            cv.imshow('object detection', imagednn)

        if is_running is False:
            break
        if cnt >= 1000000:
            cnt = 0


def key_cmd(which_key):
    #print('which_key', which_key)
    is_exit = False 
    global object_detection_enabled
    global enable_AIdrive
    if which_key & 0xFF == 82:
        print('up')
        car.motor_go(speed)
    elif which_key & 0xFF == 84:
        print('down')
        car.motor_back(speed)
    elif which_key & 0xFF == 81:
        print('left')     
        car.motor_left(30)   
    elif which_key & 0xFF == 83:
        print('right')   
        car.motor_right(30)            
    elif which_key & 0xFF == 32:
        car.motor_stop()
        enable_AIdrive = False     
        print('stop')   
    elif which_key & 0xFF == ord('q'):  
        car.motor_stop()
        print('exit')   
        enable_AIdrive = False     
        is_exit = True    
        print('enable_AIdrive: ', enable_AIdrive)          
    elif which_key & 0xFF == ord('e'):  
        enable_AIdrive = True
        print('enable_AIdrive: ', enable_AIdrive)        
    elif which_key & 0xFF == ord('w'):  
        enable_AIdrive = False
        car.motor_stop()
        print('enable_AIdrive 2: ', enable_AIdrive)   
    elif which_key & 0xFF == ord('t'):  # 't' 키 입력 시 객체 감지 활성화
        object_detection_enabled = True
        print("Object detection enabled")
    elif which_key & 0xFF == ord('r'):  # 'r' 키 입력 시 객체 감지 비활성화
        object_detection_enabled = False
        print("Object detection disabled")
    return is_exit  


# GPIO 핀 설정
LED_PIN1 = 20  # LED가 연결된 핀 번호
LED_PIN2 = 21
LED_PIN3 = 16
LED_PIN4 = 26
BUZZER = 12

# GPIO 초기화
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(LED_PIN1, GPIO.OUT)
GPIO.setup(LED_PIN2, GPIO.OUT)
GPIO.setup(LED_PIN3, GPIO.OUT)
GPIO.setup(LED_PIN4, GPIO.OUT)
GPIO.setup(BUZZER, GPIO.OUT)
p = GPIO.PWM(BUZZER, 261)

def emergency_stop():
    for _ in range(5):  # 5번 점멸
            GPIO.output(LED_PIN1, GPIO.HIGH)
            GPIO.output(LED_PIN2, GPIO.HIGH)
            GPIO.output(LED_PIN3, GPIO.HIGH)
            GPIO.output(LED_PIN4, GPIO.HIGH)
            p.start(50)
            p.ChangeFrequency(261)
            time.sleep(0.5)  # 0.5초 켜기
            GPIO.output(LED_PIN1, GPIO.LOW)
            GPIO.output(LED_PIN2, GPIO.LOW)
            GPIO.output(LED_PIN3, GPIO.LOW)
            GPIO.output(LED_PIN4, GPIO.LOW)
            p.stop()
            time.sleep(0.5)  # 0.5초 끄기
            print("Emergency alert: Activating LED")


def drive_AI(img):
    #print('id', id(model))
    global class_name
    img = np.expand_dims(img, 0)
    res = model.predict(img)[0]
    #print('res', res)
    steering_angle = np.argmax(np.array(res))
    #print('steering_angle', steering_angle)

    print('class name in drive AI', class_name)
    
    if class_name == 'cat' or class_name == 'dog': 
        print("stop fpr detected")
        car.motor_stop()
        emergency_stop()

    elif steering_angle == 0: 
        print("go")
        speedSet = 40
        car.motor_go(speedSet)
    elif steering_angle == 1:
        print("left")
        speedSet = 20
        car.motor_left(speedSet)          
    elif steering_angle == 2:
        print("right")
        speedSet = 20
        car.motor_right(speedSet)
    else:
        print("This cannot be entered")
        

def main():
    global frame
    try:
        while( camera.isOpened() ):
            starttime = time.time()
            lock.acquire()
            ret, frame = camera.read()
            frame = cv.flip(frame,-1)
            lock.release()
            
            cv.imshow('camera',frame)
            # image processing start here
            crop_img = frame[int(v_y/2):,:]
            crop_img = cv.resize(crop_img, (200, 66))
            #show_grid(crop_img)
            #cv.imshow('crop_img ', cv.resize(crop_img, dsize=(0,0), fx=2, fy=2))

            if enable_AIdrive == True:
                starttime = time.time()
                drive_AI(crop_img)
                elapsed = time.time() - starttime

            # image processing end here
            is_exit = False
            which_key = cv.waitKey(20)
            if which_key > 0:
                is_exit = key_cmd(which_key)    
            if is_exit is True:
                cv.destroyAllWindows()
                break

    except Exception as e:
        exception_type, exception_object, exception_traceback = sys.exc_info()
        filename = exception_traceback.tb_frame.f_code.co_filename
        line_number = exception_traceback.tb_lineno

        print("Exception type: ", exception_type)
        print("File name: ", filename)
        print("Line number: ", line_number)
        global is_running
        is_running = False

if __name__ == '__main__':

    #model_path = '/home/pi/hello-git/log-git/week12/lane_navigation_20241202_1207.h5' #교수님 주신 crop_img 학습
    model_path = '/home/pi/hello-git/log-git/week13/lane_navigation_20241207_0900.h5'
    #model_path = '/home/pi/hello-git/log-git/week13/lane_navigation_20241210_0643.h5' #오늘 다시 돌려본거 
    model = load_model(model_path)
    '''print('id', id(model))
    print(model.summary())'''

    model_obj = cv.dnn.readNet('/home/pi/hello-git/log-git/week13/frozen_inference_graph.pb',
                                     '/home/pi/hello-git/log-git/week13/ssd_mobilenet_v2_coco_2018_03_29.pbtxt')

    #test_fun(model)
    W = 320
    H = 240 # 바꿔도됨
    camera = cv.VideoCapture(0)
    camera.set(cv.CAP_PROP_FRAME_WIDTH,W) 
    camera.set(cv.CAP_PROP_FRAME_HEIGHT,H)
    camera.set(cv.CAP_PROP_FPS, 30)

    v_x = W
    v_y = H
    v_x_grid = [int(v_x*i/10) for i in range(1, 10)]
    moment = np.array([0, 0, 0])

    _, frame = camera.read()

    lock = threading.Lock()

    t_task1 = threading.Thread(target = object_detection_thread)
    t_task1.start()
    
    car = SDcar.Drive()
    
    is_running = True
    enable_AIdrive = False
    object_detection_enabled = False
    class_name = -1
    try : 
        main() 
    finally:

        is_running = False
        t_task1.join()

        camera.release()
        cv.destroyAllWindows()

        car.clean_GPIO()
        print('end vis')