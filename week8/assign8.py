import threading
import serial
import time
import RPi.GPIO as GPIO

bleSerial = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1.0)

PWMA = 18
PWMB = 23
AIN1 = 22
AIN2 = 27
BIN1 = 25
BIN2 = 24

gData = ""

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(PWMB, GPIO.OUT)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)

L_Motor = GPIO.PWM(PWMA, 500)
L_Motor.start(0)
R_Motor = GPIO.PWM(PWMB, 500)
R_Motor.start(0)

def move_forward():
    print("go")
    GPIO.output(AIN1, 0)
    GPIO.output(AIN2, 1)
    L_Motor.ChangeDutyCycle(100)
    GPIO.output(BIN1, 0)
    GPIO.output(BIN2, 1)
    R_Motor.ChangeDutyCycle(100)

def move_backward():
    print("back")
    GPIO.output(AIN1, 1)
    GPIO.output(AIN2, 0)
    L_Motor.ChangeDutyCycle(100)
    GPIO.output(BIN1, 1)
    GPIO.output(BIN2, 0)
    R_Motor.ChangeDutyCycle(100)

def move_left():
    print("left")
    GPIO.output(AIN1, 1)
    GPIO.output(AIN2, 0)
    L_Motor.ChangeDutyCycle(100) 
    GPIO.output(BIN1, 0)
    GPIO.output(BIN2, 1)
    R_Motor.ChangeDutyCycle(100)
    time.sleep(1)

def move_right():
    print("right")
    GPIO.output(AIN1, 0)
    GPIO.output(AIN2, 1)
    L_Motor.ChangeDutyCycle(100)
    GPIO.output(BIN1, 1)
    GPIO.output(BIN2, 0)
    R_Motor.ChangeDutyCycle(100)  
    time.sleep(1)

def stop():
    print("stop")
    L_Motor.ChangeDutyCycle(0)
    R_Motor.ChangeDutyCycle(0)

def serial_thread():
    global gData
    while True:
        data = bleSerial.readline()
        data = data.decode().strip()  
        gData = data

def main():
    global gData
    try:
        while True:
            if gData == "B2":
                gData = ""
                move_forward()
            elif gData == "B4":
                gData = ""
                move_backward()
            elif gData == "B1":
                gData = ""
                move_left()
            elif gData == "B3":
                gData = ""
                move_right()
            elif gData == "B0":
                gData = ""
                stop()
            time.sleep(0.1) 

    except KeyboardInterrupt:
        pass
    finally:
        stop() 
        GPIO.cleanup()
        bleSerial.close()

if __name__ == '__main__':
    task1 = threading.Thread(target=serial_thread)
    task1.start()
    main()
