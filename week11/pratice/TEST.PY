import RPi.GPIO as GPIO
import time

pins = {"PWMA": 18, "AIN1": 22, "AIN2": 27}

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(pins["PWMA"], GPIO.OUT)
GPIO.setup(pins["AIN1"], GPIO.OUT)
GPIO.setup(pins["AIN2"], GPIO.OUT)

try:
    motor = GPIO.PWM(pins["PWMA"], 500)  # PWM 핀 초기화
    motor.start(0)

    GPIO.output(pins["AIN1"], 0)  # 정방향
    GPIO.output(pins["AIN2"], 1)
    motor.ChangeDutyCycle(50)  # 50% 출력
    time.sleep(2)

    GPIO.output(pins["AIN1"], 1)  # 역방향
    GPIO.output(pins["AIN2"], 0)
    time.sleep(2)

    motor.ChangeDutyCycle(0)  # 모터 정지
finally:
    GPIO.cleanup()
