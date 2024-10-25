import RPi.GPIO as GPIO
import time

BUZZER = 12

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER, GPIO.OUT)

p = GPIO.PWM(BUZZER, 261)
p.start(50)

try:
    while True:
        p.start(50)
        p.ChangeFrequency(262)
        time.sleep(0.3)

        p.ChangeFrequency(330)
        time.sleep(0.3)

        p.ChangeFrequency(394)
        time.sleep(0.3)

        p.ChangeFrequency(262)
        time.sleep(0.3)

        p.ChangeFrequency(330)
        time.sleep(0.3)

        p.ChangeFrequency(394)
        time.sleep(0.3)

        p.ChangeFrequency(523)
        time.sleep(0.8)

        p.stop()
        time.sleep(1.0)

        

except KeyboardInterrupt:
    pass

p.stop()
GPIO.cleanup()