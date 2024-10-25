import RPi.GPIO as GPIO
import time

BUZZER = 12
SW1 = 5


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER, GPIO.OUT)
GPIO.setup(SW1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

p = GPIO.PWM(BUZZER, 261)
p.start(50)

try:
    while True:

        sw1Value = GPIO.input(SW1)
        if sw1Value == 1:

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
        

except KeyboardInterrupt:
    pass

p.stop()
GPIO.cleanup()