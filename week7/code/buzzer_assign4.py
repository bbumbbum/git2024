import RPi.GPIO as GPIO
import time

BUZZER = 12
SW1 = 5
SW2 = 6
SW3 = 13
SW4 = 19

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER, GPIO.OUT)
GPIO.setup(SW1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SW2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SW3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SW4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

p = GPIO.PWM(BUZZER, 261)

try:
    while True:
        sw1Value = GPIO.input(SW1)
        sw2Value = GPIO.input(SW2)
        sw3Value = GPIO.input(SW3)
        sw4Value = GPIO.input(SW4)

        if sw1Value == 1:
            p.start(50)
            p.ChangeFrequency(292)
            time.sleep(0.3)

        elif sw2Value == 1:
            p.start(50)
            p.ChangeFrequency(330)
            time.sleep(0.3)

        elif sw3Value == 1:
            p.start(50)
            p.ChangeFrequency(394)
            time.sleep(0.3)

        elif sw4Value == 1:
            p.start(50)
            p.ChangeFrequency(440)
            time.sleep(0.3)

        else:
            p.stop()

except KeyboardInterrupt:
    pass

p.stop()
GPIO.cleanup()
