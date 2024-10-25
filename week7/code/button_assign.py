import RPi.GPIO as GPIO
import time

SW1 = 5
SW2 = 6
SW3 = 13
SW4 = 19

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup(SW1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SW2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SW3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SW4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

switch_names = ['SW1', 'SW2', 'SW3', 'SW4']
switch_pins = [SW1, SW2, SW3, SW4]

prev_values = [0, 0, 0, 0]
click_counts = [0, 0, 0, 0]

try:
    while True:
        for i in range(4):
            current_value = GPIO.input(switch_pins[i])
            
            # 스위치가 눌렸을 때 (0 -> 1으로 변할 때)
            if prev_values[i] == 0 and current_value == 1:
                click_counts[i] += 1  # 클릭 횟수 증가
                print("(", "'", switch_names[i], "click', ", click_counts[i], ")")
            
            # 현재 값을 이전 값에 저장
            prev_values[i] = current_value
        
        time.sleep(0.1)

except KeyboardInterrupt:
    pass

GPIO.cleanup()
