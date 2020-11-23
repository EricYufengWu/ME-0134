from adafruit_servokit import ServoKit
import time

kit = ServoKit(channels=16)

def set_range():
    kit.servo[0].set_pulse_width_range(500, 2600)
    kit.servo[1].set_pulse_width_range(400, 2500)
    kit.servo[2].set_pulse_width_range(450, 2550)
    kit.servo[4].set_pulse_width_range(450, 2550)
    kit.servo[5].set_pulse_width_range(380, 2480)
    kit.servo[6].set_pulse_width_range(450, 2550)
    kit.servo[8].set_pulse_width_range(450, 2550)
    kit.servo[9].set_pulse_width_range(320, 2520)
    kit.servo[10].set_pulse_width_range(400, 2500)
    kit.servo[12].set_pulse_width_range(350, 2450)
    kit.servo[13].set_pulse_width_range(400, 2500)
    kit.servo[14].set_pulse_width_range(430, 2530)

def home():
    for i in range(16):
        kit.servo[i].angle = 90

set_range()

while 1:
    home()
    time.sleep(0.1)
