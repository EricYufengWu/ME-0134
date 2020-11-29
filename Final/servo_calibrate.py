from adafruit_servokit import ServoKit
import time

kit_1 = ServoKit(channels=16)
kit_2 = ServoKit(channels=16, address=0x41)

def set_range():
    kit_1.servo[0].set_pulse_width_range(500, 2600)
    kit_1.servo[1].set_pulse_width_range(400, 2500)
    kit_1.servo[2].set_pulse_width_range(450, 2550)
    kit_1.servo[4].set_pulse_width_range(450, 2550)
    kit_1.servo[5].set_pulse_width_range(380, 2480)
    kit_1.servo[6].set_pulse_width_range(450, 2550)
    kit_1.servo[8].set_pulse_width_range(450, 2550)
    kit_1.servo[9].set_pulse_width_range(320, 2520)
    kit_1.servo[10].set_pulse_width_range(400, 2500)
    kit_1.servo[12].set_pulse_width_range(350, 2450)
    kit_1.servo[13].set_pulse_width_range(400, 2500)
    kit_1.servo[14].set_pulse_width_range(430, 2530)
    kit_2.servo[0].set_pulse_width_range(300, 2400)
    kit_2.servo[1].set_pulse_width_range(350, 2450)
    kit_2.servo[2].set_pulse_width_range(400, 2500)
    kit_2.servo[12].set_pulse_width_range(500, 2600)
    kit_2.servo[13].set_pulse_width_range(400, 2500)
    kit_2.servo[14].set_pulse_width_range(500, 2600)


def home():
    for i in range(16):
        kit_1.servo[i].angle = 90
        kit_2.servo[i].angle = 90

set_range()

while 1:
    home()
    time.sleep(0.1)
