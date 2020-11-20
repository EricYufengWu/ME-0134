from adafruit_servokit import ServoKit
import time

kit = ServoKit(channels=16)

def set_range():
    kit.servo[0].set_pulse_width_range(400, 2500)
    kit.servo[1].set_pulse_width_range(400, 2500)
    kit.servo[2].set_pulse_width_range(400, 2500)
    kit.servo[3].set_pulse_width_range(400, 2500)
    kit.servo[4].set_pulse_width_range(400, 2500)
    kit.servo[5].set_pulse_width_range(400, 2500)
    kit.servo[6].set_pulse_width_range(400, 2500)
    kit.servo[7].set_pulse_width_range(400, 2500)
    kit.servo[8].set_pulse_width_range(400, 2500)
    kit.servo[9].set_pulse_width_range(400, 2500)
    kit.servo[10].set_pulse_width_range(400, 2500)
    kit.servo[11].set_pulse_width_range(400, 2500)

def home():
    for i in range(12):
        kit.servo[i].angle = 90

set_range()

while 1:
    home()
    time.sleep(0.1)
