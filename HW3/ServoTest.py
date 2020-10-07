import time
from adafruit_servokit import ServoKit
servo_range = 135
kit = ServoKit(channels=16)
kit.servo[0].actuation_range = servo_range
kit.servo[1].actuation_range = servo_range
kit.servo[2].actuation_range = servo_range


while 1:
    i = 0
    while i <= (int(servo_range)/2):
        print(i)
        kit.servo[0].angle = i
        kit.servo[1].angle = i
        kit.servo[2].angle = i
        i += 0.5
        time.sleep(0.2)
    i = int(servo_range / 2)
    while i >= 0:
        print(i)
        kit.servo[0].angle = i
        kit.servo[1].angle = i
        kit.servo[2].angle = i
        i -= 0.5
        time.sleep(0.2)

