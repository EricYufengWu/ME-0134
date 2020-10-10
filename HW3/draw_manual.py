from DeltaRobot import DeltaKinematics, DeltaMotion
from adafruit_servokit import ServoKit
import math, time

delta = DeltaKinematics()
bot = DeltaMotion()

while 1:
    coordinates = input("input desired coordinates: ")
    try:
        x0, y0, z0 = coordinates.split()
        angles = delta.inverse_kinematics(float(x0), float(y0), float(z0))
        print(angles)
        bot.move_robot(angles)
    except:
        continue
    time.sleep(0.01)
