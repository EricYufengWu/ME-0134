from DeltaRobot import DeltaKinematics
import math, time

delta = DeltaKinematics()

while 1:
    coordinates = input("input desired coordinates: ")
    x0, y0, z0 = coordinates.split()
    print(delta.inverse_kinematics(float(x0), float(y0), float(z0)))
    time.sleep(0.01)

