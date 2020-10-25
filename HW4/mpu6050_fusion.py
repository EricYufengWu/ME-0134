
# fusiontest.py Simple test program for sensor fusion on Pyboard
# Author Peter Hinch
# Released under the MIT License (MIT)
# Copyright (c) 2017 Peter Hinch
# V0.8 14th May 2017 Option for external switch for cal test. Make platform independent.
# V0.7 25th June 2015 Adapted for new MPU9x50 interface

import board
import busio
import adafruit_mpu6050
import time
import sys
from fusion import Fusion

i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)

fuse = Fusion(lambda start, end: start()-end())

# Choose test to run
Timing = True

if Timing:
    accel = mpu.acceleration # or i2c
    gyro = mpu.gyro
    start = time.time()  # Measure computation time only
    fuse.update_nomag(accel, gyro, ts = lambda : time.time()) # 1.97mS on Pyboard
    t = (time.time() - start)
    print("Update time (S):", t)

count = 0
while True:
    gyro = mpu.gyro
    acc = mpu.acceleration
    fuse.update_nomag(acc, gyro, ts = lambda : time.time()) # Note blocking mag read
    if count % 1 == 0:
        print("Heading, Pitch, Roll: {:7.3f} {:7.3f} {:7.3f}".format(gyro[0], gyro[1], gyro[2]))
        print("Heading, Pitch, Roll: {:7.3f} {:7.3f} {:7.3f}".format(fuse.heading, fuse.pitch, fuse.roll))
    time.sleep(0.02)
    count += 1