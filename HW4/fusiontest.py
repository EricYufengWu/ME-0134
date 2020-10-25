
# fusiontest.py Simple test program for sensor fusion on Pyboard
# Author Peter Hinch
# Released under the MIT License (MIT)
# Copyright (c) 2017 Peter Hinch
# V0.8 14th May 2017 Option for external switch for cal test. Make platform independent.
# V0.7 25th June 2015 Adapted for new MPU9x50 interface

import FaBo9Axis_MPU9250
import time
import sys
from fusion import Fusion

# Choose test to run
Calibrate = True
Timing = True
start_time = 0


def timediff(start, end):
    return

def strip_data(data):
    return tuple([data['x'], data['y'], data['z']])

def getmag():                               # Return (x, y, z) tuple (blocking read)
    return strip_data(imu.readMagnet())

def done():
    # return True if time.time() - start_time > 5 else False
    diff = time.time() - start_time
    # print(diff)
    return True if diff > 15 else False

imu = FaBo9Axis_MPU9250.MPU9250()

fuse = Fusion(timediff)

if Calibrate:
    print("Calibrating. Press switch when done.")
    start_time = time.time()
    fuse.calibrate(getmag, done, lambda : time.sleep(0.1))
    print(fuse.magbias)

if Timing:
    mag = strip_data(imu.readMagnet()) # Don't include blocking read in time
    accel = strip_data(imu.readAccel()) # or i2c
    gyro = strip_data(imu.readGyro())
    start = time.time()  # Measure computation time only
    fuse.update(accel, gyro, mag, ts = lambda : time.time()) # 1.97mS on Pyboard
    t = (time.time() - start)/1000
    print("Update time (mS):", t)

count = 0
while True:
    fuse.update(strip_data(imu.readAccel()), strip_data(imu.readGyro()), strip_data(imu.readMagnet()), ts = lambda : time.time()) # Note blocking mag read
    if count % 50 == 0:
        print("Heading, Pitch, Roll: {:7.3f} {:7.3f} {:7.3f}".format(fuse.heading, fuse.pitch, fuse.roll))
    time.sleep(0.02)
    count += 1