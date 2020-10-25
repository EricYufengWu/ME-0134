
# fusiontest.py Simple test program for sensor fusion on Pyboard
# Author Peter Hinch
# Released under the MIT License (MIT)
# Copyright (c) 2017 Peter Hinch
# V0.8 14th May 2017 Option for external switch for cal test. Make platform independent.
# V0.7 25th June 2015 Adapted for new MPU9x50 interface

from imusensor.MPU9250 import MPU9250
import smbus
import time
import sys
from fusion import Fusion

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

# Choose test to run
Calibrate = True
Timing = False
start_time = 0

def timediff(start, end):
    return

def strip_data(data):
    return tuple([data['x'], data['y'], data['z']])

def getmag():                               # Return (x, y, z) tuple (blocking read)
    imu.readSensor()
    return tuple([imu.MagVals[0], imu.MagVals[1], imu.MagVals[2]])

def done():
    # return True if time.time() - start_time > 5 else False
    diff = time.time() - start_time
    # print(diff)
    return True if diff > 15 else False

fuse = Fusion(timediff)

if Calibrate:
    print("Calibrating. Press switch when done.")
    start_time = time.time()
    fuse.calibrate(getmag, done, lambda : time.sleep(0.1))
    print(fuse.magbias)

if Timing:
    imu.readSensor()
    mag = tuple([imu.MagVals[0], imu.MagVals[1], imu.MagVals[2]]) # Don't include blocking read in time
    accel = tuple([imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2]]) # or i2c
    gyro = tuple([imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2]])
    start = time.time()  # Measure computation time only
    fuse.update(accel, gyro, mag, ts = lambda : time.time() / ) # 1.97mS on Pyboard
    t = (time.time() - start)/1000
    print("Update time (mS):", t)

count = 0
while True:
    imu.readSensor()
    fuse.update(tuple([imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2]]), tuple([imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2]]), tuple([imu.MagVals[0], imu.MagVals[1], imu.MagVals[2]]), ts = lambda : time.time()) # Note blocking mag read
    if count % 10 == 0:
        print("Heading, Pitch, Roll: {:7.3f} {:7.3f} {:7.3f}".format(fuse.heading, fuse.pitch, fuse.roll))
    time.sleep(0.02)
    count += 1