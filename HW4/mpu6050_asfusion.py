  
# fusiontest_as6.py Test for 6DOF asynchronous sensor fusion on Pyboard.
# Author: Peter Hinch
# Released under the MIT License (MIT) See LICENSE
# Copyright (c) 2017-2020 Peter Hinch

# Requires:
# uasyncio V3 (Included in daily builds and release builds later than V1.12).
# From https://github.com/micropython-IMU/micropython-mpu9x50:
# imu.py, mpu6050.py, vector3d.py
# From this repo: deltat.py fusion_async.py
# MPU9150 on X position

import asyncio
import board
import gc
import busio
import adafruit_mpu6050
from fusion_async import Fusion # Using async version

i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)

# User coro returns data and determines update rate.
# For 6DOF sensors two 3-tuples (x, y, z) for accel and gyro
async def read_coro():
    await asyncio.sleep(0.02)  # Plenty of time for mag to be ready
    return mpu.acceleration, mpu.gyro

fuse = Fusion(read_coro, timediff = lambda start, end: start()-end())

async def mem_manage():         # Necessary for long term stability
    while True:
        await asyncio.sleep(0.1)
        gc.collect()
        gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())

async def display():
    fs = 'Heading: {:4.0f} Pitch: {:4.0f} Roll: {:4.0f}'
    while True:
        print(fs.format(fuse.heading, fuse.pitch, fuse.roll))
        await asyncio.sleep(0.5)

async def test_task():
    await fuse.start()  # Start the update task
    loop = asyncio.get_event_loop()
    await display()

asyncio.create_task(mem_manage())
asyncio.run(test_task())