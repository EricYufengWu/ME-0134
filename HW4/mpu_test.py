import FaBo9Axis_MPU9250
import time
import sys

mpu9250 = FaBo9Axis_MPU9250.MPU9250()


while 1:
	accel = mpu9250.readAccel()
	# print(accel['x'], accel['y'], accel['z'],)
	gyro = mpu9250.readGyro()
	print(gyro['x'], gyro['y'], gyro['z'],)
	mag = mpu9250.readMagnet()
	time.sleep(0.1)