import os
import sys
import time
import smbus
import odrive

from imusensor.MPU9250 import MPU9250
from imusensor.filters import madgwick

sensorfusion = madgwick.Madgwick(0.5)

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

# imu.caliberateGyro()
# imu.caliberateAccelerometer()
# or load your own caliberation file
#imu.loadCalibDataFromFile("/home/pi/calib_real4.json")


odrv0 = odrive.find_any()
my_motor = odrv0.axis0

# print bus voltage
print("vbus voltage: ", odrv0.vbus_voltage)

# run motor calibration sequence
my_motor.requested_state = 3
time.sleep(15)

# setup close_loop control
my_motor.requested_state = 8 # AXIS_STATE_CLOSED_LOOP_CONTROL

# run motor at velocity mode (incremental speed)
my_motor.controller.config.control_mode = 3 # position control

# first slowly move the motor back to zero pos
my_motor.controller.config.vel_limit = 1
my_motor.controller.input_pos = 0
time.sleep(5)
my_motor.controller.config.vel_limit = 3
my_motor.controller.config.control_mode = 1 # velocity control

target = 72
currTime = time.time()
startTime = currTime
print_count = 0
my_motor.requested_state = 1
while True:
	imu.readSensor()
	for i in range(10):
		newTime = time.time()
		dt = newTime - currTime
		currTime = newTime

		sensorfusion.updateRollPitchYaw(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2], imu.GyroVals[0], \
									imu.GyroVals[1], imu.GyroVals[2], imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], dt)

	if print_count == 2:
		# print ("mad roll: {0} ; mad pitch : {1} ; mad yaw : {2}".format(sensorfusion.roll, sensorfusion.pitch, sensorfusion.yaw))
		theta = sensorfusion.yaw
		print_count = 0
		if time.time() - startTime > 10:
			print(theta)
			# error = theta - target
			# vel = 0.01 * error
			# print(vel)
			# my_motor.requested_state = 8
			# # my_motor.controller.input_vel = vel
			# my_motor.requested_state = 1


	print_count = print_count + 1
	time.sleep(0.01)