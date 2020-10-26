import os
import sys
import time
import odrive

import serial
from simple_pid import PID

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
my_motor.controller.config.vel_limit = 12
my_motor.controller.config.control_mode = 2 # velocity control

kp = 0.35
ki = 0.001
kd = 0.0
pid = PID(kp, ki, kd, 0)
pid.sample_time = 0.01

s = serial.Serial('/dev/ttyACM1', baudrate = 9600, timeout = 0)
s.readline()
s.reset_input_buffer()


# target = 0
# start = time.time()
# theta_prev = 0
# theta_sum = 0

while 1:
	try:
		val = s.readline().decode()
		try:
			theta = float(val)
			if theta >= 30 or theta <= -30:
				my_motor.requested_state = 1
				break
			# print(theta)
		except:
			continue

		output = pid(theta)
		print(output)

		my_motor.controller.input_vel = output

		# time.sleep(0.01)
	except KeyboardInterrupt:
		my_motor.controller.input_vel = 0
		my_motor.requested_state = 1
		break
