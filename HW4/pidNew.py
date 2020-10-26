import math
import odrive
import time
import serial
import os
import sys
# import FaBo9Axis_MPU9250

g = 9.8
odrv0 = odrive.find_any()
my_motor = odrv0.axis0
# mpu9250 = FaBo9Axis_MPU9250.MPU9250()


s = serial.Serial('/dev/ttyACM0', baudrate = 9600, timeout = 0)
s.readline()
s.reset_input_buffer()
# print bus voltage
print("vbus voltage: ", odrv0.vbus_voltage)
my_motor.requested_state = 3   # run motor calibration sequence
time.sleep(15)
# setup close_loop control
my_motor.requested_state = 8 # AXIS_STATE_CLOSED_LOOP_CONTROL
# run motor at velocity mode (incremental speed)
# my_motor.controller.config.control_mode = 3 # position control
my_motor.controller.config.control_mode = 1 # torque control
# first slowly move the motor back to zero pos
my_motor.controller.config.vel_limit = 1
my_motor.controller.input_pos = 0
time.sleep(5)
my_motor.controller.config.vel_limit = 15

# my_motor.requested_state = 1

class Arm:
	def __init__(self):
		self.armtorque = 0 # This is the odrive initial torque
		self.armtheta = 0 # This is the odrive initial angle
		self._l = 0.136
		self._m = 0.008

class Pendulum:
	def __init__(self, penduphi):
		self.pendulumPhi = penduphi # This is the initial angle of the imu
		self._l = 0.160
		self._m = 0.011

def get_angle():
	global s
	val = s.readline().decode()
	try:
		theta = float(val)
		if theta >= 30 or theta <= -30:
			my_motor.requested_state = 1
		print(theta)
		return theta
	except:
		return

def basic_calculate(arm, pendulum):
	mr = arm._m
	lr = arm._l
	mp = pendulum._m
	lp = pendulum._l
	Jr = (1/3) * (mr * (lr**2))
	Jp = (1/12) * (mp * (lp**2))
	c1 = Jr +  mp * (lr**2)
	c2 = - (1/2) * mp * lr * lp
	c3 = Jp + 1/4 * mp * (lp**2)
	c4 = -(1/2) * mp * lr * lp
	c5 = -(1/2) * mp * g * lp
	return (c1,c2,c3,c4,c5)

def pid_control(arm, time_delta, error, previous_error, integral):
	# Using PID to find control inputs

	# The gains were emperically tuned
	Kp = -150
	Kd = -20
	Ki = -20

	derivative = (error - previous_error) / time_delta
	integral += error * time_delta
	arm.armtorque = (Kp * error) + (Kd * derivative) + (Ki * integral)
	return integral

def find_error(pendulum):
	# There's a seperate function for this because of the wrap-around problem
	# This function returns the error
	# print(type(pendulum.pendulumPhi))
	previous_error = (pendulum.pendulumPhi % (360)) - 0
	if previous_error > 180:
		previous_error = previous_error - (360)
	return previous_error

def position_calculate(c1, c2, c3, c4, c5, arm, pendulum, Theta_tminus2, time_delta, previous_time_delta):
	# gyro = mpu9250.readGyro()
	theta = get_angle()

	# pendulum.pendulumPhi = gyro['x']
	if theta != None:
		pendulum.pendulumPhi = theta
		Theta_double_dot = ((c2*c5)/(c1*c3-c2*c4)) * pendulum.pendulumPhi + (c3/(c1*c3-c2*c4)) * arm.armtorque
		arm.armtheta += (((time_delta**2) * Theta_double_dot) + (((arm.armtheta  - Theta_tminus2) * time_delta) / previous_time_delta)) / 120 # convert from degree

if __name__ == '__main__':
	# Initialize the class
	arm = Arm()

	gyro = None
	while gyro == None:
		gyro=get_angle()
	print(type(gyro))
	pendulum = Pendulum(gyro)
	# print(gyro)
	time.sleep(2)
	
	# pendulum = Pendulum(gyro['x'])
	# print(gyro['x']) 

	# Initialize the other variables
	Theta_tminus1 = Theta_tminus2 = arm.armtheta
	Phi_tminus1   = Phi_tminus2   = pendulum.pendulumPhi

	#previous_error = find_error(pendulum)
	integral = 0
	previous_time_delta = 0
	simulation_time = 30
	previous_timestamp = time.time()

	previous_error = find_error(pendulum)
	integral = 0
	previous_time_delta = 0

	# Set end_time as 30 seconds
	end_time = previous_timestamp + 30

	c1,c2,c3,c4,c5 = basic_calculate(arm, pendulum)

	# Main loop
	while time.time() <= end_time:
		current_timestamp = time.time()
		time_delta = (current_timestamp - previous_timestamp)
		error = find_error(pendulum)
		if previous_time_delta != 0:    
			Theta_dot = (Theta_tminus1 - Theta_tminus2 ) / previous_time_delta
			Phi_dot = (Phi_tminus1 - Phi_tminus2) / previous_time_delta
			intergral = pid_control(arm, time_delta, error, previous_error, integral)
			position_calculate(c1, c2, c3, c4, c5, arm, pendulum, Theta_tminus2, time_delta, previous_time_delta)

		# Debug
		print(arm.armtheta, arm.armtorque)

		# Update the position and the torque of odrive
		# my_motor.controller.input_pos = 3 * arm.armtheta
		# my_motor.controller.input_torque = arm.armtorque

		# Update the variables
		previous_time_delta = time_delta
		previous_timestamp = current_timestamp
		previous_error = error
		Theta_tminus2 = Theta_tminus1
		Theta_tminus1 = arm.armtheta
		Phi_tminus2 = Phi_tminus1
		Phi_tminus1 = pendulum.pendulumPhi
		time.sleep(0.1)