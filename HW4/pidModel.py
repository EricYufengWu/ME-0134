import math
import odrive
import time
import control
import FaBo9Axis_MPU9250

odrv0 = odrive.find_any()
my_motor = odrv0.axis0
mpu9250 = FaBo9Axis_MPU9250.MPU9250()
gyro = mpu9250.readGyro()

# print bus voltage
print("vbus voltage: ", odrv0.vbus_voltage)

class Arm:
    def __init__(self):
        self.armtheta = 0 # This is the odrive start angle

class Pendulum:
    def __init__(self, penduphi):
        self.pendulumPhi = penduphi # This is the initial angle of the imu

# 暂时不需要
# def find_pid_control_input(time_delta,error,previous_error,integral):
# 	# Using PID to find control inputs

# 	# The gains were emperically tuned
# 	Kp = -150
# 	Kd = -20
# 	Ki = -20

# 	derivative = (error - previous_error) / time_delta
# 	integral += error * time_delta
# 	F = (Kp * error) + (Kd * derivative) + (Ki * integral)
# 	return F, integral

# 暂时不需要
# def find_error(pendulum):
# 	# There's a seperate function for this because of the wrap-around problem
# 	# This function returns the error
# 	previous_error = (pendulum.pendulumPhi % (2 * math.pi)) - 0
# 	if previous_error > math.pi:
# 		previous_error = previous_error - (2 * math.pi)
# 	return previous_error

def position_calculate(arm, pendulum, time_delta, Theta_dot, Theta_tminus2, Phi_dot, Phi_tminus2, previous_time_delta):

    Theta_double_dot = -0.807*arm.armtheta - 0.9415*Theta_dot + 133.8*pendulum.pendulumPhi - 20.44*Phi_dot
    Phi_double_dot = -1.029*arm.armtheta - 1.2*Theta_dot - 201.2*pendulum.pendulumPhi - 26.06*Phi_dot
    arm.armtheta += ((time_delta**2) * Theta_double_dot) + (((arm.armtheta  - Theta_tminus2) * time_delta) / previous_time_delta)
    pendulum.pendulumPhi += ((time_delta**2)*theta_double_dot) + (((pendulum.pendulumPhi - Phi_tminus2)*time_delta)/previous_time_delta)

if __name__ == '__main__':
    # Initialize the class
    arm = Arm()
    pendulum = Pendulum(gyro['x']) 

    # Initialize the other variables
    Theta_dot = 0
	Theta_tminus1 = Theta_tminus2 = arm.armtheta
	Phi_tminus1 = Phi_tminus2 = pendulum.pendulumPhi
	previous_error = find_error(pendulum)
	integral = 0
	previous_time_delta = 0
    simulation_time = 30
    previous_timestamp = time.time()

    # Main loop
    # Set end_time as 30 seconds
    end_time = previous_timestamp + 30
    while time.time() <= end_time:
        current_timestamp = time.time()
		time_delta = (current_timestamp - previous_timestamp)
		error = find_error(pendulum)
        if previous_time_delta != 0:	
			Theta_dot = (Theta_tminus1 - Theta_tminus2 ) / previous_time_delta
			x_dot = (Phi_tminus1 - Phi_tminus2) / previous_time_delta
			# F, intergral = find_pid_control_input(time_delta,error,previous_error,integral)
			position_calculate(arm, pendulum, time_delta, Theta_dot, Theta_tminus2, Phi_dot, Phi_tminus2, previous_time_delta)
	
		# Update the variables
		previous_time_delta = time_delta
		previous_timestamp = current_timestamp
		# previous_error = error
		Theta_tminus2 = Theta_tminus1
		Theta_tminus1 = arm.armtheta
		Phi_tminus2 = Phi_tminus1
		Phi_tminus1 = pendulum.pendulumPhi 