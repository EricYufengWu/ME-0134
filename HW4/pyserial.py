import serial
import time
from simple_pid import PID

kp = 0.01
ki = 0
kd = 0
pid = PID(kp, ki, kd, 0)
pid.sample_time = 0.01

s = serial.Serial('/dev/ttyACM0', baudrate = 9600)
s.readline()
s.reset_input_buffer()


# target = 0
# start = time.time()
# theta_prev = 0
# theta_sum = 0

while 1:
	try:	
		val = s.readline().decode()
		print(val)
		# try:
		# 	theta = float(val)
		# 	# print(theta)
		# except:
		# 	continue
		# print(theta)
	except KeyboardInterrupt:
		s.close()

	# end = time.time()
	# dt = end - start
	# start = end

	# rate = (theta - theta_prev) / dt
	# error = theta - target
	# output = kp*error - kd*rate + ki*theta_sum
	# print(output, dt)

	# theta_prev = theta
	# theta_sum += theta

