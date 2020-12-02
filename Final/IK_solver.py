import math
import numpy as np 


'''
Base Coordinate System:
	Z
	^
	|
	Y ---> X  
Y into the page, Z is the direction of movement

'''


class hex_IK:
	def __init__(self, width = 41.9, length = 22, coxia = 19.1, femur = 120.6, tibia = 202):
		self.W = width   # Half the distance between the left and right base servo axis in X direction
		self.L = length  # Half the distance between the front and back servos in the Y direction
		self.C = coxia   # Coxia - Rotate
		self.F = femur   # Femur - Knee
		self.T = tibia   # Tibia - Ankle

	def calculate_angle_yz(self, x0, y0, z0):
		return

	# Transfer coordinates from base coordinates to individual leg coordinate system
	def trans_coord(self, x,y,z,leg="FR"):
		# Does stuff here
		return x,y,z

	def inverse_k(self, x, y, z):
		x0, y0, z0 = self.trans_coord(x,y,z)

		try:
			theta_c = math.atan(z0/x0)
			r = x0 / math.cos(theta_c)
			theta_f = math.pi/2 - math.atan((r-self.C)/y0) - math.acos((pow(self.F,2) + y0*y0 + pow((r - self.C),2) - pow(self.T,2)) / (2 * self.F * math.sqrt(y0*y0 + pow((r - self.C),2))))
			theta_t = math.pi - math.acos((pow(self.F,2) + pow(self.T,2) - y0*y0 - pow((r - self.C),2)) / (2 * self.F * self.T)) # Write stuff here
			print(math.degrees(theta_c), math.degrees(theta_f), math.degrees(theta_t))
			return [math.degrees(theta_c), math.degrees(theta_f), math.degrees(theta_t)]
		except ValueError:
			print('No Solutions Found')
			return [-1,-1,-1]