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


	def inverse_k(self, x0, y0, z0):
		theta_c = math.atan(z0/x0)

		r = x0 / math.cos(theta_c)
		theta_f = math.pi/2 - math.atan((r-self.C)/y0) - math.acos((pow(self.F,2) + y0*y0 + pow((r - self.C),2) - pow(self.T,2)) / (2 * self.F * math.sqrt(y0*y0 + pow((r - self.C),2))))

		print(math.degrees(theta_c), math.degrees(theta_f))
		return 