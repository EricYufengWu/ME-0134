import math
import numpy as np 


'''
Base Coordinate System:
	Z
	^
	|
	Y ---> X  
Y into the page, Z is the direction of movement

Each leg has their coord origin located at the point of rotation of their coxia (rotating joint)
When defining the legs, an x,y,z coord is required to set up the start pos of each leg with offset.
Later commands will be simply shifting the positions of each leg from its start position.
for example, if I want the FR leg to move 10mm in z, do leg.inverse_k(0,0,10)

Current start pos:
front left: x0 = -50, y0 = 100, z0 = 50
front right: x0 = 50, y0 = 100, z0 = 50
mid left: x0 = -50, y0 = 100, z0 = 0
mid right: x0 = 50, y0 = 100, z0 = 0
back left: x0 = -50, y0 = 100, z0 = -50
back right: x0 = 50, y0 = 100, z0 = -50

For more info on the coord setup, checkout the jpeg images under https://github.com/EricYufengWu/ME-0134/tree/master/Final/IK_images
Website reference for IK calculation: https://hexyrobot.wordpress.com/2015/11/20/hexapod-leg-kinematics/
'''


class leg_IK:
	def __init__(self, width = 41.9, length = 22, coxia = 19.1, femur = 120.6, tibia = 202, x0 = 50, y0 = 100, z0 = 50):
		self.W = width   # Half the distance between the left and right base servo axis in X direction
		self.L = length  # Half the distance between the front and back servos in the Y direction
		self.C = coxia   # Coxia - Rotate
		self.F = femur   # Femur - Knee
		self.T = tibia   # Tibia - Ankle
		self.x0 = x0
		self.y0 = y0
		self.z0 = z0

	# Transfer coordinates from base coordinates to individual leg coordinate system
	def trans_coord(self, x_offs,y_offs,z_offs):
		# Does more stuff here
		return self.x0 + x_offs, self.y0 + y_offs, self.z0 + z_offs

	def inverse_k(self, x_offs=0, y_offs=0, z_offs=0):
		x, y, z = self.trans_coord(x_offs,y_offs,z_offs)
		print("solving IK for: ", x, y, z)

		try:
			theta_c = math.atan(z/x)
			r = x / math.cos(theta_c)
			theta_f = math.pi/2 - math.atan((r-self.C)/y) - math.acos((pow(self.F,2) + y*y + pow((r - self.C),2) - pow(self.T,2)) / (2 * self.F * math.sqrt(y*y + pow((r - self.C),2))))
			theta_t = math.pi - math.acos((pow(self.F,2) + pow(self.T,2) - y*y - pow((r - self.C),2)) / (2 * self.F * self.T)) # Write stuff here
			# print(math.degrees(theta_c), math.degrees(theta_f), math.degrees(theta_t))
			return [math.degrees(theta_c), math.degrees(theta_f), math.degrees(theta_t)]
		except ValueError:
			print('No Solutions Found')
			return [-1,-1,-1]


FR = leg_IK()
FL = leg_IK(x0 = -50)
MR = leg_IK(z0 = 0)
ML = leg_IK(x0 = -50, z0 = 0)
BR = leg_IK(z0 = -50)
BL = leg_IK(x0 = -50, z0 = -50)


# Currently working: FR, MR, BR; issues (I think): FL, ML, BL
FR.inverse_k(0,0,0)