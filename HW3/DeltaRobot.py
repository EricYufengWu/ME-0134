'''
Delta Drawing Robot Library

The inverse kinematics model below is an implementation of the kinematics calculation from the following website:
http://hypertriangself.Le.com/~aself.Lex/delta-robot-tutorial/

Coordinates Configuration: 

		theta2
		   |   
		   
		   O  __   theta1
		/
	theta3

		   Z
		   |   X
		   |  /
	   Y<--    
Notes:
--For the Cartesian Coordinate System, (0,0,0) is located at the center of the base radius
--Theta1 is pointing to the negative diself.Rection of Y axis. 
'''
from adafruit_servokit import ServoKit
import math
import time

class DeltaKinematics:
	def __init__(self, end_radius=20.65, base_radius=45.32, end_arm=248, base_arm=76.75, angle_min = 5, angle_max = 90):
		self.Re = end_radius
		self.Rb = base_radius
		self.Le = end_arm
		self.Lb = base_arm
		self.angle_min = angle_min
		self.angle_max = angle_max
		self.cos120 = -0.5
		self.sin120 = math.sqrt(3)/2.0 

	def calculate_angle_yz(self, x0, y0, z0):
		Jy = []
		Jz = []
		sol = []
		Jy.append((self.Lb**2 - self.Le**2 - self.Rb**2 + self.Re**2 - 2*self.Re*y0 + x0**2 + y0**2 + z0**2)/(2*(self.Rb - self.Re + y0)) - (z0*(self.Lb**2*z0 - self.Le**2*z0 + self.Rb**2*z0 + self.Re**2*z0 + x0**2*z0 + y0**2*z0 + self.Rb*(- self.Lb**4 + 2*self.Lb**2*self.Le**2 + 2*self.Lb**2*self.Rb**2 - 4*self.Lb**2*self.Rb*self.Re + 4*self.Lb**2*self.Rb*y0 + 2*self.Lb**2*self.Re**2 - 4*self.Lb**2*self.Re*y0 - 2*self.Lb**2*x0**2 + 2*self.Lb**2*y0**2 + 2*self.Lb**2*z0**2 - self.Le**4 + 2*self.Le**2*self.Rb**2 - 4*self.Le**2*self.Rb*self.Re + 4*self.Le**2*self.Rb*y0 + 2*self.Le**2*self.Re**2 - 4*self.Le**2*self.Re*y0 + 2*self.Le**2*x0**2 + 2*self.Le**2*y0**2 + 2*self.Le**2*z0**2 - self.Rb**4 + 4*self.Rb**3*self.Re - 4*self.Rb**3*y0 - 6*self.Rb**2*self.Re**2 + 12*self.Rb**2*self.Re*y0 - 2*self.Rb**2*x0**2 - 6*self.Rb**2*y0**2 - 2*self.Rb**2*z0**2 + 4*self.Rb*self.Re**3 - 12*self.Rb*self.Re**2*y0 + 4*self.Rb*self.Re*x0**2 + 12*self.Rb*self.Re*y0**2 + 4*self.Rb*self.Re*z0**2 - 4*self.Rb*x0**2*y0 - 4*self.Rb*y0**3 - 4*self.Rb*y0*z0**2 - self.Re**4 + 4*self.Re**3*y0 - 2*self.Re**2*x0**2 - 6*self.Re**2*y0**2 - 2*self.Re**2*z0**2 + 4*self.Re*x0**2*y0 + 4*self.Re*y0**3 + 4*self.Re*y0*z0**2 - x0**4 - 2*x0**2*y0**2 - 2*x0**2*z0**2 - y0**4 - 2*y0**2*z0**2 - z0**4)**(1/2) - self.Re*(- self.Lb**4 + 2*self.Lb**2*self.Le**2 + 2*self.Lb**2*self.Rb**2 - 4*self.Lb**2*self.Rb*self.Re + 4*self.Lb**2*self.Rb*y0 + 2*self.Lb**2*self.Re**2 - 4*self.Lb**2*self.Re*y0 - 2*self.Lb**2*x0**2 + 2*self.Lb**2*y0**2 + 2*self.Lb**2*z0**2 - self.Le**4 + 2*self.Le**2*self.Rb**2 - 4*self.Le**2*self.Rb*self.Re + 4*self.Le**2*self.Rb*y0 + 2*self.Le**2*self.Re**2 - 4*self.Le**2*self.Re*y0 + 2*self.Le**2*x0**2 + 2*self.Le**2*y0**2 + 2*self.Le**2*z0**2 - self.Rb**4 + 4*self.Rb**3*self.Re - 4*self.Rb**3*y0 - 6*self.Rb**2*self.Re**2 + 12*self.Rb**2*self.Re*y0 - 2*self.Rb**2*x0**2 - 6*self.Rb**2*y0**2 - 2*self.Rb**2*z0**2 + 4*self.Rb*self.Re**3 - 12*self.Rb*self.Re**2*y0 + 4*self.Rb*self.Re*x0**2 + 12*self.Rb*self.Re*y0**2 + 4*self.Rb*self.Re*z0**2 - 4*self.Rb*x0**2*y0 - 4*self.Rb*y0**3 - 4*self.Rb*y0*z0**2 - self.Re**4 + 4*self.Re**3*y0 - 2*self.Re**2*x0**2 - 6*self.Re**2*y0**2 - 2*self.Re**2*z0**2 + 4*self.Re*x0**2*y0 + 4*self.Re*y0**3 + 4*self.Re*y0*z0**2 - x0**4 - 2*x0**2*y0**2 - 2*x0**2*z0**2 - y0**4 - 2*y0**2*z0**2 - z0**4)**(1/2) + z0**3 + y0*(- self.Lb**4 + 2*self.Lb**2*self.Le**2 + 2*self.Lb**2*self.Rb**2 - 4*self.Lb**2*self.Rb*self.Re + 4*self.Lb**2*self.Rb*y0 + 2*self.Lb**2*self.Re**2 - 4*self.Lb**2*self.Re*y0 - 2*self.Lb**2*x0**2 + 2*self.Lb**2*y0**2 + 2*self.Lb**2*z0**2 - self.Le**4 + 2*self.Le**2*self.Rb**2 - 4*self.Le**2*self.Rb*self.Re + 4*self.Le**2*self.Rb*y0 + 2*self.Le**2*self.Re**2 - 4*self.Le**2*self.Re*y0 + 2*self.Le**2*x0**2 + 2*self.Le**2*y0**2 + 2*self.Le**2*z0**2 - self.Rb**4 + 4*self.Rb**3*self.Re - 4*self.Rb**3*y0 - 6*self.Rb**2*self.Re**2 + 12*self.Rb**2*self.Re*y0 - 2*self.Rb**2*x0**2 - 6*self.Rb**2*y0**2 - 2*self.Rb**2*z0**2 + 4*self.Rb*self.Re**3 - 12*self.Rb*self.Re**2*y0 + 4*self.Rb*self.Re*x0**2 + 12*self.Rb*self.Re*y0**2 + 4*self.Rb*self.Re*z0**2 - 4*self.Rb*x0**2*y0 - 4*self.Rb*y0**3 - 4*self.Rb*y0*z0**2 - self.Re**4 + 4*self.Re**3*y0 - 2*self.Re**2*x0**2 - 6*self.Re**2*y0**2 - 2*self.Re**2*z0**2 + 4*self.Re*x0**2*y0 + 4*self.Re*y0**3 + 4*self.Re*y0*z0**2 - x0**4 - 2*x0**2*y0**2 - 2*x0**2*z0**2 - y0**4 - 2*y0**2*z0**2 - z0**4)**(1/2) - 2*self.Rb*self.Re*z0 + 2*self.Rb*y0*z0 - 2*self.Re*y0*z0))/(2*(self.Rb - self.Re + y0)*(self.Rb**2 - 2*self.Rb*self.Re + 2*self.Rb*y0 + self.Re**2 - 2*self.Re*y0 + y0**2 + z0**2)))
		Jy.append((self.Lb**2 - self.Le**2 - self.Rb**2 + self.Re**2 - 2*self.Re*y0 + x0**2 + y0**2 + z0**2)/(2*(self.Rb - self.Re + y0)) - (z0*(self.Lb**2*z0 - self.Le**2*z0 + self.Rb**2*z0 + self.Re**2*z0 + x0**2*z0 + y0**2*z0 - self.Rb*(- self.Lb**4 + 2*self.Lb**2*self.Le**2 + 2*self.Lb**2*self.Rb**2 - 4*self.Lb**2*self.Rb*self.Re + 4*self.Lb**2*self.Rb*y0 + 2*self.Lb**2*self.Re**2 - 4*self.Lb**2*self.Re*y0 - 2*self.Lb**2*x0**2 + 2*self.Lb**2*y0**2 + 2*self.Lb**2*z0**2 - self.Le**4 + 2*self.Le**2*self.Rb**2 - 4*self.Le**2*self.Rb*self.Re + 4*self.Le**2*self.Rb*y0 + 2*self.Le**2*self.Re**2 - 4*self.Le**2*self.Re*y0 + 2*self.Le**2*x0**2 + 2*self.Le**2*y0**2 + 2*self.Le**2*z0**2 - self.Rb**4 + 4*self.Rb**3*self.Re - 4*self.Rb**3*y0 - 6*self.Rb**2*self.Re**2 + 12*self.Rb**2*self.Re*y0 - 2*self.Rb**2*x0**2 - 6*self.Rb**2*y0**2 - 2*self.Rb**2*z0**2 + 4*self.Rb*self.Re**3 - 12*self.Rb*self.Re**2*y0 + 4*self.Rb*self.Re*x0**2 + 12*self.Rb*self.Re*y0**2 + 4*self.Rb*self.Re*z0**2 - 4*self.Rb*x0**2*y0 - 4*self.Rb*y0**3 - 4*self.Rb*y0*z0**2 - self.Re**4 + 4*self.Re**3*y0 - 2*self.Re**2*x0**2 - 6*self.Re**2*y0**2 - 2*self.Re**2*z0**2 + 4*self.Re*x0**2*y0 + 4*self.Re*y0**3 + 4*self.Re*y0*z0**2 - x0**4 - 2*x0**2*y0**2 - 2*x0**2*z0**2 - y0**4 - 2*y0**2*z0**2 - z0**4)**(1/2) + self.Re*(- self.Lb**4 + 2*self.Lb**2*self.Le**2 + 2*self.Lb**2*self.Rb**2 - 4*self.Lb**2*self.Rb*self.Re + 4*self.Lb**2*self.Rb*y0 + 2*self.Lb**2*self.Re**2 - 4*self.Lb**2*self.Re*y0 - 2*self.Lb**2*x0**2 + 2*self.Lb**2*y0**2 + 2*self.Lb**2*z0**2 - self.Le**4 + 2*self.Le**2*self.Rb**2 - 4*self.Le**2*self.Rb*self.Re + 4*self.Le**2*self.Rb*y0 + 2*self.Le**2*self.Re**2 - 4*self.Le**2*self.Re*y0 + 2*self.Le**2*x0**2 + 2*self.Le**2*y0**2 + 2*self.Le**2*z0**2 - self.Rb**4 + 4*self.Rb**3*self.Re - 4*self.Rb**3*y0 - 6*self.Rb**2*self.Re**2 + 12*self.Rb**2*self.Re*y0 - 2*self.Rb**2*x0**2 - 6*self.Rb**2*y0**2 - 2*self.Rb**2*z0**2 + 4*self.Rb*self.Re**3 - 12*self.Rb*self.Re**2*y0 + 4*self.Rb*self.Re*x0**2 + 12*self.Rb*self.Re*y0**2 + 4*self.Rb*self.Re*z0**2 - 4*self.Rb*x0**2*y0 - 4*self.Rb*y0**3 - 4*self.Rb*y0*z0**2 - self.Re**4 + 4*self.Re**3*y0 - 2*self.Re**2*x0**2 - 6*self.Re**2*y0**2 - 2*self.Re**2*z0**2 + 4*self.Re*x0**2*y0 + 4*self.Re*y0**3 + 4*self.Re*y0*z0**2 - x0**4 - 2*x0**2*y0**2 - 2*x0**2*z0**2 - y0**4 - 2*y0**2*z0**2 - z0**4)**(1/2) + z0**3 - y0*(- self.Lb**4 + 2*self.Lb**2*self.Le**2 + 2*self.Lb**2*self.Rb**2 - 4*self.Lb**2*self.Rb*self.Re + 4*self.Lb**2*self.Rb*y0 + 2*self.Lb**2*self.Re**2 - 4*self.Lb**2*self.Re*y0 - 2*self.Lb**2*x0**2 + 2*self.Lb**2*y0**2 + 2*self.Lb**2*z0**2 - self.Le**4 + 2*self.Le**2*self.Rb**2 - 4*self.Le**2*self.Rb*self.Re + 4*self.Le**2*self.Rb*y0 + 2*self.Le**2*self.Re**2 - 4*self.Le**2*self.Re*y0 + 2*self.Le**2*x0**2 + 2*self.Le**2*y0**2 + 2*self.Le**2*z0**2 - self.Rb**4 + 4*self.Rb**3*self.Re - 4*self.Rb**3*y0 - 6*self.Rb**2*self.Re**2 + 12*self.Rb**2*self.Re*y0 - 2*self.Rb**2*x0**2 - 6*self.Rb**2*y0**2 - 2*self.Rb**2*z0**2 + 4*self.Rb*self.Re**3 - 12*self.Rb*self.Re**2*y0 + 4*self.Rb*self.Re*x0**2 + 12*self.Rb*self.Re*y0**2 + 4*self.Rb*self.Re*z0**2 - 4*self.Rb*x0**2*y0 - 4*self.Rb*y0**3 - 4*self.Rb*y0*z0**2 - self.Re**4 + 4*self.Re**3*y0 - 2*self.Re**2*x0**2 - 6*self.Re**2*y0**2 - 2*self.Re**2*z0**2 + 4*self.Re*x0**2*y0 + 4*self.Re*y0**3 + 4*self.Re*y0*z0**2 - x0**4 - 2*x0**2*y0**2 - 2*x0**2*z0**2 - y0**4 - 2*y0**2*z0**2 - z0**4)**(1/2) - 2*self.Rb*self.Re*z0 + 2*self.Rb*y0*z0 - 2*self.Re*y0*z0))/(2*(self.Rb - self.Re + y0)*(self.Rb**2 - 2*self.Rb*self.Re + 2*self.Rb*y0 + self.Re**2 - 2*self.Re*y0 + y0**2 + z0**2)))
		Jz.append((self.Lb**2*z0 - self.Le**2*z0 + self.Rb**2*z0 + self.Re**2*z0 + x0**2*z0 + y0**2*z0 + self.Rb*(- self.Lb**4 + 2*self.Lb**2*self.Le**2 + 2*self.Lb**2*self.Rb**2 - 4*self.Lb**2*self.Rb*self.Re + 4*self.Lb**2*self.Rb*y0 + 2*self.Lb**2*self.Re**2 - 4*self.Lb**2*self.Re*y0 - 2*self.Lb**2*x0**2 + 2*self.Lb**2*y0**2 + 2*self.Lb**2*z0**2 - self.Le**4 + 2*self.Le**2*self.Rb**2 - 4*self.Le**2*self.Rb*self.Re + 4*self.Le**2*self.Rb*y0 + 2*self.Le**2*self.Re**2 - 4*self.Le**2*self.Re*y0 + 2*self.Le**2*x0**2 + 2*self.Le**2*y0**2 + 2*self.Le**2*z0**2 - self.Rb**4 + 4*self.Rb**3*self.Re - 4*self.Rb**3*y0 - 6*self.Rb**2*self.Re**2 + 12*self.Rb**2*self.Re*y0 - 2*self.Rb**2*x0**2 - 6*self.Rb**2*y0**2 - 2*self.Rb**2*z0**2 + 4*self.Rb*self.Re**3 - 12*self.Rb*self.Re**2*y0 + 4*self.Rb*self.Re*x0**2 + 12*self.Rb*self.Re*y0**2 + 4*self.Rb*self.Re*z0**2 - 4*self.Rb*x0**2*y0 - 4*self.Rb*y0**3 - 4*self.Rb*y0*z0**2 - self.Re**4 + 4*self.Re**3*y0 - 2*self.Re**2*x0**2 - 6*self.Re**2*y0**2 - 2*self.Re**2*z0**2 + 4*self.Re*x0**2*y0 + 4*self.Re*y0**3 + 4*self.Re*y0*z0**2 - x0**4 - 2*x0**2*y0**2 - 2*x0**2*z0**2 - y0**4 - 2*y0**2*z0**2 - z0**4)**(1/2) - self.Re*(- self.Lb**4 + 2*self.Lb**2*self.Le**2 + 2*self.Lb**2*self.Rb**2 - 4*self.Lb**2*self.Rb*self.Re + 4*self.Lb**2*self.Rb*y0 + 2*self.Lb**2*self.Re**2 - 4*self.Lb**2*self.Re*y0 - 2*self.Lb**2*x0**2 + 2*self.Lb**2*y0**2 + 2*self.Lb**2*z0**2 - self.Le**4 + 2*self.Le**2*self.Rb**2 - 4*self.Le**2*self.Rb*self.Re + 4*self.Le**2*self.Rb*y0 + 2*self.Le**2*self.Re**2 - 4*self.Le**2*self.Re*y0 + 2*self.Le**2*x0**2 + 2*self.Le**2*y0**2 + 2*self.Le**2*z0**2 - self.Rb**4 + 4*self.Rb**3*self.Re - 4*self.Rb**3*y0 - 6*self.Rb**2*self.Re**2 + 12*self.Rb**2*self.Re*y0 - 2*self.Rb**2*x0**2 - 6*self.Rb**2*y0**2 - 2*self.Rb**2*z0**2 + 4*self.Rb*self.Re**3 - 12*self.Rb*self.Re**2*y0 + 4*self.Rb*self.Re*x0**2 + 12*self.Rb*self.Re*y0**2 + 4*self.Rb*self.Re*z0**2 - 4*self.Rb*x0**2*y0 - 4*self.Rb*y0**3 - 4*self.Rb*y0*z0**2 - self.Re**4 + 4*self.Re**3*y0 - 2*self.Re**2*x0**2 - 6*self.Re**2*y0**2 - 2*self.Re**2*z0**2 + 4*self.Re*x0**2*y0 + 4*self.Re*y0**3 + 4*self.Re*y0*z0**2 - x0**4 - 2*x0**2*y0**2 - 2*x0**2*z0**2 - y0**4 - 2*y0**2*z0**2 - z0**4)**(1/2) + z0**3 + y0*(- self.Lb**4 + 2*self.Lb**2*self.Le**2 + 2*self.Lb**2*self.Rb**2 - 4*self.Lb**2*self.Rb*self.Re + 4*self.Lb**2*self.Rb*y0 + 2*self.Lb**2*self.Re**2 - 4*self.Lb**2*self.Re*y0 - 2*self.Lb**2*x0**2 + 2*self.Lb**2*y0**2 + 2*self.Lb**2*z0**2 - self.Le**4 + 2*self.Le**2*self.Rb**2 - 4*self.Le**2*self.Rb*self.Re + 4*self.Le**2*self.Rb*y0 + 2*self.Le**2*self.Re**2 - 4*self.Le**2*self.Re*y0 + 2*self.Le**2*x0**2 + 2*self.Le**2*y0**2 + 2*self.Le**2*z0**2 - self.Rb**4 + 4*self.Rb**3*self.Re - 4*self.Rb**3*y0 - 6*self.Rb**2*self.Re**2 + 12*self.Rb**2*self.Re*y0 - 2*self.Rb**2*x0**2 - 6*self.Rb**2*y0**2 - 2*self.Rb**2*z0**2 + 4*self.Rb*self.Re**3 - 12*self.Rb*self.Re**2*y0 + 4*self.Rb*self.Re*x0**2 + 12*self.Rb*self.Re*y0**2 + 4*self.Rb*self.Re*z0**2 - 4*self.Rb*x0**2*y0 - 4*self.Rb*y0**3 - 4*self.Rb*y0*z0**2 - self.Re**4 + 4*self.Re**3*y0 - 2*self.Re**2*x0**2 - 6*self.Re**2*y0**2 - 2*self.Re**2*z0**2 + 4*self.Re*x0**2*y0 + 4*self.Re*y0**3 + 4*self.Re*y0*z0**2 - x0**4 - 2*x0**2*y0**2 - 2*x0**2*z0**2 - y0**4 - 2*y0**2*z0**2 - z0**4)**(1/2) - 2*self.Rb*self.Re*z0 + 2*self.Rb*y0*z0 - 2*self.Re*y0*z0)/(2*(self.Rb**2 - 2*self.Rb*self.Re + 2*self.Rb*y0 + self.Re**2 - 2*self.Re*y0 + y0**2 + z0**2)))
		Jz.append((self.Lb**2*z0 - self.Le**2*z0 + self.Rb**2*z0 + self.Re**2*z0 + x0**2*z0 + y0**2*z0 - self.Rb*(- self.Lb**4 + 2*self.Lb**2*self.Le**2 + 2*self.Lb**2*self.Rb**2 - 4*self.Lb**2*self.Rb*self.Re + 4*self.Lb**2*self.Rb*y0 + 2*self.Lb**2*self.Re**2 - 4*self.Lb**2*self.Re*y0 - 2*self.Lb**2*x0**2 + 2*self.Lb**2*y0**2 + 2*self.Lb**2*z0**2 - self.Le**4 + 2*self.Le**2*self.Rb**2 - 4*self.Le**2*self.Rb*self.Re + 4*self.Le**2*self.Rb*y0 + 2*self.Le**2*self.Re**2 - 4*self.Le**2*self.Re*y0 + 2*self.Le**2*x0**2 + 2*self.Le**2*y0**2 + 2*self.Le**2*z0**2 - self.Rb**4 + 4*self.Rb**3*self.Re - 4*self.Rb**3*y0 - 6*self.Rb**2*self.Re**2 + 12*self.Rb**2*self.Re*y0 - 2*self.Rb**2*x0**2 - 6*self.Rb**2*y0**2 - 2*self.Rb**2*z0**2 + 4*self.Rb*self.Re**3 - 12*self.Rb*self.Re**2*y0 + 4*self.Rb*self.Re*x0**2 + 12*self.Rb*self.Re*y0**2 + 4*self.Rb*self.Re*z0**2 - 4*self.Rb*x0**2*y0 - 4*self.Rb*y0**3 - 4*self.Rb*y0*z0**2 - self.Re**4 + 4*self.Re**3*y0 - 2*self.Re**2*x0**2 - 6*self.Re**2*y0**2 - 2*self.Re**2*z0**2 + 4*self.Re*x0**2*y0 + 4*self.Re*y0**3 + 4*self.Re*y0*z0**2 - x0**4 - 2*x0**2*y0**2 - 2*x0**2*z0**2 - y0**4 - 2*y0**2*z0**2 - z0**4)**(1/2) + self.Re*(- self.Lb**4 + 2*self.Lb**2*self.Le**2 + 2*self.Lb**2*self.Rb**2 - 4*self.Lb**2*self.Rb*self.Re + 4*self.Lb**2*self.Rb*y0 + 2*self.Lb**2*self.Re**2 - 4*self.Lb**2*self.Re*y0 - 2*self.Lb**2*x0**2 + 2*self.Lb**2*y0**2 + 2*self.Lb**2*z0**2 - self.Le**4 + 2*self.Le**2*self.Rb**2 - 4*self.Le**2*self.Rb*self.Re + 4*self.Le**2*self.Rb*y0 + 2*self.Le**2*self.Re**2 - 4*self.Le**2*self.Re*y0 + 2*self.Le**2*x0**2 + 2*self.Le**2*y0**2 + 2*self.Le**2*z0**2 - self.Rb**4 + 4*self.Rb**3*self.Re - 4*self.Rb**3*y0 - 6*self.Rb**2*self.Re**2 + 12*self.Rb**2*self.Re*y0 - 2*self.Rb**2*x0**2 - 6*self.Rb**2*y0**2 - 2*self.Rb**2*z0**2 + 4*self.Rb*self.Re**3 - 12*self.Rb*self.Re**2*y0 + 4*self.Rb*self.Re*x0**2 + 12*self.Rb*self.Re*y0**2 + 4*self.Rb*self.Re*z0**2 - 4*self.Rb*x0**2*y0 - 4*self.Rb*y0**3 - 4*self.Rb*y0*z0**2 - self.Re**4 + 4*self.Re**3*y0 - 2*self.Re**2*x0**2 - 6*self.Re**2*y0**2 - 2*self.Re**2*z0**2 + 4*self.Re*x0**2*y0 + 4*self.Re*y0**3 + 4*self.Re*y0*z0**2 - x0**4 - 2*x0**2*y0**2 - 2*x0**2*z0**2 - y0**4 - 2*y0**2*z0**2 - z0**4)**(1/2) + z0**3 - y0*(- self.Lb**4 + 2*self.Lb**2*self.Le**2 + 2*self.Lb**2*self.Rb**2 - 4*self.Lb**2*self.Rb*self.Re + 4*self.Lb**2*self.Rb*y0 + 2*self.Lb**2*self.Re**2 - 4*self.Lb**2*self.Re*y0 - 2*self.Lb**2*x0**2 + 2*self.Lb**2*y0**2 + 2*self.Lb**2*z0**2 - self.Le**4 + 2*self.Le**2*self.Rb**2 - 4*self.Le**2*self.Rb*self.Re + 4*self.Le**2*self.Rb*y0 + 2*self.Le**2*self.Re**2 - 4*self.Le**2*self.Re*y0 + 2*self.Le**2*x0**2 + 2*self.Le**2*y0**2 + 2*self.Le**2*z0**2 - self.Rb**4 + 4*self.Rb**3*self.Re - 4*self.Rb**3*y0 - 6*self.Rb**2*self.Re**2 + 12*self.Rb**2*self.Re*y0 - 2*self.Rb**2*x0**2 - 6*self.Rb**2*y0**2 - 2*self.Rb**2*z0**2 + 4*self.Rb*self.Re**3 - 12*self.Rb*self.Re**2*y0 + 4*self.Rb*self.Re*x0**2 + 12*self.Rb*self.Re*y0**2 + 4*self.Rb*self.Re*z0**2 - 4*self.Rb*x0**2*y0 - 4*self.Rb*y0**3 - 4*self.Rb*y0*z0**2 - self.Re**4 + 4*self.Re**3*y0 - 2*self.Re**2*x0**2 - 6*self.Re**2*y0**2 - 2*self.Re**2*z0**2 + 4*self.Re*x0**2*y0 + 4*self.Re*y0**3 + 4*self.Re*y0*z0**2 - x0**4 - 2*x0**2*y0**2 - 2*x0**2*z0**2 - y0**4 - 2*y0**2*z0**2 - z0**4)**(1/2) - 2*self.Rb*self.Re*z0 + 2*self.Rb*y0*z0 - 2*self.Re*y0*z0)/(2*(self.Rb**2 - 2*self.Rb*self.Re + 2*self.Rb*y0 + self.Re**2 - 2*self.Re*y0 + y0**2 + z0**2)))
		# Based on the way the base coordinate system is set, it is impossibself.Le for Jy to be positive. 
		# print(Jy, Jz)
		try:  
			for i in range(len(Jy)):
				if Jy[i] < 0:
					theta = math.atan(-Jz[i] / (-self.Rb - Jy[i])) * 180 / math.pi
					# print(theta)
					return theta
			return False
		except:
			return False   

	def inverse_kinematics(self, x0, y0, z0):
		theta_A = self.calculate_angle_yz(x0, y0, z0)
		if theta_A != False:
			theta_B = self.calculate_angle_yz(x0*self.cos120 + y0*self.sin120, y0*self.cos120 - x0*self.sin120, z0)
			if theta_B != False:
				theta_C = self.calculate_angle_yz(x0*self.cos120 - y0*self.sin120, y0*self.cos120+x0*self.sin120, z0)
				return theta_A, theta_B, theta_C if theta_C != False else theta_C
	 

class DeltaMotion:
	def __init__(self, s_range = 135, c_range = 120, zero = 60, driver_channel = 16):
		self.servo_range = s_range
		self.clamp_range = c_range
		self.clamp_up = c_range
		self.clamp_down = 36
		self.zero = 60
		self.kit = ServoKit(channels = driver_channel)
		self.init_robot()
		self.move_all(self.zero)

	def init_robot(self):
		for i in range(3):
			self.kit.servo[i].actuation_range = self.servo_range
		self.kit.servo[3].actuation_range = self.clamp_range
		self.kit.servo[4].actuation_range = self.clamp_range

	def move_all(self, angle):
		for i in range(3):
			self.kit.servo[i].angle = angle

	def move_axis(self, axis, angle):
		if angle_list != None:
			self.kit.servo[axis - 1].angle = self.zero + angle

	def move_robot(self, angle_list):
		if angle_list != None:
			for i in range(len(angle_list)):
				self.kit.servo[i].angle = self.zero + angle_list[i]

	def clamp_down(self):
		self.kit.servo[3].angle = self.clamp_down
		self.kit.servo[4].angle = self.clamp_down

	def clamp_up(self):
		self.kit.servo[3].angle = self.clamp_up
		self.kit.servo[4].angle = self.clamp_up

	