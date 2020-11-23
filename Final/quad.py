#quad.py

class Joint:
	def __init__(self, channel, startAng):
		self.channel = channel
		self.startAng = startAng
		self.goAng = self.startAng

JOINT1 = Joint(0, 90)
JOINT2 = Joint(1, 90)
JOINT3 = Joint(2, 90)
JOINT4 = Joint(8, 90)
JOINT5 = Joint(9, 90)
JOINT6 = Joint(10, 90)
JOINT7 = Joint(4, 90)
JOINT8 = Joint(5, 90)
JOINT9 = Joint(6, 90)
JOINT10 = Joint(12, 90)
JOINT11 = Joint(13, 90)
JOINT12 = Joint(14, 90)

class Leg:
	def __init__(self, rotate, bend1, bend2):
		self.rotate = rotate
		self.bend1 = bend1
		self.bend2 = bend2
		self.joints = [rotate, bend1, bend2]

FL = Leg(JOINT1, JOINT2, JOINT3)
FR = Leg(JOINT4, JOINT5, JOINT6)
BL = Leg(JOINT7, JOINT8, JOINT9)
BR = Leg(JOINT10, JOINT11, JOINT12)

LEGS = [FL, FR, BL, BR]