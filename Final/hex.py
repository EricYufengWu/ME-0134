#hex.py
from adafruit_servokit import ServoKit
from time import sleep
class Joint:
    def __init__(self, driver, channel, minPW, maxPW, startAng, inverted = False):
        self.driver = driver
        self.channel = channel
        self.minPW = minPW
        self.maxPW = maxPW
        self.startAng = startAng
        self.goAng = self.startAng
        self.inverted = inverted

JOINT1 = Joint(0, 0, 500, 2600, 90)
JOINT2 = Joint(0, 1, 400, 2500, 90)
JOINT3 = Joint(0, 2, 450, 2550, 90)
JOINT4 = Joint(0, 4, 450, 2550, 90)
JOINT5 = Joint(0, 5, 380, 2480, 90)
JOINT6 = Joint(0, 6, 450, 2550, 90)
JOINT7 = Joint(0, 8, 450, 2550, 90)
JOINT8 = Joint(0, 9, 320, 2520, 90)
JOINT9 = Joint(0, 10, 400, 2500, 90)
JOINT10 = Joint(0, 12, 350, 2450, 90)
JOINT11 = Joint(0, 13, 400, 2500, 90)
JOINT12 = Joint(0, 14, 430, 2530, 90)
JOINT13 = Joint(1, 0, 300, 2400, 90)
JOINT14 = Joint(1, 1, 350, 2450, 90)
JOINT15 = Joint(1, 2, 400, 2500, 90)
JOINT16 = Joint(1, 12, 500, 2600, 90)
JOINT17 = Joint(1, 13, 400, 2500, 90)
JOINT18 = Joint(1, 14, 500, 2600, 90)
# LIDAR
JOINT19 = Joint(1, 7, 500, 2600, 90)

class Leg:
    def __init__(self, rotate, knee, ankle):
        self.rotate = rotate
        self.knee = knee		
        self.ankle = ankle		
        self.joints = [rotate, knee, ankle]

FL = Leg(JOINT1, JOINT2, JOINT3)
FR = Leg(JOINT7, JOINT8, JOINT9)
ML = Leg(JOINT4, JOINT5, JOINT6)
MR = Leg(JOINT10, JOINT11, JOINT12)
BL = Leg(JOINT13, JOINT14, JOINT15)
BR = Leg(JOINT16, JOINT17, JOINT18)

LEGS = [FL, FR, ML, MR, BL, BR]

kit_1 = ServoKit(channels=16)
kit_2 = ServoKit(channels=16, address=0x41)

drivers = [kit_1, kit_2]

def set_range():
    for leg in LEGS:
        for joint in leg.joints:
            mtr = drivers[joint.driver].servo[joint.channel]
            mtr.set_pulse_width_range(joint.minPW, joint.maxPW)

def home():
    for leg in LEGS:
        for joint in leg.joints:
            drivers[joint.driver].servo[joint.channel].angle = joint.startAng
    sleep(1)

set_range()
home()