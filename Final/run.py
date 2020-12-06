from time import sleep
from Leg import Leg
from Joint import Joint

#from adafruit_servokit import ServoKit

JOINT1 = Joint(0, 0, 500, 2600, 120)
JOINT2 = Joint(0, 1, 400, 2500, 30)
JOINT3 = Joint(0, 2, 450, 2550, 30)
JOINT4 = Joint(0, 4, 450, 2550, 90)
JOINT5 = Joint(0, 5, 380, 2480, 30)
JOINT6 = Joint(0, 6, 450, 2550, 30)
JOINT7 = Joint(0, 8, 450, 2550, 60)
JOINT8 = Joint(0, 9, 320, 2520, 30)
JOINT9 = Joint(0, 10, 400, 2500, 30)
JOINT10 = Joint(0, 12, 350, 2450, 90)
JOINT11 = Joint(0, 13, 400, 2500, 30)
JOINT12 = Joint(0, 14, 400, 2600, 30)
JOINT13 = Joint(1, 0, 300, 2400, 60)
JOINT14 = Joint(1, 1, 350, 2450, 30)
JOINT15 = Joint(1, 2, 400, 2500, 30)
JOINT16 = Joint(1, 12, 500, 2600, 120)
JOINT17 = Joint(1, 13, 400, 2500, 30)
JOINT18 = Joint(1, 14, 500, 2600, 30)
# LIDAR
JOINT19 = Joint(1, 7, 500, 2600, 90)

FL = Leg(JOINT1, JOINT2, JOINT3)
ML = Leg(JOINT4, JOINT5, JOINT6)
FR = Leg(JOINT7, JOINT8, JOINT9)
MR = Leg(JOINT10, JOINT11, JOINT12)
BL = Leg(JOINT13, JOINT14, JOINT15)
BR = Leg(JOINT16, JOINT17, JOINT18)

LEGS = [FL, ML, FR, MR, BL, BR]

kit_1 = ServoKit(channels=16)
kit_2 = ServoKit(channels=16, address=0x41)

drivers = [kit_1, kit_2]

def set_range():
    for leg in LEGS:
        for joint in leg.joints:
            drivers[joint.driver].servo[joint.channel].set_pulse_width_range(joint.minPW, joint.maxPW)
            

def home():
    for leg in LEGS:
        for joint in leg.joints:
            drivers[joint.driver].servo[joint.channel].angle = joint.startAng
    sleep(1)


set_range()
home()


# calculate the inverse kinematics 
FR.inverse_k(0,0,0)

# DEBUGGING
print(FR.joints[0].goAng)
print(FR.joints[1].goAng)
print(FR.joints[2].goAng)

# Create path of angles
for i in range(len(FR.joints)):
    FR.joints[i].interpolate() 


for joint in FR.joints:
    for angle in joint.angle_sweep: 
        print('i')
        #drivers[joint.driver].servo[joint.channel].angle = angle
sleep(1)

