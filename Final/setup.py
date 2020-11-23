#setup.py
from adafruit_servokit import ServoKit
import time
from quad import LEGS

# initalize and setup functions
kit = ServoKit(channels=16)

def set_range():
    kit.servo[0].set_pulse_width_range(500, 2600)
    kit.servo[1].set_pulse_width_range(400, 2500)
    kit.servo[2].set_pulse_width_range(400, 2500)
    kit.servo[4].set_pulse_width_range(450, 2550)
    kit.servo[5].set_pulse_width_range(380, 2480)
    kit.servo[6].set_pulse_width_range(450, 2550)
    kit.servo[8].set_pulse_width_range(450, 2550)
    kit.servo[9].set_pulse_width_range(320, 2520)
    kit.servo[10].set_pulse_width_range(500, 2600)
    kit.servo[12].set_pulse_width_range(350, 2450)
    kit.servo[13].set_pulse_width_range(400, 2500)
    kit.servo[14].set_pulse_width_range(350, 2450)
set_range()

#def home():
#    for i in range(16):
#        kit.servo[i].angle = 90

def move(legs):
    for leg in legs:
        for joint in leg.joints:
            kit.servo[joint.channel].angle = joint.goAng

def step_leg(leg):
    leg.rotate.goAng = 110
    leg.bend1.goAng = 60
    leg.bend2.goAng = 140
    move(LEGS)

def rotate_joint(leg, angle):
    leg.rotate.goAng = angle
    move(LEGS)

def bend1_joint(leg, angle):
    leg.bend1.goAng = angle
    move(LEGS)

def bend2_joint(leg, angle):
    leg.bend2.goAng = angle
    move(LEGS)

fl = LEGS[0]
fr = LEGS[1]
bl = LEGS[2]
br = LEGS[3]

# home motors at initial angle (all 90)
move(LEGS)


