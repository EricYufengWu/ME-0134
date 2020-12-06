from Joint import Joint
from Leg import Leg
from adafruit_servokit import ServoKit
from time import sleep

class Hex:
    def __init__(self):

        # Joints creation

        self.JOINT1 = Joint(0, 0, 500, 2600, 120, inverted = True)
        self.JOINT2 = Joint(0, 1, 400, 2500, 30)
        self.JOINT3 = Joint(0, 2, 450, 2550, 30)
        self.JOINT4 = Joint(0, 4, 450, 2550, 90, inverted = True)
        self.JOINT5 = Joint(0, 5, 380, 2480, 30)
        self.JOINT6 = Joint(0, 6, 450, 2550, 30)
        self.JOINT7 = Joint(0, 8, 450, 2550, 60)
        self.JOINT8 = Joint(0, 9, 320, 2520, 30)
        self.JOINT9 = Joint(0, 10, 400, 2500, 30)
        self.JOINT10 = Joint(0, 12, 350, 2450, 90)
        self.JOINT11 = Joint(0, 13, 400, 2500, 30)
        self.JOINT12 = Joint(0, 14, 400, 2600, 30)
        self.JOINT13 = Joint(1, 0, 300, 2400, 60, inverted = True)
        self.JOINT14 = Joint(1, 1, 350, 2450, 30)
        self.JOINT15 = Joint(1, 2, 400, 2500, 30)
        self.JOINT16 = Joint(1, 12, 500, 2600, 120)
        self.JOINT17 = Joint(1, 13, 400, 2500, 30)
        self.JOINT18 = Joint(1, 14, 500, 2600, 30)
        # LIDAR
        self.JOINT19 = Joint(1, 7, 500, 2600, 90)
        print('Assigned joints...')                
        # Leg creation

        self.FL = Leg(self.JOINT1, self.JOINT2, self.JOINT3, rotate_offset = 75)
        self.ML = Leg(self.JOINT4, self.JOINT5, self.JOINT6, rotate_offset = 45)
        self.FR = Leg(self.JOINT7, self.JOINT8, self.JOINT9, rotate_offset = 15)
        self.MR = Leg(self.JOINT10, self.JOINT11, self.JOINT12, rotate_offset = 45)
        self.BL = Leg(self.JOINT13, self.JOINT14, self.JOINT15, rotate_offset = 15)
        self.BR = Leg(self.JOINT16, self.JOINT17, self.JOINT18, rotate_offset = 75)
        print('Assigned legs...')
        # Leg assigments and rotation offsets 
        self.LEGS = [self.FL, self.ML, self.BL, self.FR, self.MR, self.BR]
        self.LEGS_L = [self.FL,  75, self.ML, 45, self.BL, 15]
        self.LEGS_R = [self.FR, 15, self.MR, 45, self.BR, 75]


        # Create the drivers 
        self.kit_1 = ServoKit(channels=16)
        self.kit_2 = ServoKit(channels=16, address=0x41)
        self.drivers = [self.kit_1, self.kit_2]
        print('Initialized the drivers...')  
      
        # Set ranges and home the system
        self.set_ranges()
        self.home()
        print('Homed the system!')
        print('')

    def set_ranges(self):
        for leg in self.LEGS:
            for joint in leg.joints:
                self.drivers[joint.driver].servo[joint.channel].set_pulse_width_range(joint.minPW, joint.maxPW)

    def home(self):
        for leg in self.LEGS:
            for joint in leg.joints:
                self.drivers[joint.driver].servo[joint.channel].angle = joint.startAng
        sleep(1)
    
    def goTo(self, x, y, z):
        
        for leg in self.LEGS:
            print('Calculating the new positions')
            leg.inverse_k(x,y,z)
        
            # Directly controls the KNEE axis
            self.drivers[leg.joints[1].driver].servo[leg.joints[1].channel].angle = leg.joints[1].goAng + 70

            # Direcly controls the ANKLE axis
            self.drivers[leg.joints[2].driver].servo[leg.joints[2].channel].angle = 180 - leg.joints[2].goAng

            # Directly controls the ROTATE axis
            self.drivers[leg.joints[0].driver].servo[leg.joints[0].channel].angle = leg.joints[0].goAng + leg.rotate_offset
        

