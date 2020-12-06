#hex.py
#from adafruit_servokit import ServoKit
from time import sleep
from Leg import Leg
#import numpy as np

class Joint:
    def __init__(self, driver, channel, minPW, maxPW, startAng, inverted = False):
        self.driver = driver
        self.channel = channel
        self.minPW = minPW
        self.maxPW = maxPW
        self.startAng = startAng
        self.goAng = self.startAng
        self.currAng = startAng #Eric's code
        self.inverted = inverted
        
        self.angle_sweep = []


    def interpolate(self):
        
        # Number of points for accurate resolution
        resolution = 3
        self.angle_sweep = self.generateArray(self.currAng, self.goAng, resolution)
        print(self.angle_sweep)


    def generateArray(self, lwr_bnd, upr_bnd, num_pts):
        headings = [lwr_bnd + x*(upr_bnd-lwr_bnd)/num_pts for x in range(num_pts)]
        return headings

