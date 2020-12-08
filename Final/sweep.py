from adafruit_vl53l0x import VL53L0X
import time
import board
from math import cos, sin, pi
import numpy as np
from adafruit_servokit import ServoKit

def sweep(sensors, kit):
    '''Sweeping function that calculates the distance based a fixed rotation
    
        Robot should be stationary for the mapping 
        
    '''
    # Establish bound for generating an angle sweep
    lower = 60
    upper = 120
    resolution = 10
    
    # Generate an array of headings
    headings = np.linspace(upper, lower, resolution)
    
    # Generate all of the (x,y) points for the map
    readings =[]
    
    # Generate all of the points 
    for heading in headings:
        kit.servo[0].angle = heading
        time.sleep(.3)
        # Servo operation code should go here
        readings.append(sensors[0].range)
    return readings


kit = ServoKit(channels = 16)
kit.servo[0].set_pulse_width_range(400,2600)
    
i2c = board.I2C()

sensors = []
sensors.append(VL53L0X(i2c))
readings = sweep(sensors, kit)

print('TOF readings: ', readings)
