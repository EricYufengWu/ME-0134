# main_controlled.py
# This script maps the buttons the switch controller to
# the manual operation of the hexapod. 

# bluetoothctl
# devices - if you see joycon
# hold pair button
# connect to mac address with "connect "mac""


import time
from Hex import Hex
from evdev import InputDevice, categorize, ecodes

print('Starting the controller......')

gamepad = InputDevice('/dev/input/event0')


# Button assignment of the Joycon controller
yBtn = 307
bBtn = 306
xBtn = 305
aBtn = 304
slBtn = 308
srBtn = 309
homeBtn = 316
plusBtn = 313


#Establishing the mode of the controller
isLow = True

hex = Hex()

print('Controller is ready, lets go!')
for event in gamepad.read_loop():
    if event.type == ecodes.EV_KEY:
        
        #Forward()
        if event.code == yBtn:
            if event.value is 1 and isLow:
                print('forward_low')
                hex.step_high()
            elif event.value is 1:
                print('forward_high')
        
        #Left()
        elif event.code == bBtn:
            if event.value is 1 and isLow:
                print('left_low')
                hex.rotate_left()
            elif event.value is 1:
                print('left_high')

        #Right()
        elif event.code == xBtn:
            if event.value is 1 and isLow:
                print('right_low')
                hex.rotate_right()
            elif event.value is 1:
                print('right_high')
            
        #Backward()
        elif event.code == aBtn:
            if event.value is 1 and isLow:
                print('backward_low')
            elif event.value is 1 :
                print('backward_high')
        
        #low_mode()
        elif event.code == slBtn:
            if event.value is 1:
                isLow = True
                print('low_mode')
        
        #high_mode()
        elif event.code == srBtn:
            if event.value is 1:
                isLow = False
                print('high_mode')
        
        #wall_mode()
        elif event.code == homeBtn:
            if event.value is 1:
                print('home_mode')
        
        #sweep_mode()
        elif event.code == plusBtn:
            if event.value is 1:
                print('issuing a sweep')
                hex.home()

        time.sleep(.1)
        print('-------')
