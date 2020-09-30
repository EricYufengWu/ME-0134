# This code is modified from the original "CustomGamepadExample.py" from the Gamepad library.
# Specifically re-designed to work with: https://www.amazon.com/gp/product/B00IR3U03Y/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1

import sys
sys.path.insert(0, "/home/pi/Gamepad")

import Gamepad
import time
from bluedot.btcomm import BluetoothClient

# Press ENTER without typing a name to get raw numbers for each
# button press or axis movement, press CTRL+C when done
class CustomGamepad(Gamepad.Gamepad):
    def __init__(self, joystickNumber = 0):
        Gamepad.Gamepad.__init__(self, joystickNumber)
        self.axisNames = {
            0: 'LEFT-X',
            1: 'LEFT-Y',
            2: 'RIGHT-Y',
            3: 'RIGHT-X',
            4: 'DPAD-X',
            5: 'DPAD-Y'
        }
        self.buttonNames = {
            0:  '1',
            1:  '2',
            2:  '3',
            3:  '4',
            4:  'L1',
            5:  'R1',
            6:  'L2',
            7:  'R2',
            8:  'SELECT',
            9:  'START',
            10: 'L3',
            11: 'R3'
        }
        self._setupReverseMaps()

# Gamepad settings
gamepadType = CustomGamepad
if not Gamepad.available():
    print('Please connect your gamepad...')
    while not Gamepad.available():
        time.sleep(1.0)
gamepad = gamepadType()
print('Gamepad connected')

# Set initial states for the analog axis
left_x = 0.0
left_y = 0.0
right_x = 0.0
right_y = 0.0

# Bluedot Setup
def data_received(data):
    print(data)

c = BluetoothClient("raspberrypi", data_received)

# Handle joystick updates one at a time
while gamepad.isConnected():
    # Wait for the next event
    eventType, control, value = gamepad.getNextEvent()

    # Determine the type and send data to the server Raspberry Pi
    if eventType == 'BUTTON':
        # Button changed
        # print('Button: {} State: {}'.format(control, value))
        c.send('B {} {}'.format(control, value))
    elif eventType == 'AXIS':
        # Joystick changed
        if control == 'LEFT-X':
            left_x = value
        elif control == 'LEFT-Y':
            value = -value
            left_y = value
        elif control == 'RIGHT-X':
            right_x = value
        elif control == 'RIGHT-Y':
            value = -value
            right_y = value
        # print('left_x: %+.1f %%, left_y: %+.1f %%, right_y: %+.1f %%, right_y: %+.1f %%,' % (left_x * 100, left_y * 100, right_x * 100, right_y * 100))
        c.send('A {} {} '.format(control, value))
