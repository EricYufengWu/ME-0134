from adafruit_servokit import ServoKit
import time
import threading
from signal import pause

# Inintialize the servo hat
kit = ServoKit(channels=16)

# Assigning variable names to the working angles of the servo
back_pos = 90
front_pos = 120

def move():
	threading.Timer(1,move).start()
	degree = back_pos
	while degree < front_pos:
		kit.servo[3].angle=degree
		degree += 1
		time.sleep(0.1)
	while degree > back_pos:
		kit.servo[3].angle=degree
		degree -= 1
		time.sleep(0.1)

kit.servo[0].angle=back_pos
move()
pause()
