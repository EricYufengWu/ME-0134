from adafruit_servokit import ServoKit
import time

# Inintialize the servo hat
kit = ServoKit(channels=16)

# Assigning variable names to the working angles of the servo
back_pos = 80
front_pos = 110

# Moving the servo back-and-forth every one second
while True:
	try:
		kit.servo[3].angle = back_pos
		time.sleep(0.8)
		kit.servo[3].angle = front_pos
		time.sleep(0.2)
	except KeyboardInterrupt:
		kit.servo[3].angle = (front_pos + back_pos)/2
		break
