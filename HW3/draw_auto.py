from DeltaRobot import DeltaKinematics, DeltaMotion, read_csv, scale_xy
from adafruit_servokit import ServoKit
import math, time
import csv
import numpy as np

max_val = 50
z_draw_height = -220
z_offset = 5

delta = DeltaKinematics()
bot = DeltaMotion(c_down = 36)

### Prompt user to enter name of coordinates file to read
while 1:
	try:
		filename = input('filename to read: ')
		coord = read_csv(filename)
		break
	except:
		continue

### Prompt user to place paper, then clamp it down with side servos
bot.clamp_up()
cmd = input('place a piece of paper on the workspace, press ENTER when done: ')
bot.clamp_down()
time.sleep(1)

### Add a Z coordinate if the input csv only contains xy
# if len(coord[0]) == 2:
# 	for item in coord:
# 		item.append(z_draw_height)

### Converts to numpy array for easy manipulation
coord = np.asarray(coord)

### Scales the coordinate for legibility - if needed
# coord = scale_xy(coord, max_val)

### Adjust Z height for better writing quality
coord[:,2] += z_offset

### Feed the coordinates one-by-one to the bot to draw out the desired pattern
for points in coord:
	bot.move_robot(delta.inverse_kinematics(points[0], points[1], points[2]))
	# print(points)
	time.sleep(0.1)

### Finish. Now perform a little dance?
time.sleep(1)
bot.move_all(30)
bot.clamp_up()
