from DeltaRobot import DeltaKinematics, DeltaMotion
from adafruit_servokit import ServoKit
import math, time
import csv
import numpy as np

def read_csv(filename):
	with open(filename) as csv_file:
		coord = []        # convert csv file into 2D list of xyz coordinates
		line_count = 0    # keep track of number of data
		csv_reader = csv.reader(csv_file, delimiter = ",")
		for row in csv_reader:
			row_data = []
			for item in row:
				row_data.append(float(item))
			coord.append(row_data)
			line_count += 1
		print("Processed {} coordinates".format(line_count))
		return coord

def save_csv(filename, array):
	return

def scale_xy(array, max_val):
	array[:,0] *= (50 / max(abs(array[:,0])))
	array[:,1] *= (50 / max(abs(array[:,1])))
	return array

z_draw_height = -200
max_val = 50

delta = DeltaKinematics()
bot = DeltaMotion()

while 1:
	try:
		filename = input('filename to read: ')
		coord = read_csv(filename)
		break
	except:
		continue

if len(coord[0]) == 2:
	for item in coord:
		item.append(z_draw_height)

coord = np.asarray(coord)
coord = scale_xy(coord, max_val)

for points in coord:
	bot.move_robot(delta.inverse_kinematics(points[0], points[1], points[2]))
	print(points)
	time.sleep(0.2)

time.sleep(1)
bot.move_all(0)

