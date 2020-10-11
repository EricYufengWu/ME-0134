import csv

a = []
b = []
my_list = [] 

for i in range(len(a)):
	my_list.append([a[i], b[i]])


with open('initials_2.csv', mode='w') as file:
    file_writer = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    for j in range(len(my_list)):
    	file_writer.writerow(my_list[j])

my_list = [] 
coord_list = []

z_draw_height = -220


with open('LetterCoord2.csv') as file:
	coord = csv.reader(file, delimiter=',')
	for row in coord:
		my_list = row


i = 0
while i < len(my_list):
	if my_list[i] == '999':
		print("pen up detected")
		if i == 0:
			coord_list.append([float(my_list[i+1]),float(my_list[i+2]), z_draw_height + 10])
		elif i == len(my_list) - 1:
			coord_list.append([float(my_list[i-2]),float(my_list[i-1]), z_draw_height + 10])
		else:
			coord_list.append([float(my_list[i-2]),float(my_list[i-1]), z_draw_height + 10])
			coord_list.append([float(my_list[i+1]),float(my_list[i+2]), z_draw_height + 10])
		i += 1
		continue
	elif i < len(my_list) - 2:
		coord_list.append([float(my_list[i]),float(my_list[i+1]), z_draw_height])
		i += 2
		continue
	i += 1


def read_csv(filename):
	with open(filename) as csv_file:
		coord = []        # convert csv file into 2D list of xyz coordinates
		line_count = 0    # keep track of number of data
		csv_reader = csv.reader(csv_file, delimiter = ",")
		for row in csv_reader:
			row_data = []
				row_data.append(float(item))
			coord.append(row_data)
			line_count += 1
		print("Processed {} coordinates".format(line_count))
		return coord

def scale_xy(array, max_val):
	array[:,0] *= (50 / max(abs(array[:,0])))
	array[:,1] *= (50 / max(abs(array[:,1])))
	return array


max_val = 50

coord = read_csv('initials_2.csv')



