import csv

a = []
b = []
my_list = [] 

for i in range(len(a)):
	my_list.append([a[i], b[i]])

with open('drawing.csv', mode='w') as file:
    file_writer = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    for j in range(len(a)):
    	file_writer.writerow(my_list[j])