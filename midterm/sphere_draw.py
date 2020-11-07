'''
Assume you have the inverse kinematics of arobotic arm capable of 3D
positioning. Write code (pythonorMATLAB)that takes two arbitrary 3-dimensional 
points P1[x,y,z]and P2[x,y,z] on a sphere of radius R,and draws a line on the
sphere between them. The code should output a 3D plot showing the 3D 
line. Upload the code file (make sure to include comments) with your midterm 
(we will change both the points and radius andrun it–it must work).
'''
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
from astropy import coordinates
from math import pi

def prompt():
	r = input('radius of sphere: ')
	point1 = input('first point, separate with space: ').split()
	point2 = input('second point, separate with space: ').split()
	for i in range(len(point1)):
		point1[i] = float(point1[i])
		point2[i] = float(point2[i])
	return float(r), point1, point2

# This is the resolution of the line
rez = 100  

# prompt user to enter information
r, A, B = prompt()

# convert cartesian inputs to spherical coordinates for easy processing
sA = coordinates.cartesian_to_spherical(A[0],A[1],A[2])
sB = coordinates.cartesian_to_spherical(B[0],B[1],B[2])
sA = np.asarray([sA[0].value, sA[1].value, sA[2].value])
sB = np.asarray([sB[0].value, sB[1].value, sB[2].value])
# print(sA, sB)

# create incremental spherical coordinates across the two points
rs = np.linspace(sA[0], sB[0], rez)
thetas = np.linspace(sA[2], sB[2], rez)
phis = np.linspace(sA[1], sB[1], rez)
# print("theta; ", thetas)
# print("phi: ", phis)

# Convert back to cartesian
points = []
for p in range(rez):
	cart_coord = coordinates.spherical_to_cartesian(rs[p], phis[p], thetas[p])
	points.append([float(cart_coord[0].value),float(cart_coord[1].value),float(cart_coord[2].value)])

coords = np.asarray(points)

# Format for plotting
xs = coords[:,0]
ys = coords[:,1]
zs = coords[:,2]

# Generate plots
fig = plt.figure()
fig.suptitle('Line on Sphere')
ax = fig.gca(projection='3d')

# Generate a wireframe of the reference sphere
u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
x = np.cos(u)*np.sin(v)
y = np.sin(u)*np.sin(v)
z = np.cos(v)
ax.plot_wireframe(x*r, y*r, z*r)

# Plot the line on the sphere
ax.plot(xs, ys, zs, label='line on sphere', color = 'r')

# Get axis limit based on radius
axis_max = r * 1.2
axis_min = -(r * 1.2)
ax.set_xlim(axis_max, axis_min)
ax.set_ylim(axis_max, axis_min)
ax.set_zlim(axis_max, axis_min)
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.invert_zaxis()

# Show plot!
plt.show()

