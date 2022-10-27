from numpy import pi, cos, sin, arccos, arange
import mpl_toolkits.mplot3d
import matplotlib.pyplot as pp

num_pts = 1000
indices = arange(0, num_pts, dtype=float) + 0.5

phi = arccos(1 - 2*indices/num_pts)
theta = pi * (1 + 5**0.5) * indices

x, y, z = cos(theta) * sin(phi), sin(theta) * sin(phi), cos(phi)
x = x * 20
y = y * 20
z = z * 20

pp.figure().add_subplot(111, projection='3d').scatter(x, y, z)
pp.show()