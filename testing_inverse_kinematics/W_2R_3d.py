from cmath import cos
import numpy as np
import modern_robotics as mr
import math as m
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

L1 = 0.07
L2 = 0.115
L3 = 0.095
L4 = 0.07
robot_origin = [0 , 0 , 0]

# theta 1 is now horizontal.
# 
#
#           _____ {claw}
#          / theta 4
#         /
#        / 
#   ----- theta 3
#  /
# / theta 2
# |                                   ^
# | {space}                           |
#     directions:   x: --> , y: x, z: |

desired_x = 0.1
desired_y = -0.1
desired_z = 0.02 # elevation
desired_end_angle = -90 * (np.pi/180)

theta1 = np.arctan2(desired_y,desired_x)

desired_distance = np.sqrt(desired_x**2 + desired_y**2)
print('desired dist', desired_distance)

desired_distance_actual = desired_distance - L4*np.cos(desired_end_angle) #- np.sqrt(robot_origin[0]**2 + robot_origin[1]**2)
desired_elevation_actual = desired_z - L4*np.sin(desired_end_angle) - L1
print('actual desired',desired_distance_actual,desired_elevation_actual)
#'''

'''
desired_x_actual = desired_x - L4*np.cos(desired_end_angle)*np.cos(theta1) - robot_origin[0]
desired_y_actual = desired_y - L4*np.cos(desired_end_angle)*np.sin(theta1) - robot_origin[1]
desired_z_actual = desired_z - L4*np.sin(desired_end_angle) - robot_origin[2]

desired_distance_actual = np.sqrt(desired_x_actual**2 + desired_y_actual**2)
desired_elevation_actual = desired_z_actual
#'''

# using lecture slides, treat as 2d problem
plus_or_minus = -1
costheta3 = (desired_distance_actual**2 + desired_elevation_actual**2 - L2**2 - L3**2) / (2*L2*L3)
theta3 = m.atan2(plus_or_minus*m.sqrt(1 - costheta3**2),costheta3)
theta2 = m.atan2(desired_elevation_actual,desired_distance_actual) - np.arctan2(L2*np.sin(theta3),L2 + L3*np.cos(theta3))

theta2_abs = theta2
theta3_abs = theta2 + theta3
theta4_abs = desired_end_angle
theta4 =  desired_end_angle - theta2 - theta3

print(theta1, theta2, theta3, desired_end_angle - theta2 - theta3)

# generate line 1 arrays
L1_start = robot_origin
L1_end = [L1_start[0] + 0 , L1_start[1] + 0, L1_start[2] + L1]
x1 = np.linspace(L1_start[0],L1_end[0],num=50)
y1 = np.linspace(L1_start[1],L1_end[1],num=50)   
z1 = np.linspace(L1_start[2],L1_end[2],num=50)  


# generate line 2 arrays
L2_start = [L1_end[0],L1_end[1], L1_end[2]]
L2_end = [L2_start[0] + L2*np.cos(theta2_abs)*np.cos(theta1) , L2_start[1] + L2*np.cos(theta2_abs)*np.sin(theta1) , L2_start[2] + L2*np.sin(theta2_abs)]
x2 = np.linspace(L2_start[0],L2_end[0],num=50)
y2 = np.linspace(L2_start[1],L2_end[1],num=50)   
z2 = np.linspace(L2_start[2],L2_end[2],num=50)

# generate line 3 arrays
L3_start = [L2_end[0],L2_end[1],L2_end[2]]
L3_end = [L3_start[0] + L3*np.cos(theta3_abs)*np.cos(theta1) , L3_start[1] + L3*np.cos(theta3_abs)*np.sin(theta1), L3_start[2] + L3*np.sin(theta3_abs)]
x3 = np.linspace(L3_start[0],L3_end[0],num=50)
y3 = np.linspace(L3_start[1],L3_end[1],num=50)
z3 = np.linspace(L3_start[2],L3_end[2],num=50)

# generate line 4 arrays
L4_start = [L3_end[0],L3_end[1], L3_end[2]]
L4_x_distance = L4*np.cos(theta4_abs)
L4_y_distance = L4*np.sin(theta4_abs)
L4_end = [L4_start[0] + L4*np.cos(theta4_abs)*np.cos(theta1) , L4_start[1] + L4*np.cos(theta4_abs)*np.sin(theta1), L4_start[2] + L4*np.sin(theta4_abs)]
x4 = np.linspace(L4_start[0],L4_end[0],num=50)
y4 = np.linspace(L4_start[1],L4_end[1],num=50)
z4 = np.linspace(L4_start[2],L4_end[2],num=50)

print(L2*np.cos(theta2) + L3*np.cos(theta2 + theta3) , L2*np.sin(theta2) + L3*np.sin(theta2 + theta3))

# HAVE TO PLOT 3D

plt.rcParams["figure.figsize"] = [7.00, 3.50]
plt.rcParams["figure.autolayout"] = True
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x1, y1, z1, alpha=1,s=7,c='r')
ax.scatter(x2, y2, z2, alpha=1,s=7,c='b')
ax.scatter(x3, y3, z3, alpha=1,s=7,c='g')
ax.scatter(x4, y4, z4, alpha=1,s=7,c='purple')
ax.axes.set_xlim3d(left=-0.25, right=0.25) 
ax.axes.set_ylim3d(bottom=-0.25, top=0.25) 
ax.axes.set_zlim3d(bottom=-0.1, top=0.25) 
ax.text3D(L4_end[0], L4_end[1], L4_end[2],"{Claw}")
ax.text3D(robot_origin[0],robot_origin[1],robot_origin[2],"{Space}")
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')

plt.show()

# Plotting
'''
plt.plot(x1,y1)
plt.plot(x2,y2)
plt.plot(x3,y3)
plt.xticks(np.linspace(-.5,1,30))
plt.yticks(np.linspace(-.5,1,30))
print('\n',m.sqrt((desired_x_actual - L2_end[0])**2 + (desired_y_actual - L2_end[1])**2))
plt.xlim([-0.1, 0.5])
plt.ylim([-0.1, 0.5])
plt.show()
#'''