import numpy as np
import matplotlib.pyplot as plt
import yaml
import math

def lemniscate(a, b):
  t = np.linspace(0, 2*np.pi, 100)
  x = a * np.sqrt(2) * np.cos(t) / (np.sin(t)**2 + 1) + 1.5
  y = b * np.sqrt(2) * np.cos(t) * np.sin(t) / (np.sin(t)**2 + 1)
  yaw = get_yaw_angle(x,y)
  dx=[]
  dy=[]
  for i in range(len(x)):
    dx+=[math.cos(yaw[i])]
    dy+=[math.sin(yaw[i])]
  return x, y, yaw, dx, dy

def get_yaw_angle(x, y):
    # numerical computation of yaw angle using a line through the current and
    # the next waypoint
    # this works for any continous curve
    yaw = [0] * len(x)
    for i in range(len(x) - 1):
        yaw[i] = np.arctan2(y[i + 1] - y[i], x[i + 1] - x[i])
    # assuming that the next waypoint of the last waypoint is the first waypoint
    yaw[-1] = np.arctan2(y[0] - y[-1], x[0] - x[-1])

    return yaw

def xyz_to_dict_list(x, y, yaw):
    path = []
    for x_, y_, yaw_ in zip(x, y, yaw):
        path.append(dict(x=float(x_), y=float(y_), yaw=float(yaw_)))
    return path

a = 1.3
b1 = 3
b2 = 2.25
b3 = 1.5
x, y1, yaw1, dx1, dy1 = lemniscate(a, b1)
x, y2, yaw2, dx2, dy2  = lemniscate(a, b2)
x, y3, yaw3, dx3, dy3  = lemniscate(a, b3)

x = list(x)  # convert x to a list
y1 = list(y1)  # convert y to a list
dx1 = list(dx1)  # convert dx to an array
dy1 = list(dy1)  # convert dy to an array
# print(x)
# Create a dictionary to hold the setpoints
setpoints = xyz_to_dict_list(x,y1,yaw1)
# print(setpoints)
# Add the setpoints to the dictionary
# for i in range(len(x)):
#   setpoints['setpoints'].append({'x': x[i], 'y': y1[i]})

# Write the dictionary to a YAML file
# print(setpoints)
with open('setpoints.yaml', 'w') as outfile:
  yaml.dump(setpoints, outfile)



for i in range(len(x)):
    plt.arrow(x[i], y1[i], dx1[i]/5, dy1[i]/5, head_width=0.05, head_length=0.05, fc='k', ec='k')

# plt.arrow(x, y1, dx1, dy1,head_width=0.5, head_length=0.5, fc='k', ec='k' )
plt.plot(x, y1)
plt.plot(x, y2)
plt.plot(x, y3)
plt.axis('equal')
plt.show()
