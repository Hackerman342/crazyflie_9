#!/usr/bin/env python

import json
import numpy as np
from mapping import Mapping
import matplotlib.pyplot as plt
import rospy
from std_srvs.srv import SetBool
from nav_msgs.msg import OccupancyGrid

# Mapping tutorial

# Creating a map object. The main thing with this is that it has a variable called matrix.
# This matrix is the map, where 0 is free space, 1 is wall, 4 is aruco marker and 5 is a road sign
# You can create more interesting maps by creating a new json file with a more interesting layout
# The argument "awesome.world.json" demands that the json file is in the same folder.
mapp = Mapping("/home/johna/dd2419_ws/src/crazyflie_9/worlds_json/crazyflie9_apartment.world.json", 0.1, 0)


# This gives us the map matrix, which we can use to do path planning with.
matrx = mapp.matrix



xc = mapp.x_conv
yc = mapp.y_conv


# Real coordinate: x - x_conv, y - y_conv
# Using your path planner you get (x, y) coordinates. Subtract (x_conv, y_conv) from each coordinate.


markers, signs = mapp.object_poses()

for marker in markers:
    print marker

for sign in signs:
    print sign

# Since the matrix doesn't have negative indices we need to change the axis when plotting the image
# plt.imshow(matrx, extent=[-matrx.shape[1]/2., matrx.shape[1]/2., -matrx.shape[0]/2., matrx.shape[0]/2.])
plt.imshow(matrx)
matrx_indx = np.nonzero(matrx == 1)
# oy = matrx_indx[0].tolist()
# ox = matrx_indx[1].tolist()


plt.show()
print("Stop for check the matrix variable.(USED FOR DEBUG)")
