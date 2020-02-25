#!/usr/bin/env python

import json
import numpy as np
from create_map_service import Mapping
import matplotlib.pyplot as plt
import rospy
from std_srvs.srv import SetBool

# Mapping tutorial

# Creating a map object. The main thing with this is that it has a variable called matrix.
# This matrix is the map, where 0 is free space, 1 is wall, 4 is aruco marker and 5 is a road sign
# You can create more interesting maps by creating a new json file with a more interesting layout
# The argument "awesome.world.json" demands that the json file is in the same folder.
mapp = Mapping("/home/zihan/drone_ws/src/crazyflie_9/milestone2/scripts/awesome.world.json", 0.1, 2)

# This gives us the map matrix, which we can use to do path planning with.
matrx = mapp.matrix

# Since the matrix doesn't have negative indices we need to change the axis when plotting the image
plt.imshow(matrx, extent=[-matrx.shape[1]/2., matrx.shape[1]/2., -matrx.shape[0]/2., matrx.shape[0]/2.])
matrx_indx = np.nonzero(matrx == 1)
oy = matrx_indx[0].tolist()
ox = matrx_indx[1].tolist()

<<<<<<< HEAD
plt.show()
print("Stop for check the matrix variable.(USED FOR DEBUG)")
=======
matrx_indx = np.nonzero(matrx == 1)
oy = matrx_indx[0].tolist()
ox = matrx_indx[1].tolist()



plt.show()

# if __name__ == "__main__":
#     rospy.wait_for_service("create_map")

#     try:
#         map_matrix = rospy.ServiceProxy('create_map', SetBool)
#         resp1 = map_matrix(True)
#         print resp1
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e
>>>>>>> 17780ba83969ff6757e38024a70cc51c83609c07
