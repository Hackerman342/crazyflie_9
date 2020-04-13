#!/usr/bin/env python

import json
import numpy as np
import rospy
from mapping import Mapping
from std_srvs.srv import SetBool
from nav_msgs.msg import OccupancyGrid



# Start with creating a msg.
map_msg = OccupancyGrid()
map_msg.header.frame_id = 'map'
res = 0.1

# Importing map using the Mapping-class that we created, resolution = 0.1, padding = 2
padding = 2
mapp = Mapping("/home/johna/dd2419_ws/src/crazyflie_9/milestone2/scripts/awesome.world.json", res, padding)
