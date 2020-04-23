#!/usr/bin/env python

import json
import numpy as np
import rospy
import math
from mapping import Mapping
from std_srvs.srv import SetBool
from nav_msgs.msg import OccupancyGrid
from mapping import Mapping

# Imagine map between (-5, -5) to (5, 5) with a straight line between (-2, -2) to (2, 2) as an obstacle

def data_info():
    pass




def mappublisher(height, width, resolutotion, map_origin, map_data):
    msg = OccupancyGrid()
    #map_data = Mapping("/home/johna/dd2419_ws/src/crazyflie_9/worlds_json/crazyflie9_apartment.world.json", resolution, 0)
    msg.header.frame_id = 'map'
    msg.header.stamp = rospy.Time.now()
    msg.info.resolution = resolutotion
    msg.info.width      = math.ceil(width/resolutotion)
    msg.info.height     = math.ceil(height/resolutotion)
    msg.info.origin.position.x = -map_origin[0]
    msg.info.origin.position.y = -map_origin[1]
    data = [0] * int(width/resolutotion) * int(height/resolutotion)
    # matrx = np.rot90(map_data.matrix)

    matrx = map_data.matrix
    # matrx = np.rot90(np.rot90(np.rot90(matrx)))
    # matrx = np.rot90(matrx, 3)

    idx = 0
    #for row in map_data.matrix:
    for row in matrx:
        for point in row:
            if point == 1:
                data[idx] = 100
            else:
                data[idx] = 0
            idx += 1

    msg.data = data
    # print(msg.data)
    mappub.publish(msg)

if __name__ == "__main__":
    rospy.init_node('occupancy_node')
    mappub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
    rate = rospy.Rate(10)


    resolution = 0.05
    map_data = Mapping("/home/robot/dd2419_ws/src/crazyflie_9/worlds_json/crazyflie9_apartment.world.json", resolution, 0)

    end_coordinates, start_coordinates = map_data.airspace()
    height = abs(start_coordinates[0]) + abs(end_coordinates[0])
    width = abs(start_coordinates[1]) + abs(end_coordinates[1])
    map_origin = [abs(start_coordinates[0]), abs(start_coordinates[1])]


    # rate = rospy.Rate(10)
    # height, width, resolution = 10, 10, 0.05
    # map_origin = [width/2, height/2]

    while not rospy.is_shutdown():
        mappublisher(height, width, resolution, map_origin, map_data)
        rate.sleep()

# Importing map using the Mapping-class that we created, resolution = 0.1, padding = 2
# padding = 2
# mapp = Mapping("/home/johna/dd2419_ws/src/crazyflie_9/milestone2/scripts/awesome.world.json", resolution, padding)
