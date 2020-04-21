#!/usr/bin/env python

import rospy
from milestone3.srv import PathPlanning, PathPlanningResponse
# from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
# from beginner_tutorials.msg import Position
from find_path import *

def handle_path_planning(req): # req means request
    rx,ry = find_path(req.start_x, req.start_y, req.end_x, req.end_y)
    return PathPlanningResponse(rx, ry)

def path_planning_server():
    rospy.init_node('path_planning_server')
    s = rospy.Service('path_planning', PathPlanning, handle_path_planning)
    print ("Ready to do the path planning.")
    rospy.spin()

if __name__ == "__main__":
    path_planning_server()
