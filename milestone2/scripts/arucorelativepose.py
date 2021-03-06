#!/usr/bin/env python

import sys
import math
import json

import rospy
import tf2_ros 
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped
from aruco_msgs.msg import MarkerArray
from crazyflie_driver.msg import Position


def goal_callback(msg):
    print("Have goal")
    global goal # Keeps distance to aruco
    goal=msg                    
   



# Initializing Position and goal

Current_Position=Position()
Current_Position.x=0.5 #in real environment
Current_Position.y=0.5
Current_Position.z=0.4
Current_Position.yaw=0
goal=None


if __name__ == '__main__':
   
    
    rospy.init_node('arucorelativepose')
     
    aruco_sub=rospy.Subscriber("aruco_position_pose", Position, goal_callback) # Goal position is the aruco position?
    pos_publish=rospy.Publisher("/cf1/cmd_position", Position,queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        if goal:
            print("goal")
            pos_publish.publish(goal)
            Current_Position.x=goal.x
            Current_Position.y=goal.y
            Current_Position.z=goal.z
            Current_Position.yaw=goal.yaw
            goal=None


        else:
            pos_publish.publish(Current_Position)

        rate.sleep()
