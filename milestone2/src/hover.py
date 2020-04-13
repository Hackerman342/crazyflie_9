#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position, Hover
import random
import readchar
import numpy as np




def pos_callback(msg):
    global x, y, z, yaw

    x=round(msg.pose.position.x,1)
    y=round(msg.pose.position.y,1)
    z=round(msg.pose.position.z,1)
    quat = msg.pose.orientation
    _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    yaw = round((180./np.pi)*yaw) # Convert yaw to degrees

def goal_callback(msg):
    global goal
    goal=msg
    print(goal)
    # goal=msg
    # goal.x=goal.x
    # goal.y=y+goal.y
    # goal.z=z+goal.z
    # goal.yaw=yaw+goal.yaw

sub = rospy.Subscriber("cf1/pose", PoseStamped, pos_callback)
hover_publisher=rospy.Publisher("/cf1/cmd_position", Position,queue_size=10) #Publishes current height and position
hover_sub=rospy.Subscriber("goal", Position, goal_callback)

# tf_buffer = tf2_ros.Buffer()
# tf2_ros.TransformListener(tf_buffer)



Current_Position=Position()
Current_Position.x=0.5
Current_Position.y=0.5
Current_Position.z=0
Current_Position.yaw=0
Current_Position.header.frame_id = "map"

goal=None
rospy.sleep(2)

if __name__ == '__main__':
    global h, state
    rospy.init_node('hover')
    rospy.loginfo("Successful initilization of 'hover' node")
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():

        if goal:

            # odom_tf = tf_buffer.lookup_transform('')


            print("GOAL!!!")
            hover_publisher.publish(goal)
            Current_Position.x = goal.x
            Current_Position.y = goal.y
            Current_Position.z = goal.z
            Current_Position.yaw = goal.yaw
            goal=None
            rate.sleep()
        else:
            #print("NO GOAL")
            hover_publisher.publish(Current_Position)
        rate.sleep()
    