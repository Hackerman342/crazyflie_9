#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply

from geometry_msgs.msg import PoseStamped, TransformStamped
from crazyflie_driver.msg import Position
# from Astar import Astar
import matplotlib.pyplot as plt
from aruco_msgs.msg import MarkerArray
import numpy as np
from tf.transformations import *
import json

def goal_callback(msg):
    global goal
    goal = msg


def arucopose(data):


    # for elm in data.markers:
    if len(data.markers) > 0:
        elm = data.markers[0]
        cam_aruco = PoseStamped()
        cam_aruco.pose = elm.pose.pose
        cam_aruco.header.frame_id = 'cf1/camera_link'
        cam_aruco.header.stamp = rospy.Time.now()


        if not tfBuffer.can_transform(cam_aruco.header.frame_id, 'cf1/odom', rospy.Time(0)):
            rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % cam_aruco.header.frame_id)
            return

        print('detected marker', elm.id, 'Updating odom frame')

        send_transform = tfBuffer.transform(cam_aruco, 'cf1/odom', rospy.Duration(0.5))

        t= TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'cf1/odom'
        t.child_frame_id = 'arucopostion' + str(elm.id)

        t.transform.translation=send_transform.pose.position
        t.transform.rotation=send_transform.pose.orientation

        # t gives the position of the aruco in the odometry frame
        br.sendTransform(t)  # send it so that its visible in rviz, what the crazyflie sees

        # Fix coordinates to arucos based on ID from map here
        # -------------------------------

        x_map=aruco_positions[elm.id][0]
        y_map=aruco_positions[elm.id][1]
        z_map=aruco_positions[elm.id][2]

        roll_map=math.radians(aruco_positions[elm.id][3])   #Transform to radians since degrees is given in map json
        pitch_map=math.radians(aruco_positions[elm.id][4])
        yaw_map=math.radians(aruco_positions[elm.id][5])

        map_orientation = quaternion_from_euler(roll_map, pitch_map, yaw_map) # Transform to quaternion in order to take difference between where aruco is in map and where crazyflie sees it.
                                                                   # Goal is to move odometry so that base_link starts where it supposed to in map.

        #-----------------------------------

        # --------- Take the difference of the orientations between map and odometry frame in order to update the orientation.----------------

        q1_inv = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, -t.transform.rotation.w ]  # negate last element for inverse

        qr = quaternion_multiply(map_orientation, q1_inv) # (currentpose, previouspose)


        # New transformstamped in order to see result of transformation

        t_update_odom_rotation= TransformStamped()
        t_update_odom_rotation.header.stamp = rospy.Time.now()
        t_update_odom_rotation.header.frame_id = 'cf1/odom'
        t_update_odom_rotation.child_frame_id = "rotation_odometry"
        t_update_odom_rotation.transform.rotation.x=qr[0]
        t_update_odom_rotation.transform.rotation.y=qr[1]
        t_update_odom_rotation.transform.rotation.z=qr[2]
        t_update_odom_rotation.transform.rotation.w=qr[3]


        # sendtransform to see result in RVIZ of the rotated odometry frame
        # rospy.loginfo(t_update_odom_rotation)
        br.sendTransform(t_update_odom_rotation)
        rospy.sleep(0.1)


        #Set point position of aruco in the new rotated frame
        t_aruco_rotation=PoseStamped()
        t_aruco_rotation.header.stamp = t_update_odom_rotation.header.stamp
        t_aruco_rotation.header.frame_id = t_update_odom_rotation.child_frame_id
        t_aruco_rotation.pose.position = t.transform.translation
        t_aruco_rotation.pose.orientation.w = 1


        #Get position of aruco marker after rotation in the original odometry frame
        aruco_pos_final = tfBuffer.transform(t_aruco_rotation,'cf1/odom', rospy.Duration(1))
        # aruco_pos_after_rotation = tfBuffer.lookup_transform('cf1/odom',  t_aruco_rotation.child_frame_id , rospy.Duration(0.5) ) 

        #Perform final update of the odometry frame

        t_final_odom_update=TransformStamped()
        t_final_odom_update.header.stamp = rospy.Time.now()
        t_final_odom_update.header.frame_id = "map"
        t_final_odom_update.child_frame_id = 'cf1/odom'

        # Difference between real aruco position in map and the position of the aruco after the rotation is performed

        t_final_odom_update.transform.translation.x = x_map -  aruco_pos_final.pose.position.x
        t_final_odom_update.transform.translation.y = y_map -  aruco_pos_final.pose.position.y
        t_final_odom_update.transform.translation.z = z_map -   aruco_pos_final.pose.position.z



        # Set orienttion to same as in t_update_odom_rotation

        t_final_odom_update.transform.rotation.x=qr[0]
        t_final_odom_update.transform.rotation.y=qr[1]
        t_final_odom_update.transform.rotation.z=qr[2]
        t_final_odom_update.transform.rotation.w=qr[3]

        br.sendTransform(t_final_odom_update)



rospy.init_node('Localization')

# Code in order to get aruco positions from json map

#-----------------------------------------------------------------------------------
map= "/home/robot/dd2419_ws/src/crazyflie_9/worlds_json/crazyflie9_apartment.world.json" #path to where map json file  is located.

with open(map) as json_file:
    data = json.load(json_file)

aruco_markers  = data['markers'] # Get aruco markers positions
aruco_positions={} #Create dictionary
for elements in aruco_markers:

    position=elements["pose"]["position"]
    orientation=elements["pose"]["orientation"]
    aruco_positions[elements["id"]] = [position[0],position[1],position[2], orientation[0], orientation[1], orientation[2]]

#-----------------------------------------------------------------------------------

br = tf2_ros.TransformBroadcaster()
tfBuffer = tf2_ros.Buffer()
tf=tf2_ros.TransformListener(tfBuffer)
aruco_sub=rospy.Subscriber("/aruco/markers", MarkerArray, goal_callback) # Sees aruco -> go to aurocopose function
goal = None

if __name__ == "__main__":

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if goal:
            arucopose(goal)
            goal = None
        rate.sleep()
