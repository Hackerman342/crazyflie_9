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
    global counter
    global x_average
    global y_average
    global z_average
    global roll_average
    global pitch_average
    global yaw_average
    global t_final_odom_update

    for elm in data.markers:
    # if len(data.markers) > 0:
    #     elm = data.markers[0]
        cam_aruco = PoseStamped()
        cam_aruco.pose = elm.pose.pose
        cam_aruco.header.frame_id = 'cf1/camera_link'
        cam_aruco.header.stamp = rospy.Time.now()


        if not tfBuffer.can_transform(cam_aruco.header.frame_id, 'cf1/odom', rospy.Time(0)):
            rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % cam_aruco.header.frame_id)
            return

        print('detected marker', elm.id)

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



        # Difference between real aruco position in map and the position of the aruco after the rotation is performed


        x_add= x_map -  aruco_pos_final.pose.position.x
        y_add= y_map -  aruco_pos_final.pose.position.y
        z_add = z_map -   aruco_pos_final.pose.position.z



        # Set orienttion to same as in t_update_odom_rotation

        qr_0_add=qr[0]
        qr_1_add=qr[1]
        qr_2_add=qr[2]
        qr_3_add=qr[3]

        orientation_add= euler_from_quaternion((qr_0_add,qr_1_add,qr_2_add,qr_3_add))
        x_average.append(x_add)
        y_average.append(y_add)
        z_average.append(z_add)

        roll_average.append(orientation_add[0])
        pitch_average.append(orientation_add[1])
        yaw_average.append(orientation_add[2])


        counter+=1
        if counter > 10:

             #Perform final update of the odometry frame

            #t_final_odom_update=TransformStamped()
            # t_final_odom_update.header.stamp = rospy.Time.now()
            # t_final_odom_update.header.frame_id = "map"
            # t_final_odom_update.child_frame_id = 'cf1/odom'

            x_final=sum(x_average)/len(x_average)
            y_final=sum(y_average)/len(y_average)
            z_final=sum(z_average)/len(z_average)

            t_final_odom_update.transform.translation.x=x_final
            t_final_odom_update.transform.translation.y=y_final
            t_final_odom_update.transform.translation.z=z_final

            roll_final=sum(roll_average)/len(roll_average)
            pitch_final=sum(pitch_average)/len(pitch_average)
            yaw_final=sum(yaw_average)/len(yaw_average)



            final_rotation = quaternion_from_euler(roll_final, pitch_final, yaw_final)


            t_final_odom_update.transform.rotation.x=final_rotation[0]
            t_final_odom_update.transform.rotation.y=final_rotation[1]
            t_final_odom_update.transform.rotation.z=final_rotation[2]
            t_final_odom_update.transform.rotation.w=final_rotation[3]

            print('Updating odom frame')
            # br.sendTransform(t_final_odom_update)


            counter=0

            x_average=[]
            y_average=[]
            z_average=[]

            roll_average=[]
            pitch_average=[]
            yaw_average=[]


############ Begin node here ############

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

rospy.sleep(1)

# t_odom_init=TransformStamped()
# t_odom_init.header.stamp = rospy.Time.now()
# t_odom_init.header.frame_id = "map"
# t_odom_init.child_frame_id = 'cf1/odom'
# t_odom_init.transform.rotation.w = 1
# br.sendTransform(t_odom_init)  # send it so that its visible in rviz, what the crazyflie sees

# wait_to_trans = True
t_final_odom_update = TransformStamped()
t_final_odom_update.header.frame_id = "map"
t_final_odom_update.child_frame_id = 'cf1/odom'
t_final_odom_update.transform.rotation.w = 1


# ADDED HERE!!!!!!!!!!!!!!!!!!!!!
counter=0
x_average=[]
y_average=[]
z_average=[]

roll_average=[]
pitch_average=[]
yaw_average=[]
# ADDED ABOVE!!!!!!!!!!!!!!!!!!!!!

if __name__ == "__main__":

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if goal:
            arucopose(goal)
            goal = None

        # Broadcast most recent averaged transform every loop
        t_final_odom_update.header.stamp = rospy.Time.now()
        br.sendTransform(t_final_odom_update)

        rate.sleep()