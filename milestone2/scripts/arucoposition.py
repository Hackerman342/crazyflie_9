#!/usr/bin/env python

import sys
import math
import json

import rospy
import tf2_ros 
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped
from aruco_msgs.msg import MarkerArray
from crazyflie_driver.msg import Position
import numpy as np


def arucopose(data):
    
   
    for elm in data.markers:

        cam_aruco = PoseStamped()
        cam_aruco.pose = elm.pose.pose
        cam_aruco.header.frame_id = 'camera_link'
        cam_aruco.header.stamp = rospy.Time()

         
        if not tfBuffer.can_transform(cam_aruco.header.frame_id, 'cf1/odom', cam_aruco.header.stamp):
            rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % cam_aruco.header.frame_id)
            return

        
        send_transform = tfBuffer.transform(cam_aruco, 'cf1/odom', rospy.Duration(1))
    
        t= TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'cf1/odom'
        t.child_frame_id = 'arucopostion' + str(elm.id)
        

        t.transform.translation=send_transform.pose.position
        t.transform.rotation=send_transform.pose.orientation
        br.sendTransform(t)             ## Send position of aruco to RQT tree      

        quaternion=quaternion_from_euler(-1.54, 0, -1.54)
        

        aruco_pos= PoseStamped()
        aruco_pos.pose.position.y =0.5
        aruco_pos.pose.orientation.x = quaternion[0]
        aruco_pos.pose.orientation.y = quaternion[1]
        aruco_pos.pose.orientation.z = quaternion[2]
        aruco_pos.pose.orientation.w = quaternion[3]
        aruco_pos.header.frame_id = 'arucopostion' + str(elm.id)
        aruco_pos.header.stamp=rospy.Time()
        
        # aruco_pos.header.stamp = rospy.Time.now()

       
        
         # Transformation of the 
        # cam_aruco.pose.position.z=cam_aruco.pose.position.z-0.5 #Redefine z, Get relative pose in order stay half a meter away from it
        

        Pose_transform = tfBuffer.transform(aruco_pos, 'cf1/odom', rospy.Duration(1) )

        _, _, yaw_transformed = euler_from_quaternion((Pose_transform.pose.orientation.x ,
                                              Pose_transform.pose.orientation.y,
                                              Pose_transform.pose.orientation.z,
                                              Pose_transform.pose.orientation.w))
        yaw_transformed = round((180./np.pi)*yaw_transformed)
       

        aruco_position=Position()

        # aruco_position.header.frame_id="cf1/odom"
        aruco_position.x=Pose_transform.pose.position.x
        aruco_position.y=Pose_transform.pose.position.y
        aruco_position.z=Pose_transform.pose.position.z
        
        aruco_position.yaw=yaw_transformed

    
        aruco_position_pub.publish(aruco_position)
        
        

rospy.init_node('arucoposition')
br = tf2_ros.TransformBroadcaster()
tfBuffer = tf2_ros.Buffer()
tf=tf2_ros.TransformListener(tfBuffer)
aruco_sub=rospy.Subscriber("/aruco/markers", MarkerArray, arucopose)
aruco_position_pub=rospy.Publisher("aruco_position_pose", Position,queue_size=10)

if __name__ == "__main__":
    
  
    rospy.spin()


