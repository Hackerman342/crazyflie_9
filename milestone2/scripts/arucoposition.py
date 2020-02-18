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
import numpy as np


def arucopose(data):
    print("Sees aruco")
   
    for elm in data.markers:

        cam_aruco = PoseStamped()
        cam_aruco.pose = elm.pose.pose
        cam_aruco.header.frame_id = 'camera_link'
        cam_aruco.header.stamp = rospy.Time()
        _, _, yaw_transformed = euler_from_quaternion((cam_aruco.pose.orientation.x,
                                              cam_aruco.pose.orientation.y,
                                              cam_aruco.pose.orientation.z,
                                              cam_aruco.pose.orientation.w))
         
        if not tfBuffer.can_transform(cam_aruco.header.frame_id, 'cf1/odom', cam_aruco.header.stamp):
            rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % cam_aruco.header.frame_id)
            return

        
        send_transform = tfBuffer.transform(cam_aruco, 'cf1/odom', rospy.Duration(1))
    
        t= TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'cf1/odom'
        t.child_frame_id = '/arucopostion' + str(elm.id)

        t.transform.translation=send_transform.pose.position
        t.transform.rotation=send_transform.pose.orientation            ## Send position of aruco to RQT tree
        br.sendTransform(t)                     

        cam_aruco.pose.position.z=cam_aruco.pose.position.z-0.5     # Redefine z, Get relative pose in order stay half a meter away from it

        Pose_transform = tfBuffer.transform(cam_aruco, 'cf1/odom', rospy.Duration(1) )

        # t.header.stamp = rospy.Time.now()
       
        t.transform.translation=Pose_transform.pose.position
        t.transform.rotation=Pose_transform.pose.orientation 


        
        
        
        

        

        aruco_postion=Position()
        
        aruco_postion.header.frame_id="cf1/odom"
        aruco_postion.x=t.transform.translation.x
        aruco_postion.y=t.transform.translation.y
        aruco_postion.z=t.transform.translation.z
        aruco_postion.yaw=yaw_transformed
        


       
        aruco_position_pub.publish(aruco_postion)
        
        

rospy.init_node('arucoposition')
br = tf2_ros.TransformBroadcaster()
tfBuffer = tf2_ros.Buffer()
tf=tf2_ros.TransformListener(tfBuffer)
aruco_sub=rospy.Subscriber("/aruco/markers", MarkerArray, arucopose)
aruco_position_pub=rospy.Publisher("aruco_position_pose", Position,queue_size=10)

if __name__ == "__main__":
    
  
    rospy.spin()


