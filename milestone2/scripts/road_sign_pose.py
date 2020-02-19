#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
import math

import tf2_ros 
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped

from darknet_ros_msgs.msg import ObjectCount, BoundingBox, BoundingBoxes
"""
Read a bounding box and object id and determine pose in camera frame from bounding box
"""


class SignPose:

  def __init__(self):
    # Initialize parameters
    #### NOTE: Hardcoding now - will add launch file and parameters later
    self.bounding_boxes_top = '/darknet_ros/bounding_boxes'
    self.cam_frame = 'cf1/camera_link'
    self.odom_frame = 'cf1/odom'

    # From '/home/robot/dd2419_ws/src/course_packages/dd2419_launch/calibration/camera.yaml' or topic: /cf1/camera/camera_info
    #              [    fx    ,     s   ,     x0    ,     0   ,     fy    ,     y0    ,     0   ,     0   ,     1   ]
    self.cam_mat = [231.250001, 0.000000, 320.519378, 0.000000, 231.065552, 240.631482, 0.000000, 0.000000, 1.000000]
    # From '/home/robot/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/______.world.json'
    self.sign_dim = [0.20, 0.20]
    
    
    # Initialize subscribers
    rospy.Subscriber(self.bounding_boxes_top, BoundingBoxes, self.boxes_cb)

    # Initialize tf stuff
    self.br = tf2_ros.TransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    tf=tf2_ros.TransformListener(tfBuffer)
  
  def boxes_cb(self,msg):
    # Get timestamp from message and image
    self.img_time = msg.image_header.stamp.secs + (10**-9)*msg.image_header.stamp.nsecs # Time image was actually taken
    self.header_time = msg.header.stamp.secs + (10**-9)*msg.header.stamp.nsecs # Time reading is received
    # Read boxes 
    self.boxes = msg.bounding_boxes
    # Call pose extraction function
    self.extract_pose()

  def extract_pose(self):
    # Get coordinates of 
    for i in range(len(self.boxes)):
          box = self.boxes[i]
          # Class
          ob_class = box.Class

          # Size
          print(box)
          xsize = box.xmax - box.xmin
          ysize = box.ymax - box.ymin
          pix_area = xsize*ysize
          # Image Center
          xc = (box.xmax + box.xmin)/2
          yc = (box.ymax + box.ymin)/2

          #
          foc_dist = (self.cam_mat[0] + self.cam_mat[4])/2
          sign_area = self.sign_dim[0]*self.sign_dim[1]
          sign_dist2 = foc_dist*math.sqrt(sign_area)/math.sqrt(pix_area)
          sign_distx = foc_dist*self.sign_dim[0]/xsize
          sign_disty = foc_dist*self.sign_dim[1]/ysize
          # Choose which Z calculation to use
          z = sign_distx
          x0 = self.cam_mat[2]
          y0 = self.cam_mat[5]
          x = z*(xc - x0)/foc_dist
          y = z*(yc - y0)/foc_dist

          # Broadcast transform to detected sign
          t = TransformStamped()
          t.header.stamp = rospy.Time.now()
          t.header.frame_id = self.cam_frame
          t.child_frame_id = 'detected/' + str(ob_class)
          
          t.transform.translation.x = x
          t.transform.translation.y = y
          t.transform.translation.z = z
          quat = quaternion_from_euler(-1.54, 0, -1.54)
          t.transform.rotation.x = quat[0]
          t.transform.rotation.y = quat[1]
          t.transform.rotation.z = quat[2]
          t.transform.rotation.w = quat[3]
          print(t)


          self.br.sendTransform(t)

        
          
          

  def read_labels(self, labels_file):
      """
      Returns a list of strings

      Arguments:
      labels_file -- path to a .txt file
      """
      if not labels_file:
          print ('WARNING: No labels file provided. Results will be difficult to interpret.')
          return None

      labels = []
      with open(labels_file) as infile:
          for line in infile:
              label = line.strip()
              if label:
                  labels.append(label)
      assert len(labels), 'No labels found'
      return labels

if __name__ == '__main__':
  # Initialize ROS node
  rospy.init_node('road_sign_pose', anonymous=True)
  # Initialize SignPose class
  sp = SignPose()

  print("running...")
  # Run until KeyboardInterrupt 
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")  
