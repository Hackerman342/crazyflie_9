#!/usr/bin/env python

import os
import sys
import roslib
import sys
import rospy
import cv2
import numpy as np
import math

import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import TransformStamped, PoseStamped

from darknet_ros_msgs.msg import BoundingBoxes


class CrazyflieBrain():

    def __init__(self):

        self.bounding_boxes_top = "/darknet_ros/bounding_boxes"

        self.classes = ['narrows_from_left', 'no_bicycle', 'residential', 'roundabout']
        # From '/home/robot/dd2419_ws/src/darknet_ros/darknet_ros/config/yolo-lite-cf9-4classes.yaml'

        # Set up checkpoint clearing service
        rospy.wait_for_service('clearpointservice')


        # Initialize subscriber to bounding box
        rospy.Subscriber(self.bounding_boxes_top, BoundingBoxes, self.detection_cb)




    def state_machine(self):
        """
        Process:
            Send a goal pose directly above starting position

            Call path planning service (from current to offset from target sign)
            Call path following service
            Confirm detection of target sign
            Call checkpoint clear service

            Repeat for next sign
        """
        pass


if __name__ == "__main__":
  # Initialize ROS node
  rospy.init_node('cf9_brain', anonymous=True)
  # Initialize brain class
  cfb = CrazyflieBrain()

  print("running...")
  # Run until KeyboardInterrupt
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

