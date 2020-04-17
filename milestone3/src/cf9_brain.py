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
from std_srvs.srv import Empty, EmptyResponse
from darknet_ros_msgs.msg import BoundingBoxes


class CrazyflieBrain():

    def __init__(self):

        self.bounding_boxes_top = "/darknet_ros/bounding_boxes"

        self.classes = ['narrows_from_left', 'no_bicycle', 'residential', 'roundabout']
        # From '/home/robot/dd2419_ws/src/darknet_ros/darknet_ros/config/yolo-lite-cf9-4classes.yaml'

        # Set up checkpoint clearing service
        rospy.wait_for_service('clearpointservice')

        # Initialize callback variables
        self.boxes = None

        # Initiali
        self.goal_pub = rospy.Publisher("goal", Position, queue_size=10)


        # Initialize subscriber to bounding box
        rospy.Subscriber(self.bounding_boxes_top, BoundingBoxes, self.detection_cb)


    def detection_cb(self, msg):
        # # Get timestamp from message and image
        # self.img_time = msg.image_header.stamp.secs + (10**-9)*msg.image_header.stamp.nsecs # Time image was actually taken
        # self.header_time = msg.header.stamp.secs + (10**-9)*msg.header.stamp.nsecs # Time reading is received
        # Read boxes
        self.boxes = msg.bounding_boxes



    def state_machine(self):
        """
        Process:
            X Send a goal pose directly above starting position (already done in hover node)

            Call path planning service (from current to offset from target sign)
            Call path following service
            X Confirm detection of target sign
            X Call checkpoint clear service

            Repeat for next sign
        """

        """ Wait for detection """
        rate = rospy.Rate(1)
        saw_sign = False
        while not saw_sign:
            if self.boxes:
                for box in self.boxes:
                    print(box)
                    if box.Class == 'roundabout':
                        saw_sign = True
            self.boxes = None
            rate.sleep()

        """ Call clear checkpoint """
        try:
            checkpoint = rospy.ServiceProxy('clearpointservice', Empty)
            checkpoint()
        except rospy.ServiceException as e:
            print ("Service call failed: %s" % e)

        """ Fix odometry """
        # Send goal to look at marker


        rospy.sleep(1) # Wait br



if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('cf9_brain', anonymous=True)
    # Initialize brain class
    cfb = CrazyflieBrain()

    #rospy.sleep(10) # Wait for hover to initialize and get ready to do stuff

    print("running...")
    # Run until KeyboardInterrupt

    try:
        cfb.state_machine()
    except KeyboardInterrupt:
        print("Shutting down")

