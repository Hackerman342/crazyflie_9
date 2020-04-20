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
from crazyflie_driver.msg import Position
from crazyflie_driver.srv import GoToRequest, GoToResponse, GoTo

class CrazyflieBrain():

    def __init__(self):

        self.bounding_boxes_top = "/darknet_ros/bounding_boxes"

        self.classes = ['narrows_from_left', 'no_bicycle', 'residential', 'roundabout']
        # From '/home/robot/dd2419_ws/src/darknet_ros/darknet_ros/config/yolo-lite-cf9-4classes.yaml'

        # Set up checkpoint clearing service
        print('waiting for service')
        rospy.wait_for_service('clearpointservice')
        print('got service')

        # Initialize callback variables
        self.boxes = None

        # Initialize goal publisher (for interacting with 'hover' node)
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

        roundabout_pose = Position()
        roundabout_pose.x = 0.0
        roundabout_pose.y = 0.7
        roundabout_pose.z = 0.6
        roundabout_pose.yaw = 0.0

        self.obstacle_sequence('roundabout', roundabout_pose)


        """ Temporary manual path plan between signs"""
        temp_pose = Position()
        roundabout_pose.x = -0.25
        roundabout_pose.y = 0.5
        roundabout_pose.z = 0.6
        roundabout_pose.yaw = 90.0
        self.goal_pub.publish(temp_pose)
        rospy.sleep(2)

        roundabout_pose.x = -0.5
        roundabout_pose.y = 0.4
        roundabout_pose.yaw = 180.0
        self.goal_pub.publish(temp_pose)
        rospy.sleep(5)

        roundabout_pose.x = -0.75
        roundabout_pose.y = 0.4
        self.goal_pub.publish(temp_pose)
        rospy.sleep(2)


        narrow_left_pose = Position()
        narrow_left_pose.x = -0.9
        narrow_left_pose.y = 0.4
        narrow_left_pose.z = 0.6
        narrow_left_pose.yaw = 180.0

        self.obstacle_sequence('narrows_from_left', narrow_left_pose)

        rospy.loginfo('Finished!!!')



    def obstacle_sequence(self, sign_class, observe_pose):
        """ Go to pose """
        # Call path planning and following sequence
        self.goal_pub.publish(observe_pose)
        print('once')
        rospy.sleep(3)
        self.goal_pub.publish(observe_pose)
        print('twice')
        rospy.sleep(3)

        rospy.loginfo("Made it to pose for observing sign")


        """ Wait for detection """
        rate = rospy.Rate(1)
        saw_sign = False
        while not saw_sign:
            if self.boxes:
                for box in self.boxes:
                    print('Seeing', box.Class)
                    if box.Class == sign_class:
                        rospy.loginfo('Confirmed detection of correct sign')
                        saw_sign = True
            self.boxes = None
            rate.sleep()

        """ Call clear checkpoint """
        clear_pose = GoToRequest()

        clear_pose.goal.x = observe_pose.x
        clear_pose.goal.y = observe_pose.y
        clear_pose.goal.z = observe_pose.z
        clear_pose.yaw = observe_pose.yaw

        try:
            checkpoint = rospy.ServiceProxy('clearpointservice', GoTo)
            # rospy.loginfo('Clearing checkpoint')
            checkpoint(clear_pose)
            # rospy.loginfo('Checkpoint cleared')
        except rospy.ServiceException as e:
            print ("Service call failed: %s" % e)

        rospy.sleep(3)


        """ Fix odometry """
        # Send goal to look at marker
        # Call path planning and following sequence
        self.goal_pub.publish(observe_pose)
        print('once')
        rospy.sleep(3)
        self.goal_pub.publish(observe_pose)
        print('twice')
        rospy.sleep(3)



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

