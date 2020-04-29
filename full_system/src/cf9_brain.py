#!/usr/bin/env python

import os
import sys
import roslib
import sys
import rospy
import cv2
import numpy as np
import math
from mapping import Mapping

import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path

from std_srvs.srv import Empty, EmptyResponse
from full_system.srv import PathPlanning, PathPlanningResponse

from darknet_ros_msgs.msg import BoundingBoxes
from crazyflie_driver.msg import Position
from crazyflie_driver.srv import GoToRequest, GoToResponse, GoTo


class CrazyflieBrain():

    def __init__(self):

        self.bounding_boxes_top = "/darknet_ros/bounding_boxes"

        self.classes = ['narrows_from_left', 'no_bicycle', 'residential', 'roundabout']
        # From '/home/robot/dd2419_ws/src/darknet_ros/darknet_ros/config/yolo-lite-cf9-4classes.yaml'

        # Set up checkpoint clearing service
        print('waiting for clear checkpoint service')
        rospy.wait_for_service('clearpointservice')
        print('got clear checkpoint service')

        print('waiting for path planning service')
        rospy.wait_for_service('path_planning')
        print('got path planning service')

        # Creating a map object to obtain the poses of the objects in the map.
        # markers and signs are both lists with tuples (name, pose).
        # objects are a dictionary containing both markers and signs where the names are keys and poses are values.
        # self.objects in form [x, y, z, roll, pitch, yaw]
        # NOTE: Roll and pitch are such that yaw is flipped 180 deg and roll & pitch can be ignored
        self.map = Mapping('/home/robot/dd2419_ws/src/crazyflie_9/worlds_json/crazyflie9_apartment.world.json', 0.05, 2)
        self.markers, self.signs, self.objects = self.map.object_poses()

        self.objects['narrows_from_left'] = self.objects.pop('road_narrows_from_left')
        self.objects['roundabout'] = self.objects.pop('roundabout_warning')

        # Signs that the drone has already visited. Should be reset when signs_visited == classes
        self.signs_visited = []

        # Initialize callback variables
        self.boxes = None

        # Initialize tfbuffer
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        # Initialize goal publisher (for interacting with 'hover' node)
        self.goal_pub = rospy.Publisher("goal", Position, queue_size=10)

        # Initialize path publisher (just visualization)
        self.path_pub = rospy.Publisher("path_vis", Path, queue_size=10)

        # Initialize subscriber to bounding box
        rospy.Subscriber(self.bounding_boxes_top, BoundingBoxes, self._detection_cb)

        # Pause for subscription and tf_buffer
        rospy.sleep(5)

    def _detection_cb(self, msg):
        self.boxes = msg.bounding_boxes


    def state_machine(self):
        """
        Process:
            Call path planning service (from current pose to observation pose)
            Follow path following sequence
            Confirm detection of target sign
            Call checkpoint clear service
            Return to observation pose

            Repeat for next sign
        """
        for _ in range(len(self.objects)):
            target_sign = self.get_closest_sign()
            self.obstacle_sequence(target_sign)

        rospy.loginfo('Finished!!!')

    def get_closest_sign(self):
        cf_trans = self.tf_buffer.lookup_transform("map", "cf1/base_link", rospy.Time.now(), rospy.Duration(10))
        current_x = cf_trans.transform.translation.x
        current_y = cf_trans.transform.translation.y

        dist_dict = {}

        """ Calculate the distance """
        for sign in self.classes:
            if sign not in self.signs_visited:
                sign_pose = self.objects[sign]
                sign_x, sign_y = sign_pose[0], sign_pose[1]
                distance = math.hypot(sign_x-current_x, sign_y-current_y)
                dist_dict[sign] = distance

        # The name of the sign to the shortest distance:
        shortest_dist_sign = min(dist_dict, key=dist_dict.get)

        self.signs_visited.append(shortest_dist_sign)

        if self.signs_visited == self.classes:
            self.signs_visited = [shortest_dist_sign]

        return shortest_dist_sign



    def obstacle_sequence(self, sign_class):

        rospy.loginfo('Begin sequence for next road sign')
        print('Sign class: ', sign_class)
        rospy.sleep(3)

        """ Calc observation pose based on sign pose"""
        offset = 0.5 # [m] | How far away to look at sign
        yaw = self.objects[sign_class][5] # rpy

        observe_pose = Position()
        observe_pose.x = self.objects[sign_class][0] - math.cos(math.radians(yaw))*offset
        observe_pose.y = self.objects[sign_class][1] - math.sin(math.radians(yaw))*offset
        observe_pose.z = 0.5
        observe_pose.yaw = yaw

        """ use tf to get the starting position in the map frame """
        start_pose_ = self.tf_buffer.lookup_transform("map", "cf1/base_link", rospy.Time.now(), rospy.Duration(10))

        """ Define path planning start and end in pixel coords """
        resolution = 0.05
        start_x = int(round(-start_pose_.transform.translation.x/resolution)+100)  # transfer it from meters to pixels
        start_y = int(round(-start_pose_.transform.translation.y/resolution)+100)
        end_x = int(round(-observe_pose.x/resolution)+100)
        end_y = int(round(-observe_pose.y/resolution)+100)

        """ Call path planning service """
        rospy.loginfo('Starting path planning')
        path_planning = rospy.ServiceProxy('path_planning', PathPlanning)
        path = path_planning(start_x, start_y, end_x, end_y)

        """ Get path as """
        pathx = [-(c-100)*resolution for c in path.rx] # In meters, which can be published to the /cf1/cmd_position
        pathy = [-(c-100)*resolution for c in path.ry] # In meters, which can be published to the /cf1/cmd_position
        path_yaw = []
        for i in range(len(pathx)-1):
            yaw = math.degrees(math.atan2(pathy[i+1]-pathy[i],pathx[i+1]-pathx[i]))
            path_yaw.append(yaw)
        path_yaw.append(observe_pose.yaw)

        rospy.loginfo('Finished path planning')

        """ Build path msg and visualize in RVIZ """
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        poses = []
        for i in range(len(pathx)):
            pose = PoseStamped()
            # pose.header.frame_id = 'map'
            pose.pose.position.x = pathx[i]
            pose.pose.position.y = pathy[i]
            pose.pose.position.z = 0.5

            poses.append(pose)
        path_msg.poses = poses
        path_msg.header.stamp = rospy.Time.now()
        self.path_pub.publish(path_msg)


        """
        Add rotation to path direction before beginning path
        """

        """ Follow the path """
        path_pose = Position()
        path_pose.z = 0.5

        path_rate = rospy.Rate(1)
        for i in range(len(pathx)):
            path_pose.x = pathx[i]
            path_pose.y = pathy[i]
            path_pose.yaw = path_yaw[i]

            self.goal_pub.publish(path_pose)
            rospy.loginfo('Sent next path pose')

            # Republish path for visualization
            path_msg.header.stamp = rospy.Time.now()
            self.path_pub.publish(path_msg)

            path_rate.sleep()
        rospy.sleep(2)

        self.goal_pub.publish(observe_pose)
        rospy.loginfo('Confirm at end of path - 1x')
        rospy.sleep(3)
        self.goal_pub.publish(observe_pose)
        rospy.loginfo('Confirm at end of path - 2x')
        rospy.sleep(3)

        rospy.loginfo("Made it to pose for observing sign")


        """ Wait for detection """
        self.boxes = None
        rate = rospy.Rate(1)
        rospy.loginfo("Start looking for road sign detections")
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
            rospy.loginfo('Starting clear checkpoint sequence')
            checkpoint = rospy.ServiceProxy('clearpointservice', GoTo)
            checkpoint(clear_pose)
            rospy.loginfo('Checkpoint cleared')
        except rospy.ServiceException as e:
            print ("Service call failed: %s" % e)

        rospy.sleep(3)


        """ Fix odometry """
        self.goal_pub.publish(observe_pose)
        rospy.loginfo("Correct pose to observation pose - 1x")
        rospy.sleep(3)
        self.goal_pub.publish(observe_pose)
        rospy.loginfo("Correct pose to observation pose - 2x")
        rospy.sleep(3)



if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('cf9_brain', anonymous=True)
    # Initialize brain class
    cfb = CrazyflieBrain()

    print("running...")

    # Run until KeyboardInterrupt
    try:
        cfb.state_machine()
    except KeyboardInterrupt:
        print("Shutting down")

