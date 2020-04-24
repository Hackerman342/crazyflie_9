#!/usr/bin/env python

import os
import sys
import roslib
import sys
import rospy
import cv2
import numpy as np
import math
import mapping

import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path

from std_srvs.srv import Empty, EmptyResponse
from milestone3.srv import PathPlanning, PathPlanningResponse

from darknet_ros_msgs.msg import BoundingBoxes
from crazyflie_driver.msg import Position
from crazyflie_driver.srv import GoToRequest, GoToResponse, GoTo

# /home/zihan/dd2419_ws/src/crazyflie_9/worlds_json/crazyflie9_apartment.world.json

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
        self.map = mapping('/home/robot/dd2419_ws/src/crazyflie_9/worlds_json/crazyflie9_apartment.world.json', 0.05, 2)
        self.markers, self.signs, self.objects = self.map.object_poses()
    

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
            X Send a goal pose directly above starting position (already done in hover node)

            Call path planning service (from current to offset from target sign)
            Call path following service
            X Confirm detection of target sign
            X Call checkpoint clear service

            Repeat for next sign
        """

        # roundabout_pose = Position()
        # roundabout_pose.x = 0.0
        # roundabout_pose.y = 0.7
        # roundabout_pose.z = 0.6
        # roundabout_pose.yaw = 0.0

        # self.obstacle_sequence('roundabout', roundabout_pose)

        # narrow_left_pose = Position()
        # narrow_left_pose.x = -0.9
        # narrow_left_pose.y = 0.4
        # narrow_left_pose.z = 0.6
        # narrow_left_pose.yaw = 180.0

        # self.obstacle_sequence('narrows_from_left', narrow_left_pose)


        residential_pose = Position()
        residential_pose.x = 3.5
        residential_pose.y = 0.5
        residential_pose.z = 0.6
        residential_pose.yaw = 270.0

        self.obstacle_sequence('residential', residential_pose)

        rospy.loginfo('Finished!!!')



    def obstacle_sequence(self, sign_class, observe_pose):

        """ Go to pose """

        start_pose_ = self.tf_buffer.lookup_transform("map", "cf1/base_link", rospy.Time.now(), rospy.Duration(10))

        # use tf to get the start_x .... and end_y in the map frame.
        resolution = 0.05
        start_x = int(round(-start_pose_.transform.translation.x/resolution)+100)  # transfer it from meters to pixels
        start_y = int(round(-start_pose_.transform.translation.y/resolution)+100)
        end_x = int(round(-observe_pose.x/resolution)+100)
        end_y = int(round(-observe_pose.y/resolution)+100)

        # Call path planning and following sequence

        path_planning = rospy.ServiceProxy('path_planning', PathPlanning)
        path = path_planning(start_x, start_y, end_x, end_y)


        pathx = [-(c-100)*resolution for c in path.rx] # In meters, which can be published to the /cf1/cmd_position
        pathy = [-(c-100)*resolution for c in path.ry] # In meters, which can be published to the /cf1/cmd_position
        path_yaw = []
        for i in range(len(pathx)-1):
            yaw = math.degrees(math.atan2(pathy[i+1]-pathy[i],pathx[i+1]-pathx[i]))
            path_yaw.append(yaw)
        path_yaw.append(observe_pose.yaw)

        # print("pathx: ", pathx)
        # print("pathy: ", pathy)
        # print("path_yaw: ", path_yaw)

        rospy.sleep(3) # Pause so humans can read path on screen


        #### Start build path msg to visualize in RVIZ ####
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        poses = []
        for i in range(len(pathx)):
            pose = PoseStamped()
            # pose.header.frame_id = 'map'
            pose.pose.position.x = pathx[i]
            pose.pose.position.y = pathy[i]
            pose.pose.position.z = 0.6

            poses.append(pose)
        path_msg.poses = poses
        path_msg.header.stamp = rospy.Time.now()
        self.path_pub.publish(path_msg)
        #### End build path msg to visualize in RVIZ ####


        ###### From here DOWN as path_following() function ######

        """
        Add rotation to path direction before beginning path
        """

        path_pose = Position()
        path_pose.z = 0.6
        path_pose.yaw = 0.0

        path_rate = rospy.Rate(0.5)
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

        ###### From here UP as path_following() function ######


        self.goal_pub.publish(observe_pose)
        rospy.loginfo('Confirm at end of path - 1x')
        rospy.sleep(3)
        self.goal_pub.publish(observe_pose)
        rospy.loginfo('Confirm at end of path - 2x')
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

