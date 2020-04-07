#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
import math

from scipy import ndimage

import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import TransformStamped, PoseStamped

from darknet_ros_msgs.msg import ObjectCount, BoundingBox, BoundingBoxes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
"""
Read a bounding box and object id and determine pose in camera frame from bounding box
"""


class SignPose:

  def __init__(self):

    # Pull ROS parameters from launch file:
    param = rospy.search_param("bounding_box_topic")
    self.bounding_boxes_top = rospy.get_param(param)
    param = rospy.search_param("image_topic")
    self.img_top = rospy.get_param(param)
    # param = rospy.search_param("pose_difference_topic")
    # self.pose_diff_top = rospy.get_param(param)

    param = rospy.search_param("camera_frame")
    self.cam_frame = rospy.get_param(param)
    param = rospy.search_param("odometry_frame")
    self.odom_frame = rospy.get_param(param)
    param = rospy.search_param("map_frame")
    self.map_frame = rospy.get_param(param)

    # From '/home/robot/dd2419_ws/src/course_packages/dd2419_launch/calibration/camera.yaml' or topic: /cf1/camera/camera_info
    #              [    fx    ,     s   ,     x0    ,     0   ,     fy    ,     y0    ,     0   ,     0   ,     1   ]
    self.cam_mat = [231.250001, 0.000000, 320.519378, 0.000000, 231.065552, 240.631482, 0.000000, 0.000000, 1.000000]
    # From '/home/robot/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/______.world.json'
    self.sign_dim = [0.20, 0.20]

    self.classes = ['narrows_from_left', 'no_bicycle', 'residential', 'roundabout']
    # From '/home/robot/dd2419_ws/src/darknet_ros/darknet_ros/config/yolo-lite-cf9-4classes.yaml'

    # Initialize subscriber to bounding box
    rospy.Subscriber(self.bounding_boxes_top, BoundingBoxes, self.boxes_cb)

    # Initialize subscriber to image
    rospy.Subscriber(self.img_top, Image, self.img_cb)
    # Initialize CvBridge
    self.bridge = CvBridge()

    # Initialize tf broadcaster
    self.br = tf2_ros.TransformBroadcaster()
    # Listen to other tf transforms
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)

    # Initialize publisher for differecne between detected and true sign pose
    # self.pose_diff_pub = rospy.Publisher(self.pose_diff_top, PoseStamped, queue_size=2)


  def boxes_cb(self,msg):
    # Get timestamp from message and image
    self.img_time = msg.image_header.stamp.secs + (10**-9)*msg.image_header.stamp.nsecs # Time image was actually taken
    self.header_time = msg.header.stamp.secs + (10**-9)*msg.header.stamp.nsecs # Time reading is received
    # Read boxes
    self.boxes = msg.bounding_boxes
    # Call pose extraction function
    self.extract_pose()


  def img_cb(self,data):
      # Convert the image from OpenCV to ROS format
      try:
          self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError as e:
          print(e)


  def extract_pose(self):
    # Get coordinates of detected road signs
    for i in range(len(self.boxes)):

        box = self.boxes[i]
        ob_class = box.Class

        # Size
        self.xsize = box.xmax - box.xmin
        self.ysize = box.ymax - box.ymin
        #pix_area = self.xsize*self.ysize
        self.y_over_x = float(self.ysize)/float(self.xsize)

        # Image Center
        self.xc = (box.xmax + box.xmin)/2
        yc = (box.ymax + box.ymin)/2

        # Camera Parameters
        foc_dist = (self.cam_mat[0] + self.cam_mat[4])/2
        self.x0 = self.cam_mat[2]
        y0 = self.cam_mat[5]

        # Test different sign distance calculations
        sign_disty = foc_dist*self.sign_dim[1]/self.ysize
        #sign_distx = foc_dist*self.sign_dim[0]/self.xsize
        #sign_area = self.sign_dim[0]*self.sign_dim[1]
        #sign_dist2 = foc_dist*math.sqrt(sign_area)/math.sqrt(pix_area)

        # Choose which Z calculation to use
        z = sign_disty

        # Infer x and y tranforms
        x = z*(self.xc - self.x0)/foc_dist
        y = z*(yc - y0)/foc_dist

        # Broadcast transform to detected sign
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.cam_frame
        t.child_frame_id = str(ob_class) + '/detected'

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        # quat = quaternion_from_euler(1.57, 1.57, 0)
        quat = quaternion_from_euler(-1.57, 0, -1.57)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.br.sendTransform(t)
        # Listen to difference between true stop and detected stop

        # try:
        #   difference = self.relative_tf_in_map(self.map_frame, self.true_stop_frame, t.child_frame_id)
        #   self.pose_diff_pub.publish(difference)
        # except:
        #   print('Error calculating pose difference between true and detected sign')


  def relative_tf_in_map(self, map_frame, true_frame, detected_frame):
    trans_true_stop = self.tfBuffer.lookup_transform(map_frame, true_frame, rospy.Time(0))
    trans_detect_stop = self.tfBuffer.lookup_transform(map_frame, detected_frame, rospy.Time(0))

    # Calc difference between detected and true sign in map frame
    diff = PoseStamped()
    diff.header.frame_id = map_frame
    diff.pose.position.x = trans_true_stop.transform.translation.x - trans_detect_stop.transform.translation.x
    diff.pose.position.y = trans_true_stop.transform.translation.y - trans_detect_stop.transform.translation.y
    diff.pose.position.z = trans_true_stop.transform.translation.z - trans_detect_stop.transform.translation.z

    q_true = []
    q_true.append(trans_true_stop.transform.rotation.x)
    q_true.append(trans_true_stop.transform.rotation.y)
    q_true.append(trans_true_stop.transform.rotation.z)
    q_true.append(trans_true_stop.transform.rotation.w)

    q_detect_inv = [] # -w makes it inverse
    q_detect_inv.append(trans_detect_stop.transform.rotation.x)
    q_detect_inv.append(trans_detect_stop.transform.rotation.y)
    q_detect_inv.append(trans_detect_stop.transform.rotation.z)
    q_detect_inv.append(-1*trans_detect_stop.transform.rotation.w)

    diff.pose.orientation = quaternion_multiply(q_true,q_detect_inv)

    return diff


if __name__ == '__main__':
  # Initialize ROS node
  rospy.init_node('multiclass_road_sign_pose', anonymous=True)
  # Initialize SignPose class
  sp = SignPose()

  print("running...")
  # Run until KeyboardInterrupt
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  # def read_labels(self, labels_file):
  #     """
  # Returns a list of strings

  # Arguments:
  # labels_file -- path to a .txt file
  # """
  # if not labels_file:
  #     print ('WARNING: No labels file provided. Results will be difficult to interpret.')
  #     return None

  # labels = []
  # with open(labels_file) as infile:
  #     for line in infile:
  #         label = line.strip()
  #         if label:
  #             labels.append(label)
  # assert len(labels), 'No labels found'
  # return labels
