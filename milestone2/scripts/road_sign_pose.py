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
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped

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
    param = rospy.search_param("binary_image_topic")
    self.binary_img_top = rospy.get_param(param)
    param = rospy.search_param("camera_frame")
    self.cam_frame = rospy.get_param(param)
    param = rospy.search_param("odometry_frame")
    self.odom_frame = rospy.get_param(param)
    # param = rospy.search_param("map_frame")
    # self.map_frame = rospy.get_param(param)


    # From '/home/robot/dd2419_ws/src/course_packages/dd2419_launch/calibration/camera.yaml' or topic: /cf1/camera/camera_info
    #              [    fx    ,     s   ,     x0    ,     0   ,     fy    ,     y0    ,     0   ,     0   ,     1   ]
    self.cam_mat = [231.250001, 0.000000, 320.519378, 0.000000, 231.065552, 240.631482, 0.000000, 0.000000, 1.000000]
    # From '/home/robot/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/______.world.json'
    self.sign_dim = [0.20, 0.20]
    
    
    # Initialize subscriber to bounding box
    rospy.Subscriber(self.bounding_boxes_top, BoundingBoxes, self.boxes_cb)
    
    # Initialize subscriber to image
    rospy.Subscriber(self.img_top, Image, self.img_cb)
    # Initialize CvBridge
    self.bridge = CvBridge()
    # Initialize publisher for binary, thresholded sign (helps for debugging)
    self.image_pub = rospy.Publisher(self.binary_img_top, Image, queue_size=2)

    # Initialize tf broadcaster
    self.br = tf2_ros.TransformBroadcaster()
    # Listen to other tf transforms
    tfBuffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfBuffer)
  

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
    # Get coordinates of 
    for i in range(len(self.boxes)):
          box = self.boxes[i]
          # Class
          ob_class = box.Class
          print(ob_class)
          if ob_class == 'stop sign':
                
            # Size
            self.xsize = box.xmax - box.xmin
            self.ysize = box.ymax - box.ymin
            pix_area = self.xsize*self.ysize
            self.y_over_x = float(self.ysize)/float(self.xsize)
            print('y over x: ', self.y_over_x)

            # Image Center
            self.xc = (box.xmax + box.xmin)/2
            yc = (box.ymax + box.ymin)/2

            # Camera Parameters
            foc_dist = (self.cam_mat[0] + self.cam_mat[4])/2
            self.x0 = self.cam_mat[2]
            y0 = self.cam_mat[5]

            # Test different sign distance calculations
            #sign_distx = foc_dist*self.sign_dim[0]/self.xsize
            sign_disty = foc_dist*self.sign_dim[1]/self.ysize
            #sign_area = self.sign_dim[0]*self.sign_dim[1]
            #sign_dist2 = foc_dist*math.sqrt(sign_area)/math.sqrt(pix_area)
            # Choose which Z calculation to use
            z = sign_disty
            # Infer x and y tranforms
            x = z*(self.xc - self.x0)/foc_dist
            y = z*(yc - y0)/foc_dist

            # Crop out image inside bounding box (w/ some padding)
            pad = 10 # pixels
            crop_img = self.cv_image[box.ymin-pad:box.ymax+pad, box.xmin-pad:box.xmax+pad, :]
            
            # Call angle extraction function
            ang = self.extract_angle(crop_img)
            #ang = 0


            # Broadcast transform to detected sign
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = self.cam_frame
            t.child_frame_id = 'detected/' + str(ob_class)
            
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            quat = quaternion_from_euler(-1.54+ang, 0, -1.54)
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]

            self.br.sendTransform(t)

  def extract_angle(self, crop_img):
    # For color red
    lower1 = np.array([150,25,0])
    upper1 = np.array([180,255,255])
    lower2 = np.array([0,25,0])
    upper2 = np.array([30,255,255])

    # Convert BGR to HSV
    hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only the pixels in ranage
    mask1 = cv2.inRange(hsv, lower1, upper1)
    mask2 = cv2.inRange(hsv, lower2, upper2)

    # Bitwise-AND mask and original image
    crop_img1 = cv2.bitwise_and(crop_img, crop_img, mask= mask1)
    crop_img2 = cv2.bitwise_and(crop_img, crop_img, mask= mask2)

    crop_img = crop_img1 + crop_img2
    
    # Turn thresholded image into 2D binary image
    vals = np.sum(crop_img, axis = 2)
    vals[vals > 0] = 1

    # Perform dilation to fill in words
    vals = ndimage.binary_dilation(vals, iterations = 3).astype(vals.dtype)

    # Create 3D binary image to publish (for debugging only)
    pubvals = np.repeat(vals[:, :, np.newaxis], 3, axis=2)*255
    pubvals = pubvals.astype(np.uint8)

    # Calculate center of mass in x-direction
    sums = np.sum(vals, axis=0)
    # Remove zeros (padding around sign)
    sums = sums[sums > 10]

    # Find where the max location occurs
    # If multiple occurences, average the locations
    where = np.where(sums == np.amax(sums))[0]
    print(where)
    max_loc = float(np.average(where))/float(sums.size)

    # cells = np.arange(sums.size)

    # if np.sum(sums) > 0:
    #   x_com = np.dot(sums,cells)/np.sum(sums)
    # else:
    #   x_com = sums.size/2

    # # Calculate ratio of C.O.M. to box center
    # ratio = x_com/sums.size

    # Tuneable parameters
    rect_slide = 0.25 # Ratio change y over x for 90deg | default: 0.5
    max_slide = 0.2 # Ratio change of max_loc for 90deg | default: 0.2
    ave = 0.5 # Typical center for max_loc | default: 0.5

    # if ratio == 0:
    #   ang = 0
    # else:
    if self.y_over_x < 1:
          self.y_over_x = 1

    ang = 1.57*((self.y_over_x - 1)/rect_slide)*((ave - max_loc)/max_slide)
    #ang = (self.y_over_x1.57/slide)*(ave - max_loc)
    
    if abs(ang) > 1.57:
          ang = 1.57*np.sign(ang)

    print('max loc: ', max_loc)
    print('angle: ', ang)

    # Publish thresholded sign (only for debugging)
    try:
      #self.image_pub.publish(self.bridge.cv2_to_imgmsg(crop_img, "bgr8"))
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(pubvals, "bgr8"))
      # self.image_pub.publish(self.bridge.cv2_to_imgmsg(vals, encoding="passthrough"))
    except CvBridgeError as e:
      print(e)

    return ang


          
          

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
