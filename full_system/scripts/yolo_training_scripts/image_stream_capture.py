#!/usr/bin/env python
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

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
"""
Capture images from a video stream published over ROS as an Image msg
"""


class ImageStreamCapture:

    def __init__(self):

        self.img_top = "/cf1/camera/image_raw"

        self.img_count = 100

        self.initial_label = 101 # integer


        self.fps = 5 # [hz]

        self.resize_bool = False # True or False

        self.resize = (300, 300) # [x(?), y(?)] pixel dimensions - confirm x & y!

        self.save_path = "/home/robot/yolo_training/training_images/narrows_from_right/"

        self.img_type = ".jpg"

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Initialize subscriber to image
        rospy.Subscriber(self.img_top, Image, self.img_cb)
        rospy.sleep(5) # Pause briefly for subscription



    def img_cb(self,data):
        # Convert the image from OpenCV to ROS format
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def capture(self):
        rate = rospy.Rate(self.fps)
        for i in range(int(self.img_count)):
            if self.resize_bool:
                image = cv2.resize(self.cv_image, self.resize)
            else:
                image = self.cv_image

            cv2.imwrite(self.save_path + str(i+self.initial_label) + self.img_type, image)


            print("captured image ", i+self.initial_label)
            rate.sleep()



    # def extract_pose(self):
    #     # Crop out image inside bounding box (w/ some padding)
    #     pad = 10 # pixels
    #     try:
    #         crop_img = self.cv_image[box.ymin-pad:box.ymax+pad, box.xmin-pad:box.xmax+pad, :]
    #     except:
    #         crop_img = self.cv_image[box.ymin:box.ymax, box.xmin:box.xmax, :]


    # def extract_angle(self, crop_img):
    #     # For color red
    #     lower1 = np.array([150,25,0])
    #     upper1 = np.array([180,255,255])
    #     lower2 = np.array([0,25,0])
    #     upper2 = np.array([30,255,255])

    #     # Convert BGR to HSV
    #     hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

    #     # Threshold the HSV image to get only the pixels in ranage
    #     mask1 = cv2.inRange(hsv, lower1, upper1)
    #     mask2 = cv2.inRange(hsv, lower2, upper2)

    #     # Bitwise-AND mask and original image
    #     crop_img1 = cv2.bitwise_and(crop_img, crop_img, mask= mask1)
    #     crop_img2 = cv2.bitwise_and(crop_img, crop_img, mask= mask2)

    #     crop_img = crop_img1 + crop_img2

    #     # Turn thresholded image into 2D binary image
    #     vals = np.sum(crop_img, axis = 2)
    #     vals[vals > 0] = 1

    #     # Perform dilation to fill in words
    #     vals = ndimage.binary_dilation(vals, iterations = 3).astype(vals.dtype)

    #     if self.binary_publish:
    #         # Create 3D binary image to publish (for debugging only)
    #         pubvals = np.repeat(vals[:, :, np.newaxis], 3, axis=2)*255
    #         pubvals = pubvals.astype(np.uint8)

    #     # Calculate center of mass in x-direction
    #     sums = np.sum(vals, axis=0)
    #     # Remove zeros (padding around sign)
    #     sums = sums[sums > 10]

    #     # Find where the max location occurs
    #     # If multiple occurences, average the locations
    #     where = np.where(sums == np.amax(sums))[0]
    #     max_loc = float(np.average(where))/float(sums.size)

    #     # cells = np.arange(sums.size)

    #     # if np.sum(sums) > 0:
    #     #   x_com = np.dot(sums,cells)/np.sum(sums)
    #     # else:
    #     #   x_com = sums.size/2

    #     # # Calculate ratio of C.O.M. to box center
    #     # ratio = x_com/sums.size

    #     # Tuneable parameters
    #     rect_slide = 0.25 # Ratio change y over x for 90deg | default: 0.5
    #     max_slide = 0.2 # Ratio change of max_loc for 90deg | default: 0.2
    #     ave = 0.5 # Typical center for max_loc | default: 0.5

    #     if self.y_over_x < 1:
    #         self.y_over_x = 1

    #     ang = 1.57*((self.y_over_x - 1)/rect_slide)*((ave - max_loc)/max_slide)

    #     if abs(ang) > 1.57:
    #         ang = 1.57*np.sign(ang)

    #     #print('max loc: ', max_loc)
    #     #print('angle: ', ang)

    #     # Publish thresholded sign (only for debugging)
    #     if self.binary_publish:
    #         try:
    #             self.image_pub.publish(self.bridge.cv2_to_imgmsg(pubvals, "bgr8"))
    #         except CvBridgeError as e:
    #             print(e)

    #     return ang




if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('image_stream_capture', anonymous=True)
    # Initialize SignPose class
    isc = ImageStreamCapture()
    # Run capture function
    isc.capture()


    # print("running...")
    # # Run until KeyboardInterrupt
    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print("Shutting down")