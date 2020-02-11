#!/usr/bin/env python


from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("/myresult", Image, queue_size=2)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, self.callback)


    def callback(self,data):
        # Convert the image from OpenCV to ROS format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

            

        # Convert BGR to HSV
        blurred_cv_image = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred_cv_image, cv2.COLOR_BGR2HSV)

        # define range of the color we look for in the HSV space
        lower_red = np.array([155,25,0])
        upper_red = np.array([179,255,255])

        # Threshold the HSV image to get only the pixels in ranage
        mask = cv2.inRange(hsv, lower_red, upper_red)
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = box.astype('int')

            cv2.drawContours(cv_image, [box], -1, (0, 255, 0), 3)

        # Bitwise-AND mask and original image
        #res = cv2.bitwise_and(cv_image, cv_image, mask= mask)

        # Publish the image
        try:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
          print(e)

def main(args):
    rospy.init_node('marker', anonymous=True)    

    ic = image_converter()

    print("running...")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
