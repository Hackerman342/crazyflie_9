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

class ObjectDetect:

  def __init__(self):
    print('openCV version:', cv2.__version__)

    
    self.image_pub = rospy.Publisher("/object_detect", Image, queue_size=2)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, self.callback)


    ##### DNN stuff #####

    # arguments from the argument parser of the example
    self.prototxt = 'MobileNetSSD_deploy.prototxt.txt'
    self.model = 'MobileNetSSD_deploy.caffemodel'
    self.confidence = 0.2 # Minimum probability

    # initialize the list of class labels MobileNet SSD was trained to
    # detect, then generate a set of bounding box colors for each class
    self.CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
      "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
      "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
      "sofa", "train", "tvmonitor"]
    self.COLORS = np.random.uniform(0, 255, size=(len(self.CLASSES), 3))

    # load our serialized model from disk
    print("[INFO] loading model...")
    self.net = cv2.dnn.readNetFromCaffe(self.prototxt, self.model)

  
  def callback(self,data):
    # Convert the image from OpenCV to ROS format
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    ##
    # Do the dnn stuff to the image
    ##
    result = self.dnn_object_detect(cv_image)

    # Publish the image
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(result, "bgr8"))
    except CvBridgeError as e:
      print(e)

  
  def dnn_object_detect(self, image):
    (h, w) = image.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)

    # pass the blob through the network and obtain the detections and
    # predictions
    print("[INFO] computing object detections...")
    self.net.setInput(blob)
    detections = self.net.forward()

    # loop over the detections
    for i in np.arange(0, detections.shape[2]):
      # extract the confidence (i.e., probability) associated with the
      # prediction
      confidence = detections[0, 0, i, 2]

      # filter out weak detections by ensuring the `confidence` is
      # greater than the minimum confidence
      if confidence > self.confidence:
        # extract the index of the class label from the `detections`,
        # then compute the (x, y)-coordinates of the bounding box for
        # the object
        idx = int(detections[0, 0, i, 1])
        box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
        (startX, startY, endX, endY) = box.astype("int")

        # display the prediction
        label = "{}: {:.2f}%".format(self.CLASSES[idx], confidence * 100)
        print("[INFO] {}".format(label))
        cv2.rectangle(image, (startX, startY), (endX, endY),
          self.COLORS[idx], 2)
        y = startY - 15 if startY - 15 > 15 else startY + 15
        cv2.putText(image, label, (startX, y),
          cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLORS[idx], 2)

    return image

def main(args):
  rospy.init_node('stop_sign_detect', anonymous=True)

  od = ObjectDetect()

  print("running...")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
