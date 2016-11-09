#!/usr/bin/env python
from __future__ import print_function
from prj7_vision_test.msg import prj7_box
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import imutils
import numpy as np

# Class that converts the image/video obtained from the bluefox2_single/image_raw topic to a image/video which
# can be edited in python with use of openCV. 
class image_converter:    
    def __init__(self):
        # The resulting image/video after editiing is published to the image_topic_2 topic
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=1000)
        # Publisher for the box coordinates and size
        self.box_pub = rospy.Publisher("prj7_box", prj7_box, queue_size=1000)

        self.bridge = CvBridge()
        # The image/video is received from the bluefox2_single/image_raw topic
       # self.image_sub = rospy.Subscriber("bluefox2_single/image_raw", Image,self.callback)
        self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image,self.callback)

    def callback(self,data):
        try:
            # cv_image = converted image/video from ROS to openCV
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            print(e)

#--------------------Code to detect objects / get cordinates ----------------------------------------        
        # blur the image slightly, and threshold it
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        thresh = cv2.threshold(blurred, 84, 255, cv2.THRESH_BINARY_INV)[1]
#        kernel = np.ones((11, 11), np.uint8)
#        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        
        # find contours in the thresholded image
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        
	id = 0
        # loop over the contours
	for c in cnts:
         rect = cv2.minAreaRect(c)
         box = cv2.boxPoints(rect)
         box = np.int0(box) # this returns the center(x,y), width, heigth, and angle of rotation
         
         rect2 = np.array(rect)
         xy = np.array(rect2[0])
         size = np.array(rect2[1])
         
         # Draw smallest possible rectangle around object
         cv2.drawContours(cv_image, [box], 0, (0, 255, 0), 2)
         self.box_pub.publish(id, xy[0], xy[1], size[0], size[1], rect[2])
         id =  id + 1
#         print (rect)
  
#------------------------------------------------------------------------------------------------------------        
        # Window which displays the image
        cv2.imshow("Thresh", thresh)
        cv2.imshow("Detection", cv_image)
#        cv2.imshow("Opening", opening)
        cv2.waitKey(3)
    
        try:
            # Convert imgae/video back from openCV to ROS
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))
        except CvBridgeError as e:
          print(e)

def main(args):
    rospy.init_node('image_converter', anonymous=True)
    image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

#-----------------------------------------------------------------------------------------------------    
