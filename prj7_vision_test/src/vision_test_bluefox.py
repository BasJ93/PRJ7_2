#!/usr/bin/env python
from __future__ import print_function
from prj7_vision_test.msg import prj7_box
import roslib
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import imutils
import numpy as np

#Authors: Guido laessen, Bas Janssen
#Fontys Mechatronica
#2016

# Class that converts the image/video obtained from the bluefox2_single/image_raw topic to a image/video which
# can be edited in python with use of openCV. 
class image_converter:    
    def __init__(self):
        # Publisher for the box coordinates and size
        self.box_pub = rospy.Publisher("prj7_box_bluefox", prj7_box, queue_size=1000)

        self.bridge = CvBridge()
        # The image/video is received from the bluefox2_single/image_raw topic
        self.image_sub = rospy.Subscriber("bluefox2_single/image_raw", Image,self.callback)

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
        cv2.waitKey(3)

def main(args):
    rospy.init_node('prj7_vision_test_bluefox', anonymous=True)
    image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

#-----------------------------------------------------------------------------------------------------    
