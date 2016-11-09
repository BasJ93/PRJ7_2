#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import imutils

# Class that converts the image/video obtained from the bluefox2_single/image_raw topic to a image/video which
# can be edited in python with use of openCV. 
class image_converter:    
    def __init__(self):
        # The resulting image/video after editing is published to the image_topic_2 topic
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=1000)

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
        blurred = cv2.GaussianBlur(cv_image, (3, 3), 0)
        thresh = cv2.threshold(blurred, 84, 255, cv2.THRESH_BINARY_INV)[1]
        
        # find contours in the thresholded image
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        
        # loop over the contours
	for c in cnts:
		# compute the center of the contour
		M = cv2.moments(c)
#  		print(M)
		if M["m00"] > 0:
        		cX = int(M["m10"] / M["m00"])
        		cY = int(M["m01"] / M["m00"])
        	 
        		# draw the contour and center of the shape on the image
        		cv2.drawContours(cv_image, [c], -1, (0, 255, 0), 2)
        		cv2.circle(cv_image, (cX, cY), 7, (255, 255, 255), -1)
        		cv2.putText(cv_image, "center", (cX - 20, cY - 20),
        			cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
#------------------------------------------------------------------------------------------------------------        
        # Window which displays the image
        cv2.imshow("Thresh", thresh)
        cv2.imshow("Detection", cv_image)
        cv2.waitKey(3)
    
        try:
            # Convert imgae/video back from openCV to ROS
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))
        except CvBridgeError as e:
          print(e)

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

#-----------------------------------------------------------------------------------------------------    
