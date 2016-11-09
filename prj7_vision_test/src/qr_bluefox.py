#!/usr/bin/env python
from __future__ import print_function
import sys
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge, CvBridgeError
#import numpy as np
#import roslib
#import imutils
#import imageio
#import skimage
import Image
import rospy
import zbar
import cv2

x = 0

class image_converter:    
    def __init__(self):
        self.bridge = CvBridge()
        # The image/video is received from the bluefox2_single/image_raw topic
        self.image_sub = rospy.Subscriber("bluefox2_single/image_raw", RosImage,self.callback)

    def callback(self,data):
        try:
            # cv_image = converted image/video from ROS to openCV
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            print(e)
            
#--------------------Code to detect QR code ----------------------------------------------------------   
        # create a reader
        scanner = zbar.ImageScanner()

        # configure the reader
        scanner.parse_config('enable')

        # obtain image data
        width, height = cv_image.shape[:2]
        raw = cv_image.tostring()        
#        pil = Image.fromstring("L", (height, width), raw)       
#        zbarString = pil.tostring()

        # wrap image data
        image = zbar.Image(width, height, 'Y800', raw)

        # scan the image for barcodes
        scanner.scan(image)

        # extract results
        for symbol in image:
            print (symbol.type)
            # do something useful with results
            if symbol.data == "None":
                return "No QR code found"
            else:
                return symbol.data
            print (symbol.location)
            print ("123")
            
#        print ("567")
            
#------------------------------------------------------------------------------------------------------------        
        # Window which displays the image
        global x
        if x < 1:
            raw.show()
            x = x+1
        cv2.imshow("Detection", cv_image)
        cv2.waitKey(3)
        
#---------------------------------------------------------------------------------------------------------------
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