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
import zbar
import Image as Image_

#Authors: Guido Claessen, Bas Janssen
#Fontys Mechatronica
#2016

x = 0
zeroX = 0
zeroY = 0
pixel = 1

# Class that converts the image/video obtained from the bluefox2_single/image_raw topic to a image/video which
# can be edited in python with use of openCV.
# The class can also determine the location of it's reference qr code to calculate relative positon and size of the detected objects.
class image_converter:    
    def __init__(self):
        # Publisher for the box coordinates and size
        self.box_pub = rospy.Publisher("prj7_box_bluefox", prj7_box, queue_size=1000)

        self.bridge = CvBridge()
        # The image/video is received from the bluefox2_single/image_raw topic
        self.image_sub = rospy.Subscriber("bluefox2_single/image_raw", Image,self.callback)

    def zeroreff(self, cv_image):
        global zeroX
        global zeroY 
        global pixel
        
        # create a reader
        scanner = zbar.ImageScanner()

        # configure the reader
        scanner.parse_config('enable')

        # sharpen the camera image, for better qr code detection.
        kernel_sharpen_3 = np.array([[-1,-1,-1,-1,-1],
                             [-1,2,2,2,-1],
                             [-1,2,8,2,-1],
                             [-1,2,2,2,-1],
                             [-1,-1,-1,-1,-1]]) / 8.0
                             
        output_3 = cv2.filter2D(cv_image, -1, kernel_sharpen_3)

        # convert the image data to a suitable format for zbar
        width, height = output_3.shape[:2]
        raw = output_3.tostring()        
        pil = Image_.fromstring("L", (height, width), raw)   
        width, height = pil.size    
        zbarString = pil.tostring()

        # wrap image data
        image = zbar.Image(width, height, 'Y800', zbarString)

        # scan the image for barcodes
        scanner.scan(image)

        # extract results
        for symbol in image:
            # do something useful with results
            if symbol.data == "None":
                return "No QR code found"
            else:
                loc = symbol.location
                print (loc)                
                zeroX = (loc[0][0]+loc[2][0])/2
                zeroY = (loc[0][1]+loc[2][1])/2
                width = (loc[1][0] - loc[0][0])
                height = (loc[2][1] - loc[1][1])
                pixel_x = abs(74.0 / width)#/1000
                pixel_y = abs(74.0 / height)#/1000
                pixel = max(pixel_x, pixel_y)
                print (symbol.data)
                print ("Zero:", zeroX, zeroY)
                print ("Dimensions:", width, height)
                print ("mm/px:", pixel_x, pixel_y, pixel)
#                cv2.rectangle(cv_image, symbol.location[0], symbol.location[2], (0, 255, 0))

    def callback(self,data):
        try:
            # cv_image = converted image/video from ROS to openCV
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            print(e)


        global zeroX
        global zeroY
        global pixel
        global x
        
        # Perhaps we can do this once in a while to make sure the calibration stays acurate.
        if x<1:
            image_converter.zeroreff(self, cv_image)
            x = 1

#--------------------Code to detect objects / get cordinates ----------------------------------------        
        # blur the image slightly, and threshold it
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        thresh = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY_INV)[1]
        
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
         #Publish the center location, size and rotation of the box to the box topic in meters and degrees (@TODO convert degrees to radians)
         self.box_pub.publish(id, (xy[0] - zeroX) * pixel, (xy[1] - zeroY) * pixel, size[0] * pixel, size[1] * pixel, rect[2])
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
