#!/usr/bin/env python
from __future__ import print_function
import sys
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
#import roslib
#import imutils
#import imageio
#import skimage
import Image
import rospy
import zbar
import cv2


#QR codes are 74mm square

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

#        pil = Image.open("/home/bas/catkin_ws/src/prj7_vision_test/src/camera_far.jpg").convert('L')

        kernel_sharpen_3 = np.array([[-1,-1,-1,-1,-1],
                             [-1,2,2,2,-1],
                             [-1,2,8,2,-1],
                             [-1,2,2,2,-1],
                             [-1,-1,-1,-1,-1]]) / 8.0
                             
        output_3 = cv2.filter2D(cv_image, -1, kernel_sharpen_3)

        # obtain image data
        width, height = output_3.shape[:2]
        raw = output_3.tostring()        
        pil = Image.fromstring("L", (height, width), raw)   
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
#                print (loc)                
                x = (loc[0][0]+loc[2][0])/2
                y = (loc[0][1]+loc[2][1])/2
                width = (loc[2][0] - loc[0][0])
                height = (loc[2][1] - loc[0][1])
                pixel_x = abs(74.0 / width)
                pixel_y = abs(74.0 / height)
                print (symbol.data)
                print ("Zero:", x, y)
                print ("Dimensions:", width, height)
                print ("mm/px:", pixel_x, pixel_y, max(pixel_x, pixel_y))
                cv2.rectangle(cv_image, symbol.location[0], symbol.location[2], (0, 255, 0))
            
#------------------------------------------------------------------------------------------------------------        
        # Window which displays the image
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