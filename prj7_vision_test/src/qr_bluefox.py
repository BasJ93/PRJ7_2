#!/usr/bin/python
import sys
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
import zbar
from sensor_msgs.msg import Image as RosImage
import Image
import cv2

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


        cv2.imshow("QR", cv_image)
#        pil_im = Image.fromarray(cv_image)
#        pil_im.show()

        # create a reader
#        scanner = zbar.ImageScanner()

        # configure the reader
#        scanner.parse_config('enable')

#        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY,dstCn=0)
        # obtain image data
#        pil = Image.fromarray(gray)
#        width, height = pil.size
#        raw = pil.tostring()


#        gray = Image.fromstring("L", cv_image.shape[:2], cv_image.tostring())
#        pil = Image.fromarray(gray)
#        width, height = pil.size
#        raw = pil.tostring()
        # wrap image data
#        image = zbar.Image(width, height, 'Y800', raw)

        # scan the image for barcodes
#        scanner.scan(image)        

        # extract results
#        for symbol in image:
            # do something useful with results
#            print (symbol.location)
#            print ("123")
            
#        raw.show()
#        print ("567")

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