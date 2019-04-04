#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from baxter_core_msgs.msg import DigitalIOState
from std_msgs.msg import Bool
import cv2
import QRalternative as qr
import zbar
import numpy as np

print(cv2.__version__)
opencv_2 = cv2.__version__.startswith('2')

# make scanner object
scanner = zbar.ImageScanner()

# codes center
centers = 0

def callback(data):
    global centers

    bridge = CvBridge()

    # Convert incoming image from a ROS image message to a CV image that open CV can process.
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    # Display the converted cv image, this is the raw camera feed data.
    cv2.imshow("Hand Camera Feed", cv_image)
    cv2.waitKey(1)
    h, w, c = cv_image.shape
    img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    image = zbar.Image(w, h, 'Y800', img_gray.tostring())
    scanner.scan(image)

    # find codes
    codes_coordinates = qr.getCodes(image)
    centers, img_final = qr.getCodesCenters(cv_image, codes_coordinates)
    print(centers)



if __name__ == "__main__":

    # create node
    rospy.init_node('boardboundsdetection', anonymous=True)

    # create subscriber to the right hand camera, each frame recieved calls the callback function
    camera_sub = rospy.Subscriber('camera_head',Image,callback)


    #prevents program from exiting, allowing subscribers and publishers to keep operating
    #in our case that is the camera subscriber and the image processing callback function
    rospy.spin()

