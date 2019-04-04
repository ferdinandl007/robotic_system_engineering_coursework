#!/usr/bin/env python

import sys
import zbar
import rospy
from sensor_msgs.msg import Image
from my_baxter.msg import Blobs
from my_baxter.msg import Blob
from my_baxter.msg import Bound
from cv_bridge import CvBridge, CvBridgeError
from baxter_core_msgs.msg import DigitalIOState
import cv2
import blobFunctions as bf
import QRalternative as qr
print(cv2.__version__)

# TODO make this node start only after all calibrations are done ????
# colors need to be calibrated well, atm they are all grey


# string_col = sys.argv[1]
# string_col = 'pink,red,orange,yellow,green,dark_green'
string_col = 'green,red,yellow'
hsv_offset = 10

# TODO load color names from a file/cmd line
# hanoi_colors = ['pink','red','orange','yellow','green','dark_green','blue','dark_blue']    # temporarily hard-coded
hanoi_colors = string_col.split(',')
# their sizes are increasing: color 1 - size 1, ..., color n - size n

# load parameters
# hsv_baxter = bd.loadHSV()                                   # load HSV ranges
# white_balance,gamma,tile_size = bf.loadProcessParams()      # load pre-processing params
# hsv_baxter = bd.hsv_manual
bgr_average = bf.getBGRavgColorsFromFile('/home/tina/hrwros_ws/src/my_baxter/src/webcam_disks')
hsv_baxter = bf.calibrateColors(bgr_average,hsv_offset)

# make scanner object for QR code detection
scanner = zbar.ImageScanner()

# publisher to board config (Blobs information)
blobPub = rospy.Publisher('/hanoi/boardConfiguration', Blobs)
# publisher to Baxter's screen
boxesPub = rospy.Publisher('/robot/xdisplay',Image)

# callback function for camera subscriber, called by the camera subscriber for every frame.


def callback(data):
    global hanoi_board,board_changed,hanoi_board_old,scanner

    bridge = CvBridge()
    
    # Convert incoming image from a ROS image message to a CV image that open CV can process.
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    # Display the converted cv image, this is the raw camera feed data.
    cv2.imshow("Hand Camera Feed", cv_image)
    
    # preprocess image based on params
    # preprocessed_image = bf.preprocessImage(cv_image.copy(), gamma, tile_size, white_balance)
    preprocessed_image = cv_image

    # detect blobs based on hsv ranges
    centers_x, centers_y, widths, heights, rotations, found_colors, contours = bf.getBoundingBoxes(preprocessed_image.copy(),
                                                                                                           hsv_baxter,
                                                                                                           hanoi_colors)
    stacked_masks,boxes_image = bf.drawBoxes(cv_image.copy(), contours, found_colors)
    # found_colors will keep the same order as the original hanoi_colors
    # not found colors will be skipped

    # get board bounds
    # process frame for scanner
    h, w, c = cv_image.shape
    img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    image = zbar.Image(w, h, 'Y800', img_gray.tostring())
    scanner.scan(image)

    # find codes
    codes_coordinates = qr.getCodes(image)
    centers, img_final = qr.getCodesCenters(cv_image, codes_coordinates)

    # should detect which code is which side (by x coordinate)
    # encode this in the message as well

    # publish bounding boxes to baxter's screen
    out_image_boxes = bridge.cv2_to_imgmsg(boxes_image, "bgr8")
    boxesPub.publish(out_image_boxes)

    # publish board state to topic
    blobs_msg = Blobs()
    for b in range(len(found_colors)):
        blob = Blob(found_colors[b],centers_x[b],centers_y[b],hanoi_colors.index(found_colors[b])+1)
        blobs_msg.blobs.append(blob)
    for c in centers:
        bound = Bound(c[0],c[1])
        blobs_msg.bounds.append(bound)

    blobPub.publish(blobs_msg)
    cv2.imshow("Bounding boxes", boxes_image)
    cv2.imshow("Masks", stacked_masks)
    cv2.waitKey(1) # needed for cv2.imshow to take effect. REMOVE otherwise


if __name__ == '__main__':

    rospy.init_node('blobDetectionController', anonymous=True)

    # subscriber to the color calibration node, this node shuld start only when calibration was done (i.e. True on /hanoi/colorCalibration) ???? not sure though
    # calib_sub = rospy.Subscriber('/hanoi/colorCalibration',Bool,callback)    

    # create subscriber to the right hand camera
    camera_sub = rospy.Subscriber('/cameras2/right_hand_camera/image',Image,callback)

    # prevents program from exiting, allowing subscribers and publishers to keep operating
    # in our case that is the camera subscriber and the image processing callback function
    rospy.spin()

