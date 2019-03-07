#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from baxter_core_msgs.msg import DigitalIOState
from std_msgs.msg import Bool
import cv2
import blobDetectionNEW as bd
import disksArrangement as da
#import Disk
import DiskNew
import Board
print(cv2.__version__)

# TODO make this node start only after all calibrations are done ????
# colors need to be calibrated well, atm they are all grey


# TODO load color names from a file/cmd line
# hanoi_colors = ['pink','red','orange','yellow','green','dark_green','blue','dark_blue']    # temporarily hard-coded
hanoi_colors = ['pink', 'red', 'orange', 'yellow', 'green', 'dark_green']


# load parameters
hsv_baxter = bd.loadHSV()                                   # load HSV ranges
white_balance,gamma,tile_size = bd.loadProcessParams()      # load pre-processing params
# hsv_baxter = bd.hsv_manual

# initialize board
hanoi_board = Board.Board()
hanoi_board_old = Board.Board()
board_changed = False

# publisher to which we post messages for Baxter's screen and board state
boardPub = rospy.Publisher('/hanoi/boardConfiguration',String)
boxesPub = rospy.Publisher('/robot/xdisplay',Image)

#callback function for camera subscriber, called by the camera subscriber for every frame.
def callback(data):
    global hanoi_board,board_changed,hanoi_board_old

    bridge = CvBridge()
    
    # Convert incoming image from a ROS image message to a CV image that open CV can process.
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    # Display the converted cv image, this is the raw camera feed data.
    cv2.imshow("Hand Camera Feed", cv_image)
    
    # preprocess image based on params
    preprocessed_image = bd.preprocessImage(cv_image.copy(), gamma, tile_size, white_balance)

    # detect blobs based on hsv ranges
    centers_x, centers_y, widths, heights, rotations, found_colors, contours = bd.getBoundingBoxes(preprocessed_image.copy(),
                                                                                                           hsv_baxter,
                                                                                                           hanoi_colors)
    stacked_masks,boxes_image = bd.drawBoxes(cv_image.copy(), contours, found_colors)

    # make disks info based on the detected color blobs
    all_disks = []
    size = 1
    for color in hanoi_colors:
        all_disks.append(DiskNew.DiskNew(color, size))
        size += 1 # added as theoretical value, to check if a bigger disk is placed on top of a smaller disk
    all_disks = da.updateDetectedDisks(all_disks, centers_x, centers_y, widths, heights, rotations, found_colors)

    # get new board state
    detected_disks = []
    for d in all_disks:
        if (d.center_x is not None) and (d.center_y is not None):
            detected_disks.append(d)
    hanoi_board_old = hanoi_board
    hanoi_board = Board.Board()
    hanoi_board = da.getBoardLayout(cv_image,detected_disks,hanoi_board) # get board config

    # check state
    board_changed = da.boardChanged(hanoi_board_old,hanoi_board)
    print('Board changed: '+str(board_changed))
    da.printBoard(hanoi_board)  # print board state

    # publish bounding boxes to baxter's screen
    out_image_boxes = bridge.cv2_to_imgmsg(boxes_image, "bgr8")
    boxesPub.publish(out_image_boxes)

    # TODO create message type and publish board state to topic
    boardPub.publish('Hello')
    cv2.imshow("Bounding boxes", boxes_image)
    cv2.imshow("Masks", stacked_masks)
    cv2.waitKey(1) # needed for cv2.imshow to take effect. REMOVE otherwise


if __name__ == '__main__':

    rospy.init_node('boardrecognition', anonymous=True)

    # subscriber to the color calibration node, this node shuld start only when calibration was done (i.e. True on /hanoi/colorCalibration) ???? not sure though
    # calib_sub = rospy.Subscriber('/hanoi/colorCalibration',Bool,callback)    

    # create subscriber to the right hand camera, each frame recieved calls the callback function
    camera_sub = rospy.Subscriber('/cameras/right_hand_camera/image',Image,callback)

    # prevents program from exiting, allowing subscribers and publishers to keep operating
    # in our case that is the camera subscriber and the image processing callback function
    rospy.spin()

