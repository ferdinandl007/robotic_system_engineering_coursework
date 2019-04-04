#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from my_baxter.msg import Blobs
from my_baxter.msg import Disk
from my_baxter.msg import Board
import cv2
import numpy as np
import DiskNew as dk
import Board as bd
print(cv2.__version__)
opencv_2 = cv2.__version__.startswith('2')


# initializations
MAX_NUM_DISKS = 10
# string_col = sys.argv[1]
# string_col = 'pink,red,orange,yellow,green,dark_green'
string_col = 'green,red,yellow'
hanoi_colors = string_col.split(',')
latest_complete_bounds = 0  # last position of both bounds

# publlisher to board state (disks)
boardPub = rospy.Publisher('/hanoi/boardState',Board)


def callbackBlobs(data):
    global latest_complete_bounds, hanoi_board
    hanoi_board = bd.Board() # initialize clean board class

    # save last complete bounds state
    if len(data.bounds) == 2:
        latest_complete_bounds = data.bounds

    # make disks info based on the detected color blobs
    # coord_x,coord_y,disk_color,size (theoretical size)
    found_disks = []
    for d in range(len(data.blobs)):
        found_disks.append(dk.DiskNew(data.blobs[d].coord_x, data.blobs[d].coord_y, data.blobs[d].disk_color, data.blobs[d].size))

    # get board state (disk placement on board) based on board bounds
    hanoi_board.determineBoardLayout(data.bounds, found_disks)
    # board_encoding = hanoi_board.getBoardEncoding(MAX_NUM_DISKS)

    # make Board message and publish it
    board_msg = Board()
    for l in hanoi_board.left_tower:
        disk = Disk(l.color,l.size)
        board_msg.left.append(disk)
    for m in hanoi_board.middle_tower:
        disk = Disk(m.color,m.size)
        board_msg.middle.append(disk)
    for r in hanoi_board.right_tower:
        disk = Disk(r.color,r.size)
        board_msg.right.append(disk)
    boardPub.publish(board_msg)


if __name__ == "__main__":

    # create node
    rospy.init_node('boardRecognitionController', anonymous=True)

    # create subscriber to board configuration topic
    board_config_sub = rospy.Subscriber('/hanoi/boardConfiguration', Blobs, callbackBlobs)

    # create subscriber to the right hand camera
    camera_sub = rospy.Subscriber('/cameras2/right_hand_camera/image',Image)

    #prevents program from exiting, allowing subscribers and publishers to keep operating
    #in our case that is the camera subscriber and the image processing callback function
    rospy.spin()
