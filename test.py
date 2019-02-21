import cv2
print(cv2.__version__)
import numpy as np
import  os
import glob
opencv_2 = cv2.__version__.startswith('2')
import blobDetection as bd
import disksArrangement as da
import Disk
import Board


#######################################################################################################################
## test with image
# img = cv2.imread('tower.jpg')
# bd.getBoundingBoxes(img,bd.hsv_manual,'hsv')

#######################################################################################################################
## test with webcam
# cap = cv2.VideoCapture(0)
# while(True):
#
#    # delay
#     for i in range(20):
#         print('')
#
#     ret, img = cap.read()
#     bd.getBoundingBoxes(img, bd.hsv_manual, 'hsv')
#     cv2.imshow('frame', img)
#
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
#
# # When everything done, release the capture
# cap.release()
# cv2.destroyAllWindows()


#######################################################################################################################
## test with baxters camera
cap = cv2.VideoCapture('baxter_video.avi')
# Read until video is completed
while (cap.isOpened()):
    # Capture frame-by-frame
    ret, img = cap.read()
    if ret == True:

        # Display the resulting frame
        cv2.imshow('Frame', img)

        # Get blobs coordinates, info
        centers_x,centers_y,widths,heights,rotations,found_colors = bd.getBoundingBoxes(img, bd.hsv_manual, 'hsv')

        # Construct disks and empty board
        disks_list = da.make_disks(centers_x, centers_y, widths, heights, rotations, found_colors)
        board = Board.Board()

        # Determine disks arrangement
        board = da.getBoardLayout(img, disks_list, board)

        # Print board state
        print('Left tower:')
        for l in reversed(board.left_tower):
            print('['+l.color+'] ')
        print('Middle tower:')
        for m in reversed(board.middle_tower):
            print('['+m.color+'] ')
        print('Right tower:')
        for r in reversed(board.right_tower):
            print('['+r.color+'] ')
        print('########################################')

        # Press Q on keyboard to  exit
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

    # Break the loop
    else:
        break

# When everything done, release the video capture object
cap.release()

# Closes all the frames
cv2.destroyAllWindows()


