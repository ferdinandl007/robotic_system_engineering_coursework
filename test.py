import cv2
print(cv2.__version__)
import numpy as np
import  os
import glob
opencv_2 = cv2.__version__.startswith('2')
import skvideo.io
import blobDetection as bd
import disksArrangement as da
import Disk
import Board


#######################################################################################################################
## test with baxters camera

def testVideo():

    cap = skvideo.io.vreader('baxter_video.avi')
    # Read until video is completed
    for img in cap:
        img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)

        # Display the resulting frame
        # img = bd.preprocessImage(img) # if this is not used anymore, also change the hsv values from blobDetection
        cv2.imshow('Frame', img)

        # Get blobs/boxes coordinates, info
        centers_x,centers_y,widths,heights,rotations,found_colors,contours = bd.getBoundingBoxes(img, hsv_baxter,hanoi_colors)

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

        print('')

        # Draw boxes
        bd.drawBoxes(img, contours, found_colors)


    # Closes all the frames
    cv2.destroyAllWindows()


if __name__=="__main__":

    # 1) color calibration
    hanoi_colors = ['pink','red','orange','yellow','green','blue','dark blue']
    img = cv2.imread('centeredTower.png')
    img2 = cv2.imread('centeredTower.png')
    preproc_calib_img = False
    # hsv offset needs to be adjusted depending on the colors used (ex: smaller if both blue and dark blue used)
    hsv_offset = 3
    bgr_baxter = bd.getBGRavgColorsFromBaxter(img,hanoi_colors,preproc_calib_img,True)
    hsv_baxter = bd.calibrateColors(bgr_baxter,hsv_offset)

    # 2) blobs detection

    # # preprocess frames if needed
    # img = bd.preprocessImage(img)

    # # get bounding box info
    # centers_x,centers_y,widths,heights,rotations,found_colors,contours = bd.getBoundingBoxes(img, hsv_baxter, hanoi_colors)
    # # draw boxes
    # bd.drawBoxes(img2, contours, found_colors)

    testVideo()
