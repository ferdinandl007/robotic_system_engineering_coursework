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


hanoi_colors = ['pink', 'red', 'orange', 'yellow', 'green', 'dark green', 'blue', 'dark blue']
hsv_offset = 3
bgr_baxter = {}
hsv_baxter = {}
preproc_calib_img = False
save = True
calib_button_press_1 = False
calib_button_press_2 = False
calib_done = False


def calibrationButtonCallback(key_press):
    global calib_button_press_1,calib_button_press_2,calib_done

    if calib_done:
        return
    if calib_button_press_1 == False and key_press == True:
        calib_button_press_1 = True
    if key_press == True and calib_button_press_1 == True:
        calib_button_press_2 = True


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

def testWebcam():
    preproc_calib_img = False
    save = True
    cap = cv2.VideoCapture(1)

    while(True):
        ret, frame = cap.read() # frame not returned

        bgr_baxter = bd.getBGRavgColorsFromBaxter(frame, hanoi_colors, preproc_calib_img, True)
        # hsv_baxter = bd.calibrateColors(bgr_baxter, hsv_offset)
        cv2.imshow('Original',frame)

        key = cv2.waitKey(1)
        if key == ord('c'):
            calibrationButtonCallback(True)

    cap.release()


if __name__=="__main__":


    cap = cv2.VideoCapture(1)

    while(True):
        ret, frame = cap.read() # frame not returned
        frame_draw = frame.copy()

        # if we haven't done color calibration
        if not calib_done:
            # if we pressed the key once (signal start of calibration process)
            if calib_button_press_1:
                bgr_baxter = bd.getBGRavgColorsFromBaxter(frame, hanoi_colors, preproc_calib_img, True)
                cv2.imshow('Draw', frame_draw)
            # if we pressed the key twice (signal the end of calibration process)
            if calib_button_press_2:
                hsv_baxter = bd.calibrateColors(bgr_baxter, hsv_offset)
                calib_done = True
                # cv2.destroyAllWindows()

        cv2.imshow('Original',frame)

        # if we have the dictionary of colors, the calibration was done
        # then we can start detecting color blobs
        if len(hsv_baxter) != 0:
            centers_x,centers_y,widths,heights,rotations,found_colors,contours = bd.getBoundingBoxes(frame_draw,
                                                                                                     hsv_baxter,
                                                                                                     hanoi_colors)
            bd.drawBoxes(frame, contours, found_colors)


        key = cv2.waitKey(1)
        if key == ord('c'):
            calibrationButtonCallback(True)

    cap.release()



    # img = cv2.imread('centeredTower.png')
    # img2 = cv2.imread('centeredTower.png')
    # preproc_calib_img = False
    # # hsv offset needs to be adjusted depending on the colors used (ex: smaller if both blue and dark blue used)
    # hsv_offset = 3
    # save blob masks in folder True
    # bgr_baxter = bd.getBGRavgColorsFromBaxter(img,hanoi_colors,preproc_calib_img,True)
    # hsv_baxter = bd.calibrateColors(bgr_baxter,hsv_offset)

    # 2) blobs detection

    # # preprocess frames if needed
    # img = bd.preprocessImage(img)

    # # get bounding box infoq
    # centers_x,centers_y,widths,heights,rotations,found_colors,contours = bd.getBoundingBoxes(img, hsv_baxter, hanoi_colors)
    # # draw boxes
    # bd.drawBoxes(img2, contours, found_colors)

    # testVideo()
    # bgr_baxter = testWebcam()
    # hsv_baxter = bd.calibrateColors(bgr_baxter, hsv_offset)
    # print('')