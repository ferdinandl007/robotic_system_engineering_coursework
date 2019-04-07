#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from baxter_core_msgs.msg import DigitalIOState

import cv2
print(cv2.__version__)
opencv_2 = cv2.__version__.startswith('2')

# initializations
Z_SECS = 5      # seconds to wait for user to put disk back VOLUNTARILY
X_SECS = 5      # seconds to wait for user to put disk back AFTER BEING ASKED
wrong_moves = 0
valid_move = None   # will hold value from topic
timer_Z_started = False
timer_X_started = False
feedback_string = "Missing disk. "+str(X_SECS)+" seconds to put it back."

# will publish the number of wrong moves made by the player (since we entered this node)
wrongMovesPub = rospy.Publisher('/hanoi/wrongMoves',Int8)

# will publish to speech node
feedbackPub = rospy.Publisher('/hanoi/userFeedback',String)

# will publish to missing disk topic
missingDiskPub = rospy.Publisher('/hanoi/missingDisk',Bool)


# update the number of wrong moves, if it was changed by another node (Incorrect move)
def callbackWrongMoves(data):
    global wrong_moves
    wrong_moves = data.data


# X_SECS after player wa asked to put back disk, check if disk back
def callbackTimerX(event):
    global valid_move,wrong_moves,timer_Z_started,timer_X_started

    # after X_SECS, disk is still not back
    if valid_move == 2:
        print('Disk is not back')
        # update number of wrong moves
        wrong_moves += 1
        wrongMovesPub.publish(wrong_moves)

        # tell user we continue without the disk
        feedback_string_continue = "Continuing without the missing disk"
        feedbackPub.publish(feedback_string_continue)

        # tell nodes a disk is missing
        missingDiskPub.publish(True)

    # if a correct/incorrect move has been made, reset flags
    timer_X_started = False
    timer_Z_started = False


# Z_SECS after missing disk is detected, it will check if there was another move
def callbackTimerZ(event):
    global valid_move,timer_X_started

    # if an incorrect or correct move has been made in the meantime, don't do anything
    if valid_move == 3 or valid_move == 1:
        print('Do nothing. User made wrong move (dealt with by incorrectmoveAlg) or corrected move')
        # MAYBE set timer_Z_started = False

    # if disk still missing
    else:
        # tell user to put it back
        print('Tell player to put disk back')
        feedbackPub.publish(feedback_string)
        # start another timer X to allow player to put it back
        rospy.Timer(rospy.Duration(X_SECS), callbackTimerX, oneshot=True)  # oneshot to fire only once
        timer_X_started = True


def callbackMissing(data):
    global valid_move,timer_Z_started
    valid_move = data.data

    # disk is missing and timer to allow to put back VOLUNTARILY has not started
    if valid_move == 2 and (not timer_Z_started):

        # start a timer Z to allow user to put back by himself
        rospy.Timer(rospy.Duration(Z_SECS), callbackTimerZ, oneshot=True) # oneshot to fire only once
        timer_Z_started = True


if __name__ == "__main__":

    # create node
    rospy.init_node('missingDiskAlgorithm', anonymous=True)

    # create subscriber to the valid move topic
    validMoveSub = rospy.Subscriber('/hanoi/validMove', Int8, callbackMissing)

    # create subscriber to the wrong moves topic
    wrongMovesSub = rospy.Subscriber('/hanoi/wrongMoves', Int8, callbackWrongMoves)


    #prevents program from exiting, allowing subscribers and publishers to keep operating
    #in our case that is the camera subscriber and the image processing callback function
    rospy.spin()
