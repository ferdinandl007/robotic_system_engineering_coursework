#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Int8
from cv_bridge import CvBridge, CvBridgeError
from baxter_core_msgs.msg import DigitalIOState
from std_msgs.msg import Bool
import cv2
print(cv2.__version__)
opencv_2 = cv2.__version__.startswith('2')


X_SECS = 5     # seconds to wait for move correction
wrong_moves = 0
valid_move = None
timer_X_started = False
timerX = None   # timer to allow correction
feedback_string = "Invalid move. "+str(X_SECS)+" seconds to correct it."

# will publish the number of wrong moves made by the player (since we entered this node)
wrongMovesPub = rospy.Publisher('/hanoi/wrongMoves',Int8)

# will stop game if move is not corrected within an amount of time
gameRunningPub = rospy.Publisher('/hanoi/gameRunning',Int8)

# will publish to speech node to tell user how much time he has
feedbackPub = rospy.Publisher('/hanoi/userFeedback',String)


# update the number of wrong moves, if it was changed by another node (Missing disk)
def callbackWrongMoves(data):
    global wrong_moves
    wrong_moves = data.data


# after X_SECS, it will check if there was another move made
# if move is still incorrect, will stop the game
def callbackTimer(event):
    global valid_move,timer_X_started,timerX,wrong_moves
    if valid_move == 3:
        print('Move not corrected. Stopping game')
        gameRunningPub.publish(3)   # stop game (allow for setup)
        # clear all variables
        timer_X_started = False
        timerX = None
        wrong_moves = 0
        valid_move = None


def callbackMove(data):
    global valid_move,wrong_moves,timer_X_started, timerX
    valid_move = data.data

    # if move is incorrect and no timer was started yet
    if valid_move == 3 and (not timer_X_started):

        # update number of wrong moves
        wrong_moves += 1
        wrongMovesPub.publish(wrong_moves)

        # warn user that he has X_SECS to correct it and start timer
        print('Player has '+str(X_SECS)+' seconds to correct it')
        feedbackPub.publish(feedback_string)
        # maybe sleep here
        timerX = rospy.Timer(rospy.Duration(X_SECS), callbackTimer, oneshot=True)
        timer_X_started = True

    # if move is correct/missing and timer was started previously (due to incorrect move)
    elif valid_move != 3 and timer_X_started and timerX is not None:
        # top timer
        timerX.shutdown()
        timer_X_started = False


if __name__ == "__main__":

    # create node
    rospy.init_node('incorrectMoveAlgorithm', anonymous=True)

    # create subscriber to the valid move topic
    validMoveSub = rospy.Subscriber('/hanoi/validMove',Int8,callbackMove)

    # create subscriber to the wrong moves topic
    wrongMovesSub = rospy.Subscriber('/hanoi/wrongMoves',Int8,callbackWrongMoves)


    #prevents program from exiting, allowing subscribers and publishers to keep operating
    #in our case that is the camera subscriber and the image processing callback function
    rospy.spin()
