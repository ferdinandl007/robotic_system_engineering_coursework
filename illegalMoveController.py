#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Int8
from hanoi.msg import FilteredBoard
from cv_bridge import CvBridge, CvBridgeError
from baxter_core_msgs.msg import DigitalIOState
from std_msgs.msg import Bool
import cv2
print(cv2.__version__)
opencv_2 = cv2.__version__.startswith('2')


ALLOWED_WRONG_MOVES = 5

# will publish to speech node to tell user where to move what
feedbackPub = rospy.Publisher('/hanoi/userFeedback',String)


# will publish to the game state that game has ended
gameRunningPub1 = rospy.Publisher('/hanoi/gameRunning',Int8)

game_ended_string = "Game over"

def callbackWrongMoveCount(data):
    global ALLOWED_WRONG_MOVES

    # when the number of allowed wrong moves is passed, terminate game
    if data.data > (ALLOWED_WRONG_MOVES+1):

        # first warn user that the game is over
        feedbackPub.publish(game_ended_string)

        # publish game end
        gameRunningPub1.publish(0)

        print('')



if __name__ == "__main__":

    # create node
    rospy.init_node('illegalMoveController', anonymous=True)

    # create subscriber to the valid move topic
    wrongMovesSub = rospy.Subscriber('/hanoi/wrongMoves', Int8, callbackWrongMoveCount)


    #prevents program from exiting, allowing subscribers and publishers to keep operating
    #in our case that is the camera subscriber and the image processing callback function
    rospy.spin()
