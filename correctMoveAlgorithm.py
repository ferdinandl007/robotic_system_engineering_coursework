#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
from cv_bridge import CvBridge, CvBridgeError
from baxter_core_msgs.msg import DigitalIOState
import cv2

print(cv2.__version__)
opencv_2 = cv2.__version__.startswith('2')


feedback_string = "Congratulations"

# will publish to speech node to tell user how much time he has
feedbackPub = rospy.Publisher('/hanoi/userFeedback',String)


def callbackMove(data):
    valid_move = data.data

    # if move is correct
    if valid_move == 1 :
        print(feedback_string)
        feedbackPub.publish(feedback_string)


if __name__ == "__main__":

    # create node
    rospy.init_node('correctMoveAlgorithm', anonymous=True)

    # create subscriber to the valid move topic
    validMoveSub = rospy.Subscriber('/hanoi/validMove', Int8, callbackMove)

    #prevents program from exiting, allowing subscribers and publishers to keep operating
    #in our case that is the camera subscriber and the image processing callback function
    rospy.spin()
