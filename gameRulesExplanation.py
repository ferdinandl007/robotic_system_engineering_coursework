#!/usr/bin/env python

import rospy
import baxter_interface
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


# will publish to speech node to explain to user game rules
feedbackPub = rospy.Publisher('/hanoi/userFeedback',String)


game_rules_string = "This is the game explanation string."

def sendGameRulesExplanation(v):
    # if button is pressed
    if v == True:
        print('button pushed')
        feedbackPub.publish(game_rules_string)



if __name__ == "__main__":

    # create node
    rospy.init_node('gameRulesExplanation', anonymous=True)

    # create subscriber to the left arm button
    leftArmNav = baxter_interface.Navigator('left')

    # check for left arm button 1 press for game rules explanation
    leftArmNav.button0_changed.connect(sendGameRulesExplanation)

    #prevents program from exiting, allowing subscribers and publishers to keep operating
    #in our case that is the camera subscriber and the image processing callback function
    rospy.spin()
