#!/usr/bin/env python

import rospy
import baxter_interface
from std_msgs.msg import String
from std_msgs.msg import Int8
from my_baxter.msg import FilteredBoard
from my_baxter.msg import Moves
from cv_bridge import CvBridge, CvBridgeError
from baxter_core_msgs.msg import DigitalIOState
from std_msgs.msg import Bool

left_tower = 0
middle_tower = 1
right_tower = 2


# decodes tower numbers to tower names for movement command
def decodeSolution(move_string):
    global left_tower,middle_tower,right_tower
    string = move_string.split(" ")
    disk_num = int(string[1])
    origin_tower_num = int(string[3])
    destination_tower_num = int(string[5])

    # decode origing
    command = "Move disk from "
    if origin_tower_num == left_tower:
        command = command+"left tower"
    elif origin_tower_num == middle_tower:
        command = command + "middle tower"
    else:
        command = command + "right tower"

    # decode destination
    command = command + " to "
    if destination_tower_num == left_tower:
        command = command + "left tower"
    elif destination_tower_num == middle_tower:
        command = command + "middle tower"
    else:
        command = command + "right tower"

    return command,origin_tower_num,destination_tower_num




# will publish to speech node to tell user where to move what
movementLAPub = rospy.Publisher('/hanoi/movementLA',String)

# will publish to speech node to tell user where to move what, once the button is pressed
feedbackPub = rospy.Publisher('/hanoi/userFeedback',String)


next_move = None
command = None
origin = None
destination = None

def callbackNextMove(data):
    global next_move,command,origin,destination
    try:
        next_move = data.moves[0]
        command,origin,destination = decodeSolution(next_move)
    except IndexError:
        # there are no more moves
        next_move = "END"
    except:
        # if there is another problem
        next_move = "PROBLEM"


def callbackHint(v):
    global next_move, command, origin, destination
    # if button is pressed
    if v == True:
        name_of_file = "movement"+str(origin)+"-"+str(destination)+".txt"
        movementLAPub.publish(name_of_file)
        feedbackPub.publish(command)


if __name__ == "__main__":

    # create node
    rospy.init_node('provideHintAlgorithm', anonymous=True)

    # create subscriber to the valid move topic
    movesSub = rospy.Subscriber('/hanoi/nextMove', Moves, callbackNextMove)

    # create subscriber to the left arm button
    leftArmNav = baxter_interface.Navigator('left')

    # check for left arm button 1 press for YES HINT
    leftArmNav.button1_changed.connect(callbackHint)


    #prevents program from exiting, allowing subscribers and publishers to keep operating
    #in our case that is the camera subscriber and the image processing callback function
    rospy.spin()
