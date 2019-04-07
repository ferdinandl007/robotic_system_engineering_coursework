#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Int8
from my_baxter.msg import Moves
from my_baxter.msg import FilteredBoard
from cv_bridge import CvBridge, CvBridgeError
from baxter_core_msgs.msg import DigitalIOState
from std_msgs.msg import Bool

left_tower = 0
middle_tower = 1
right_tower = 2
moves_msg = Moves()     # will hold the moves needed to be done by the player

def moveDisks(diskPositions, largestToMove, targetPeg):
    global moves_msg
    for badDisk in range(largestToMove, len(diskPositions)):

        currentPeg = diskPositions[badDisk]
        if currentPeg != targetPeg:
            #found the largest disk on the wrong peg

            #sum of the peg numbers is 3, so to find the other one...
            otherPeg = 3 - targetPeg - currentPeg

            #before we can move badDisk, we have get the smaller ones out of the way
            moveDisks(diskPositions, badDisk+1, otherPeg)

            print "Move ", badDisk, " from ", currentPeg, " to ", targetPeg
            move_string = "Move "+str(badDisk)+" from "+str(currentPeg)+" to "+str(targetPeg)
            moves_msg.moves.append(move_string)
            diskPositions[badDisk]=targetPeg

            #now we can put the smaller ones in the right place
            moveDisks(diskPositions, badDisk+1, targetPeg)

            break



# will publish to speech node to tell user where to move what
nextMovePub = rospy.Publisher('/hanoi/nextMove',Moves)


last_encoded_board = None
def callbackComputeMove(data):
    global last_encoded_board,moves_msg
    moves_msg = Moves()    # reinitialize moves
    last_encoded_board = list(data.encoded) # [1st largest disk pos, 2nd largest, ....]

    # if new data (valid board state) is received, compute the next moves
    # solution will be first string in the moves array
    moveDisks(last_encoded_board, left_tower, right_tower)

    # nextMovePub.publish(next_move)
    nextMovePub.publish(moves_msg)


if __name__ == "__main__":

    # create node
    rospy.init_node('gameController', anonymous=True)

    # create subscriber to the valid move topic
    lastValidSub = rospy.Subscriber('/hanoi/lastValidBoard', FilteredBoard, callbackComputeMove)


    #prevents program from exiting, allowing subscribers and publishers to keep operating
    #in our case that is the camera subscriber and the image processing callback function
    rospy.spin()
