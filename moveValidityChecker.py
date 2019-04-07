#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from my_baxter.msg import Board
from my_baxter.msg import FilteredBoard
from cv_bridge import CvBridge, CvBridgeError
from baxter_core_msgs.msg import DigitalIOState
from std_msgs.msg import Bool
import cv2
import zbar
import numpy as np
from Queue import Queue
import copy
import QRalternative as qr
import DiskNew as dk
import Board as bd
import validityFunctions as vf
print(cv2.__version__)
opencv_2 = cv2.__version__.startswith('2')


# initializations
# string_col = rospy.get_param("/hanoi_colors")
string_col = 'green,red,yellow'
hanoi_colors = string_col.split(',')
# string_col = 'pink,red,orange,yellow,green,dark_green'
initial_colors = hanoi_colors
previous_board = None
last_valid_board = None
missing_disk_color = None

# NOTE: if gameRunning is 0, initial colors should be reinitialized

# publishes move validity to validMove topic
# 1 = move is correct (cardinality kept, same disk colors across states) [probable]
# 2 = disk is missing (cardinality is not kept) [probable, if fault in CV or user removes]
# 3 = move is not correct
# 3.a cardinality kept, same disk colors across states, but more than 1 disk moves [probable, if fault in CV or user makes mistake]
# 3.b cardinality kept, but different disk colors across states) [probable, if fault in CV]

# publishes if ove validity code (1,2 or 3)
validMovePub = rospy.Publisher('/hanoi/validMove',Int8)

# publishes last valid board to aid the player when he makes a mistake
lastValidBoardPub = rospy.Publisher('/hanoi/lastValidBoard',FilteredBoard)


def addBackColors(added_disk_color,hanoi_colors,initial_colors):
    indexes = []
    for col in initial_colors:
        # if color exists in current set of color or those that are put back
        if (col in hanoi_colors) or (col in added_disk_color):
            # find its index in initial lit of colors
            ind = initial_colors.index(col)
            indexes.append(ind)
    updated_colors = [None] * len(indexes)
    # rebuild the list of colors to contain new colors as well
    for ind in indexes:
        updated_colors[ind] = initial_colors[ind]
    return updated_colors


# if a disk is missing (after Z+X time), update the colors to look for
# the missing disk will be ignored for the rest of the game
# CV will still see it, but it will not be modeled by this node
def callbackDeleteColors(data):
    global hanoi_colors,current_board,last_valid_board
    # if /hanoi/missingDisk is true
    if data.data:
        new_colors = vf.getDiskColor(current_board)     # as set (order of colors not kept)
        new_colors = list(new_colors)
        temp_colors = []
        print("Colors before: ",hanoi_colors)
        for col in hanoi_colors:
            if col in new_colors:
                temp_colors.append(col)
        hanoi_colors = temp_colors      # ordered colors
        print("Colors after: ", hanoi_colors)

        # print("Colors before: ", hanoi_colors)
        # for col in missing_disk_color:
        #     hanoi_colors.remove(col)
        # print("Colors after: ", hanoi_colors)

        last_valid_board = current_board    # update last valid board to have less colors
        lastValidBoardPub.publish(last_valid_board)


def callbackCheck(data):
    global previous_board,current_board,last_valid_board,missing_disk_color,hanoi_colors

    # if we are at the beginning, initialize previous_board and last_valid_board
    if previous_board is None:
        previous_board = data
        last_valid_board = data
        lastValidBoardPub.publish(last_valid_board)
        print('previos board none')
    # update current board state
    current_board = data

    # CASE I: cardinality is kept
    if len(current_board.encoded) == len(previous_board.encoded):

        # check if there is difference in disk colors across states
        previous_colors = vf.getDiskColor(previous_board)
        current_colors = vf.getDiskColor(current_board)
        color_diff = vf.diskColorDifference(previous_colors,current_colors)

        # CASE Ia: differing disk colors  [case 3.b]
        if len(color_diff) != 0:
            print('Warn user, publish 3')
            validMovePub.publish(3) # maybe model it differently/atm modelled as fault in player move
        # CASE Ib: same disk colors
        else:
            # compute the number of moves of each disk from previous to current state
            moves = vf.numDisksMoved(previous_board,current_board)
            # detect which moved
            name_moved = vf.whichMoved(moves)

            # CASE Ib 1: only one disk moved [THIS SHOULD HAPPEN]
            if len(name_moved) == 1:
                print('Ok, Checking positions')
                valid_move = vf.checkMove(name_moved[0],current_board)
                print(valid_move)

                # CASE Ib 1a: the move is valid [THIS SHOULD HAPPEN]
                if valid_move:
                    print('Congrats,move ok, publish 1')
                    last_valid_board = current_board    # update last_valid_board
                    validMovePub.publish(1)     # tell Game AI node that it can compute next move
                    lastValidBoardPub.publish(last_valid_board)

                # CASE Ib 1b: the move is not valid, inform user to go back to last valid state
                else:
                    print('MOVE NOT OK, publish 3')
                    validMovePub.publish(3)

            # CASE Ib 2: more than one disk moved [case 3.a]
            elif len(name_moved) > 1 :
                print("Looks like " + str(len(name_moved)) + " disks moved position. This is not valid. publish 3")
                validMovePub.publish(3)

            # CASE: nothing changed
            else:
                # board states are the same
                print('Started recording changes in board states')

    # CASE II: cardinality is not kept - a disk is missing [case 2]
    elif len(current_board.encoded) < len(previous_board.encoded):
        print('Missing disk, should publish 2')
        previous_colors = vf.getDiskColor(previous_board)
        current_colors = vf.getDiskColor(current_board)
        color_diff = vf.diskColorDifference(previous_colors, current_colors)
        missing_disk_color = list(color_diff)
        validMovePub.publish(2)

    # CASE III: cardinality is not kept - a disk is added
    else:
        # detect which disk colors are added
        previous_colors = vf.getDiskColor(previous_board)
        current_colors = vf.getDiskColor(current_board)
        color_diff = vf.diskColorDifference(previous_colors, current_colors)
        added_disk_color = list(color_diff)[0]  # assumes that only one is added
        print('Added disk. Check if correct position')

        # check if move is valid
        valid_move = vf.checkMove(added_disk_color, current_board)

        # add disk back to list
        print("Colors before: ", hanoi_colors)
        updated_colors = addBackColors(added_disk_color, hanoi_colors, initial_colors)
        hanoi_colors = updated_colors
        print("Colors after: ", hanoi_colors)

        # CASE IIIa : the move is valid [THIS SHOULD HAPPEN]
        if valid_move:
            print('Congrats,move ok, publish 1')
            last_valid_board = current_board  # update last_valid_board
            validMovePub.publish(1)  # tell Game AI node that it can compute next move
            lastValidBoardPub.publish(last_valid_board)

        # CASE IIIb: the move is not valid, inform user to go back to last valid state
        else:
            print('MOVE NOT OK, publish 3')
            validMovePub.publish(3)


    # when all checks are done, previous_board is the new board
    previous_board = current_board


if __name__ == "__main__":

    # create node
    rospy.init_node('moveValidityChecker', anonymous=True)

    # create subscriber to the filtered board topic
    filteredBoardSub = rospy.Subscriber('/hanoi/filteredBoardState', FilteredBoard, callbackCheck)

    # create subscriber to the missing disk topic
    missingDiskSub = rospy.Subscriber('/hanoi/missingDisk', Bool, callbackDeleteColors)

    #prevents program from exiting, allowing subscribers and publishers to keep operating
    #in our case that is the camera subscriber and the image processing callback function
    rospy.spin()
