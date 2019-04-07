#!/usr/bin/env python

import rospy
import sys
from my_baxter.msg import Board
from my_baxter.msg import FilteredBoard
from my_baxter.msg import Disk
from cv_bridge import CvBridge, CvBridgeError
from baxter_core_msgs.msg import DigitalIOState
from std_msgs.msg import Bool
import cv2
import numpy as np
import copy
import disksArrangement as da
import Board as bd
from Queue import Queue

print(cv2.__version__)
opencv_2 = cv2.__version__.startswith('2')


# initializations
MAX_NUM_DISKS = 10      # maximum number of disks to play with
# string_col = rospy.get_param("/hanoi_colors")
string_col = 'green,red,yellow'
hanoi_colors = string_col.split(',')
# string_col = 'pink,red,orange,yellow,green,dark_green'
MAX_WINDOW_SIZE = 10     # the number of states across which votes for position are counted
MIN_NUM_VOTES = 7       # minimum number of votes for a position to be valid
board_data_history = Queue(maxsize=MAX_WINDOW_SIZE)     # will hold the states (FIFO)
previous_board = None
current_board = None
publish_timer = None    # timer to count delay
PUBLISH_DELAY = 2   # delay with which we publish in seconds

# publisher to which filtered (and stable) board state
filteredboardPub = rospy.Publisher('/hanoi/filteredBoardState',FilteredBoard)

# function to count votes for positions of disks
# returns the most voted position for each disk color if larger than minimum number of votes
def countVotes(board_data_history, MIN_NUM_VOTES):
    history = copy.deepcopy(board_data_history.queue)   # the states
    colors = set()  # all colors detected across the states
    votes = {}

    # count votes across board states
    while len(history) != 0:
        data = history.pop()
        # diskcolor-positiononpole-polenumber

        # voting for left tower disks
        for index,l in enumerate(data.left):
            code = l.disk_color+"-"+str(index)+"-"+str(0)
            colors.add(l.disk_color)
            if votes.get(code) is None:
                votes[code] = 1
            else:
                votes[code] += 1
        # voting for middle tower disks
        for index,m in enumerate(data.middle):
            code = m.disk_color+"-"+str(index)+"-"+str(1)
            colors.add(m.disk_color)
            if votes.get(code) is None:
                votes[code] = 1
            else:
                votes[code] += 1
        # votes for right tower disks
        for index,r in enumerate(data.right):
            code = r.disk_color+"-"+str(index)+"-"+str(2)
            colors.add(r.disk_color)
            if votes.get(code) is None:
                votes[code] = 1
            else:
                votes[code] += 1

    # find most probable position of a color
    best_voted = []
    for color in colors:
        max_color_votes = 0
        most_voted_position = ""
        for k in votes.keys():
            if color in k:
                # save the position with largest number of votes so far
                if votes.get(k) > max_color_votes:
                    max_color_votes = votes.get(k)
                    most_voted_position = k
        if max_color_votes >= MIN_NUM_VOTES:
            best_voted.append(most_voted_position)

    # return colors and positions
    return best_voted


# returns the encoded state of the current board state as array
# [post 1st largest, pos 2nd largest, ...]
def getEncoding(current_board,MAX_NUM_DISKS):
    encoded_board = bd.Board()  # make board class
    # update it
    encoded_board.left_tower = current_board.left
    encoded_board.middle_tower = current_board.middle
    encoded_board.right_tower = current_board.right
    encoded_board = encoded_board.getBoardEncoding(MAX_NUM_DISKS)
    return encoded_board


# publish current board state
def callbackPublish(event):
    global current_board, previous_board
    print('publish now')
    filteredboardPub.publish(current_board)


# starts a new timer each time a change in board is detected
# after PUBLISH_DELAY seconds, it will call the callbackPublish which publishes filtered board state
# publishing will take place if no other board change has been detected in PUBLISH_DELAY seconds
def startTimerForPublish(delay):
    global publish_timer

    # start timer if it was not started
    if publish_timer is None:
        publish_timer = rospy.Timer(rospy.Duration(delay), callbackPublish,oneshot=True)
    # start timer again (stop and start new)
    else:
        publish_timer.shutdown()
        publish_timer = rospy.Timer(rospy.Duration(delay), callbackPublish,oneshot=True)


def callbackBoardState(data):
    global board_data_history,previous_board,current_board,PUBLISH_DELAY

    # accumulate 5 states at the beginning
    # or whenever we just removed the oldest state
    if board_data_history.qsize() < MAX_WINDOW_SIZE:
        board_data_history.put(data)  # start adding to history

    # if 5 states have been accumulated, start averaging states
    if board_data_history.qsize() == MAX_WINDOW_SIZE:

        # find most voted position for each color
        print('Doing averaging...')
        best_voted = countVotes(board_data_history, MIN_NUM_VOTES)

        # populate towers based on most voted positions
        left_tower_temp = []
        middle_tower_temp = []
        right_tower_temp = []
        for voted in best_voted:
            # diskcolor-positiononpole-polenumber
            info = voted.split('-')
            info[1] = int(info[1])
            info[2] = int(info[2])
            if info[2] == 0:
                left_tower_temp.append(info)
            if info[2] == 1:
                middle_tower_temp.append(info)
            if info[2] == 2:
                right_tower_temp.append(info)

        # order them by their position on pole
        left_tower = sorted(left_tower_temp, key=lambda disk: disk[1])
        middle_tower = sorted(middle_tower_temp, key=lambda disk: disk[1])
        right_tower = sorted(right_tower_temp, key=lambda disk: disk[1])

        # make current board message
        current_board = FilteredBoard()
        for l in left_tower:
            disk = Disk(l[0], hanoi_colors.index(l[0])+1)   # Disk(color,size)
            current_board.left.append(disk)
        for m in middle_tower:
            disk = Disk(m[0], hanoi_colors.index(m[0])+1)
            current_board.middle.append(disk)
        for r in right_tower:
            disk = Disk(r[0], hanoi_colors.index(r[0])+1)
            current_board.right.append(disk)

        # get encoding of boards
        encoded_board = getEncoding(current_board, MAX_NUM_DISKS)
        print("Encoded board:", encoded_board)
        current_board.encoded = encoded_board

        # if there isn't any previous board (we just started), initialize it
        if previous_board is None:
            previous_board = current_board
            startTimerForPublish(PUBLISH_DELAY)     # publish with delay

        # if the board has changed from None to a state, or from a state to a state
        else:
            if da.boardChanged(previous_board, current_board):
                previous_board = current_board          # update previous board
                startTimerForPublish(PUBLISH_DELAY)     # publish with delay
        # else: board has not changed, don't do anything

        # remove the oldest board state from queue when done
        print('Dequeing')
        board_data_history.get()


if __name__ == "__main__":

    # create node
    rospy.init_node('boardChangeController', anonymous=True)

    # create subscriber to the board state topic
    boardStateSub = rospy.Subscriber('/hanoi/boardState', Board, callbackBoardState)

    #prevents program from exiting, allowing subscribers and publishers to keep operating
    #in our case that is the camera subscriber and the image processing callback function
    rospy.spin()
