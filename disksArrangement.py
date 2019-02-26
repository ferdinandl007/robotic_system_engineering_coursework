# Standard imports
import cv2
print(cv2.__version__)
import numpy as np
import  os
import Disk
import Board
opencv_2 = cv2.__version__.startswith('2')


def make_disks(centers_x,centers_y,widths,heights,rotations,found_colors):
    disks = [] # disk objects will be arranged in the order of found_colors
    for d in range(len(centers_x)):
        disks.append(Disk.Disk(centers_x[d],centers_y[d],widths[d],heights[d],rotations[d],found_colors[d]))
    return disks



def getBoardLayout(img, disks, board):
    # cv2.imshow('',img)
    # print(img.shape) # height, whidth, num channels

    # image info
    width = img.shape[1]
    segment_size = width/3

    # determine to what tower each disk belongs
    tower_left = []
    tower_middle = []
    tower_right = []

    # arrange disks on towers, but not in decreasing order yet
    for disk in disks:
        # use QR code result to detect positions!

        # disk belonds to left tower
        if disk.center_x < segment_size:
            tower_left.append(disk)
        # disk belongs to middle tower
        elif disk.center_x < 2*segment_size:
            tower_middle.append(disk)
        # disk belongs to right tower
        else:
            tower_right.append(disk)

    # sort the disks based on y coordinate
    # which is on top of which
    tower_left = sorted(tower_left, key=lambda disk: disk.center_y, reverse=True)
    tower_middle = sorted(tower_middle, key=lambda disk: disk.center_y, reverse=True)
    tower_right = sorted(tower_right, key=lambda disk: disk.center_y, reverse=True)

    # make board object
    for d in tower_left:
        board.left_tower.append(d)
    for d in tower_middle:
        board.middle_tower.append(d)
    for d in tower_right:
        board.right_tower.append(d)

    return board