# Standard imports
import cv2
print(cv2.__version__)
import numpy as np
import  os
opencv_2 = cv2.__version__.startswith('2')




def printBoard(board):
    print('Left tower:')
    for l in reversed(board.left_tower):
        print('['+l.color+'] ')
    print('Middle tower:')
    for m in reversed(board.middle_tower):
        print('['+m.color+'] ')
    print('Right tower:')
    for r in reversed(board.right_tower):
        print('['+r.color+'] ')
    print('########################################')


def getDiskInd(col, disks_list):
    for i,e in enumerate(disks_list):
        if e.color == col:
            return i
    return -1   # for colorname not found


def updateDetectedDisks(all_disks, centers_x, centers_y, widths, heights, rotations, found_colors):
    print(len(found_colors))
    for c in range(len(found_colors)):
        index_disk = getDiskInd(found_colors[c], all_disks) # get the index of disk in the initial list
        all_disks[index_disk].center_x = centers_x[c]
        all_disks[index_disk].center_y = centers_y[c]
        all_disks[index_disk].width = widths[c]
        all_disks[index_disk].height = heights[c]
        all_disks[index_disk].rotation = rotations[c]
    return all_disks


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



    print('')
    array_disks_pos = []
    array_colors = []
    array_ids = []
    for f in board.left_tower:
        array_disks_pos.append(0)
        array_colors.append(f.color)
        array_ids.append(f.id)
    for m in board.middle_tower:
        array_disks_pos.append(1)
        array_colors.append(m.color)
        array_ids.append(m.id)
    for r in board.right_tower:
        array_disks_pos.append(2)
        array_colors.append(r.color)
        array_ids.append(r.id)


    print('')
    return board,array_disks_pos, array_colors, array_ids



def boardChanged(old_board, new_board):
    """Compares the states of the new and olf board. Returns True if they are the same, False otherwise"""

    # compare lengths of towers
    if len(old_board.left_tower) != len(new_board.left_tower):
        return True

    if len(old_board.middle_tower) != len(new_board.middle_tower):
        return True

    if len(old_board.right_tower) != len(new_board.right_tower):
        return True

    # compare colors within towers
    for a,b in zip(old_board.left_tower,new_board.left_tower):
        if a.color != b.color:
            return True
    for a,b in zip(old_board.middle_tower,new_board.middle_tower):
        if a.color != b.color:
            return True
    for a,b in zip(old_board.right_tower,new_board.right_tower):
        if a.color != b.color:
            return True

    # if we get to this point, towers haven't changed
    return False
