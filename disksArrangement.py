# Standard imports
import cv2
print(cv2.__version__)
import numpy as np
import  os
opencv_2 = cv2.__version__.startswith('2')




def getDiskInd(col, disks_list):
    for i,e in enumerate(disks_list):
        if e.color == col:
            return i
    return -1   # for colorname not found


def boardChanged(old_board, new_board):
    """Compares the states of the new and olf board. Returns True if they are the same, False otherwise"""

    # compare lengths of towers
    if len(old_board.left) != len(new_board.left):
        return True

    if len(old_board.middle) != len(new_board.middle):
        return True

    if len(old_board.right) != len(new_board.right):
        return True

    # compare colors within towers
    for a,b in zip(old_board.left,new_board.left):
        if a.disk_color != b.disk_color:
            return True
    for a,b in zip(old_board.middle,new_board.middle):
        if a.disk_color != b.disk_color:
            return True
    for a,b in zip(old_board.right,new_board.right):
        if a.disk_color != b.disk_color:
            return True

    # if we get to this point, towers haven't changed
    return False
