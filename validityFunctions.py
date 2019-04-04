
'''Returns the color of disks on the board'''
def getDiskColor(board):
    colors = set()
    for l in board.left:
        colors.add(l.disk_color)
    for m in board.middle:
        colors.add(m.disk_color)
    for r in board.right:
        colors.add(r.disk_color)
    return colors


'''Returns which disks belong to only one or the other board'''
def diskColorDifference(previous_colors,current_colors):
    # symmetric difference: the elements belonging to either A or B, but not to both sets simultaneously
    diff = current_colors.symmetric_difference(previous_colors)
    return diff


''''''
def getPositions(board):
    positions = {}
    for i in range(len(board.left)):
        positions[board.left[i].disk_color] = [i,0]
    for j in range(len(board.middle)):
        positions[board.middle[j].disk_color] = [j,1]
    for k in range(len(board.right)):
        positions[board.right[k].disk_color] = [k, 2]
    return positions


'''For invlaid move where user moves 2 disks'''
def numDisksMoved(previous_board,current_board):
    # NOTE: assumes that the same colors exist in both boards
    previous_positions = getPositions(previous_board)
    current_positions = getPositions(current_board)
    moves = {}

    for color in previous_positions.keys():
        moved = False
        prev = previous_positions.get(color)
        curr = current_positions.get(color)

        # check if disk moved ON THE POLE or moved FROM A POLE TO ANOTHER
        if (prev[0] != curr[0]) or (prev[1] != curr[1]):
            moved = True

        # update number of moves for disk
        moves[color] = moved

    return moves


def whichMoved(moves):
    name_moved = []
    for color,moved in moves.items():
        if moved:
            name_moved.append(color)
    return name_moved


"""Checks if the destination tower is in order"""
def checkMove(color_moved_disk,current_board):
    check_tower = None
    if any(x.disk_color == color_moved_disk for x in current_board.left):
        check_tower = 0
    if any(x.disk_color == color_moved_disk for x in current_board.middle):
        check_tower = 1
    if any(x.disk_color == color_moved_disk for x in current_board.right):
        check_tower = 2

    sorted_sizes = []
    sizes = []
    if check_tower == 0:
        for l in current_board.left:
            sizes.append(l.size)
    if check_tower == 1:
        for m in current_board.middle:
            sizes.append(m.size)
    if check_tower == 2:
        for r in current_board.right:
            sizes.append(r.size)

    # sort in reverse order (largest disk first, smallest disk last)
    sorted_sizes = sorted(sizes, reverse = True)
    return sizes == sorted_sizes

