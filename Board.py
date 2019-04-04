class Board:
    def __init__(self):
        self.left_tower = []
        self.middle_tower = []
        self.right_tower = []
        # use list.append(x) and list.pop() to bahave as stack


    # def updateUsedColors(self,curr_col):
    #     self.current_colors = curr_col


    def printBoard(self):
        print('Left tower:')
        for l in reversed(self.left_tower):
            print('[' + l.color + '] ')
        print('Middle tower:')
        for m in reversed(self.middle_tower):
            print('[' + m.color + '] ')
        print('Right tower:')
        for r in reversed(self.right_tower):
            print('[' + r.color + '] ')
        print('########################################')



    def determineBoardLayout(self, bounds, disks):
        # cv2.imshow('',img)
        # print(img.shape) # height, whidth, num channels

        # image info
        # width = bounds[1].coord_x - bounds[0].coord_x
        width = 640 # temporarily
        segment_size = width / 3

        # determine to what tower each disk belongs
        tower_left = []
        tower_middle = []
        tower_right = []

        # arrange disks on towers, but not in decreasing order yet
        for disk in disks:

            # disk belonds to left tower
            if disk.center_x < segment_size:
                tower_left.append(disk)
            # disk belongs to middle tower
            elif disk.center_x < 2 * segment_size:
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
            self.left_tower.append(d)
        for d in tower_middle:
            self.middle_tower.append(d)
        for d in tower_right:
            self.right_tower.append(d)


    # returns 0 if disk on left tower, 1 on middle, 2 on right
    def getTowerIndex(self, disk_size):
        # disk is on left tower
        for l in self.left_tower:
            if l.size == disk_size:
                return 0
        # disk is on middle tower
        for m in self.middle_tower:
            if m.size == disk_size:
                return 1
        # disk is on right tower
        for r in self.right_tower:
            if r.size == disk_size:
                return 2
        # if nowhere
        return -1


    # returns [pos 1st largest disk, pos 2nd largest,...]
    def getBoardEncoding(self,MAX_NUM_DISKS):
        structure = []
        for s in range(MAX_NUM_DISKS-1,-1,-1):
            ind = self.getTowerIndex(s)
            if ind != -1:
                structure.append(ind)
        return structure