

moves = []
def moveDisks(diskPositions, largestToMove, targetPeg):
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
            moves.append(move_string)
            diskPositions[badDisk]=targetPeg

            #now we can put the smaller ones in the right place
            moveDisks(diskPositions, badDisk+1, targetPeg)

            break


if __name__ == "__main__":
    moveDisks([2, 1, 0, 2], 0, 2)
    print(moves)



