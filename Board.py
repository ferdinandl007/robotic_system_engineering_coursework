class Board:
    def __init__(self):
        self.left_tower = []  # should behave as a stack
        self.middle_tower = []
        self.right_tower = []

        # use list.append(x) and list.pop()