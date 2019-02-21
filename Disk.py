class Disk:
    def __init__(self,cx,cy,w,h,rot,col):
        self.center_x = cx
        self.center_y = cy
        self.width = w
        self.height = h
        self.rotation = rot
        self.color = col


    def getInfo(self):
        print('hello')