import cv2
print(cv2.__version__)
import numpy as np
import  os
opencv_2 = cv2.__version__.startswith('2')
import blobDetection as bd

# img = cv2.imread('tower.jpg')
# bd.getBoundingBoxes(img,bd.hsv_manual,'hsv')


cap = cv2.VideoCapture(0)
while(True):

   # delay
    for i in range(20):
        print('')

    ret, img = cap.read()
    bd.getBoundingBoxes(img, bd.hsv_manual, 'hsv')
    cv2.imshow('frame', img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()


