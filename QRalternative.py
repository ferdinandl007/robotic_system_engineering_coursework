import zbar
import numpy as np
import cv2


def getCodes(image):
    # order of coordinates: left-up, left-down, right-down, right-up (corners)
    codes_coordinates = []
    for symbol in image:
        # print('decoded',symbol.type,'symbol','"%s"' % symbol.data)
        # left_up = symbol.location[0]
        # left_down = symbol.location[1]
        # right_down = symbol.location[2]
        # right_up = symbol.location[3]
        if symbol.type == symbol.QRCODE:
            codes_coordinates.append(symbol.location)
    return codes_coordinates


def getCodesCenters(img,codes_coordinates):
    h, w, c = img.shape
    centers = []
    for code in codes_coordinates:
        mask = np.zeros((h,w), np.uint8)
        pts = np.array(code,dtype=np.int32)
        pts = pts.reshape((-1, 1, 2))
        cv2.polylines(img, [pts], True, (0, 255, 0),thickness=2)  # draw bounds around QR codes
        cv2.fillConvexPoly(mask,pts,(255,255,255))      # draw binary mask of QR code
        # cv2.imshow('mask',mask)
        # cv2.waitKey(0)

        # compute center
        cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        M = cv2.moments(cnts[0])
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        centers.append((cX,cY))
        for center in centers:
            cv2.circle(img, center, 7, (255, 0, 0), -1)

    return centers, img



if __name__=="__main__":
    # img = cv2.imread('QRimage.jpg')

    # make scanner object
    scanner = zbar.ImageScanner()

    # read from webcam
    cap = cv2.VideoCapture(1)
    while (True):

        # Capture frame-by-frame
        ret, img = cap.read()
        if ret:
            # process frame for scanner
            h, w, c = img.shape
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            image = zbar.Image(w, h, 'Y800', img_gray.tobytes())
            scanner.scan(image)

            # find codes
            codes_coordinates = getCodes(image)
            centers, img_final = getCodesCenters(img, codes_coordinates)

            # Display the resulting frame
            cv2.imshow('frame', img_final)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()











