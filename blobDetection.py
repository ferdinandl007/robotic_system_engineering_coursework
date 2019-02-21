# https://www.pyimagesearch.com/2016/02/01/opencv-center-of-contour/

# Standard imports
import cv2
print(cv2.__version__)
import numpy as np
import  os
opencv_2 = cv2.__version__.startswith('2')

# for testing purposes only
def testShow(to_show):
    cv2.imshow('',to_show)
    cv2.waitKey(0)

# source: image from my webcam
# H, S, V found manually
# PINK_MIN = np.array([143,22,211], np.uint8)
# PINK_MAX = np.array([178,244,255], np.uint8)
# RED_MIN = np.array([0,5,0], np.uint8)
# RED_MAX = np.array([2,255,255], np.uint8)
# ORANGE_MIN = np.array([4, 126, 143], np.uint8)
# ORANGE_MAX = np.array([17, 255, 255], np.uint8)
# YELLOW_MIN = np.array([25,88,0], np.uint8)
# YELLOW_MAX = np.array([39,255,255], np.uint8)
# GREEN_MIN = np.array([37,33,0], np.uint8)
# GREEN_MAX = np.array([74,255,255], np.uint8)
# BLUE_MIN = np.array([87,71,0], np.uint8)
# BLUE_MAX = np.array([106,255,255], np.uint8)
# DARKBLUE_MIN = np.array([110,167,0], np.uint8)
# DARKBLUE_MAX = np.array([135,255,255], np.uint8)
# WOOD_MIN =  np.array([14,19,0], np.uint8)
# WOOD_MAX =  np.array([31,110,255], np.uint8)

# source: image from Baxter's camera
# H, S, V found manually
PINK_MIN = np.array([143,22,211], np.uint8)
PINK_MAX = np.array([178,244,255], np.uint8)
RED_MIN = np.array([0,145,0], np.uint8)
RED_MAX = np.array([4,255,255], np.uint8)
ORANGE_MIN = np.array([4, 137, 0], np.uint8)
ORANGE_MAX = np.array([14, 255, 255], np.uint8)
YELLOW_MIN = np.array([19,115,0], np.uint8)
YELLOW_MAX = np.array([35,255,255], np.uint8)
GREEN_MIN = np.array([43,47,0], np.uint8)
GREEN_MAX = np.array([77,255,255], np.uint8)
BLUE_MIN = np.array([89,71,0], np.uint8)
BLUE_MAX = np.array([106,255,255], np.uint8)
DARKBLUE_MIN = np.array([106,71,0], np.uint8)
DARKBLUE_MAX = np.array([149,255,255], np.uint8)
WOOD_MIN =  np.array([0,0,143], np.uint8)
WOOD_MAX =  np.array([35,107,255], np.uint8)


hsv_manual = {
    'pink': [PINK_MIN,PINK_MAX],
    'red': [RED_MIN,RED_MAX],
    'orange': [ORANGE_MIN,ORANGE_MAX],
    'yellow': [YELLOW_MIN,YELLOW_MAX],
    'green': [GREEN_MIN,GREEN_MAX],
    'blue': [BLUE_MIN,BLUE_MAX],
    'dark blue': [DARKBLUE_MIN,DARKBLUE_MAX]
}

def getBGRaverage(path):
    bgr_average = {}
    disks = os.listdir(path)

    for d in disks:
        if d.endswith('.png') or d.endswith('.jpg'):
            color_name = d.split('.')[0]
            img = cv2.imread(os.path.join(path,d))
            b, g, r, _ = cv2.mean(img)
            bgr_average[color_name] = np.uint8([[[int(b),int(g),int(r)]]])
    return bgr_average

# source: average pixel value of disks
bgr_average = getBGRaverage('video_disks')

# source: using color picker in GIMP on image of disks
bgr_picker = {
    'pink': np.uint8([[[116, 53, 246]]]),
    'red': np.uint8([[[3, 12, 216]]]),
    'orange': np.uint8([[[17, 64, 222]]]),
    'yellow': np.uint8([[[1, 209, 251]]]),
    'green': np.uint8([[[84, 199, 155]]]),
    'dark green': np.uint8([[[60, 175, 1]]]),
    'blue': np.uint8([[[219, 194, 98]]]),
    'dark blue': np.uint8([[[166, 67, 5]]])
}

# source: http://www.tayloredmktg.com/rgb/
# theoretical colors
bgr_default = {
    'pink': np.uint8([[[180, 105, 255]]]),  # hot pink
    'red': np.uint8([[[0, 0, 255]]]),
    'orange': np.uint8([[[0, 69, 255]]]),  # orange red
    'yellow': np.uint8([[[0, 255, 255]]]),
    'green': np.uint8([[[152, 251, 152]]]),  # pale green
    'dark green': np.uint8([[[50, 215, 50]]]),  # lime green
    'blue': np.uint8([[[250, 206, 135]]]),  # light sky blue
    'dark blue': np.uint8([[[255, 0, 0]]])
}


# Hue: the type of chroma
# Saturation: the amount of white content in the chroma
# Value: the amount of blank content in the chroma
def getBoundsHSV(color):
    # 2) transform to hsv colospace
    hsv = cv2.cvtColor(color,cv2.COLOR_BGR2HSV)
    #hsv = colors_dictionary.get(color_name)

    # 3) get color bounds
    # Note: hue has to be between [0,179]
    offset = 5
    hue = int (hsv[0][0][0])

    # wrap around
    low_overflow = False
    up_overflow = False
    if(hue - offset) < 0:
        hue_low = (hue - offset) % 180
        low_overflow = True
    if (hue + offset) > 179:
        hue_up = (hue + offset) % 180
        up_overflow = True
    if low_overflow and not up_overflow:
        upper = np.uint8([hsv[0][0][0] + offset, 255, 255])
        lower = np.uint8([hue_low, 100, 100])
    if up_overflow and not low_overflow:
        upper = np.uint8([hue_up, 255, 255])
        lower = np.uint8([hsv[0][0][0] - offset, 100, 100])
    if not (low_overflow or up_overflow):
        lower = np.uint8([hsv[0][0][0] - offset, 100, 100])
        upper = np.uint8([hsv[0][0][0] + offset, 255, 255])

    return lower,upper


def getColorMask(colorLower, colorUpper, hsv):
    # if overflow occurs
    if (colorLower[0] > colorUpper[0]):
        part1_start = np.uint8([colorLower[0], 100, 100])
        part1_stop = np.uint8([179, 255, 255])
        part2_start = np.uint8([0, 100, 100])
        part2_stop = np.uint8([colorUpper[0], 255, 255])
        color_mask1 = cv2.inRange(hsv, part1_start, part1_stop)
        color_mask2 = cv2.inRange(hsv, part2_start, part2_stop)
        color_mask = cv2.bitwise_or(color_mask1, color_mask2)

    # if overflow does not occur
    else:
        color_mask = cv2.inRange(hsv, colorLower, colorUpper)

    # Smooth result (better actually with iterations=2)
    color_mask = cv2.erode(color_mask, None, iterations=2)
    color_mask = cv2.dilate(color_mask, None, iterations=2)
    return color_mask


def getAllMasks(img,colors_dictionary,color_space):
    # Hue range:[0,179], Saturation range:[0,255], Value range:[0,255]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    color_masks = []
    colors = list(colors_dictionary.keys())

    # choose method depending on the colorspace of the dictionary
    if color_space == 'bgr':
        for col in colors:
            low,up = getBoundsHSV(colors_dictionary.get(col))
            mask = getColorMask(low, up, hsv)
            color_masks.append(mask)

    elif color_space == 'hsv':
        for col in colors:
            mask = getColorMask(hsv_manual.get(col)[0], hsv_manual.get(col)[1], hsv)
            color_masks.append(mask)
    else:
        print('Colorspace not valid!')
    return color_masks


def getContours(color_masks,colors):
    contours = []
    found_colors = []

    for m in range(len(color_masks)):

        # version opencv 2
        if opencv_2:
            cntsR, hierarchy = cv2.findContours(color_masks[m].copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # version opencv 3
        else:
            img, cntsR, hierarchy = cv2.findContours(color_masks[m].copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # if contours have been found, select the one with largest area
        # ASSUMES disk area is the largest
        if len(cntsR) != 0:
            # get the index of the largest contour
            areas = [cv2.contourArea(c) for c in cntsR]
            max_index = np.argmax(areas)
            c = cntsR[max_index]
            contours.append(c)
            found_colors.append(colors[m])
            # cX,cY = getCenterContour(c)   # if we need it?
        else:
            print(colors[m]+' disk was not found!')

    return contours,found_colors


def getWoodContour(wood_mask):
    # version opencv 2
    if opencv_2:
        cntsR, hierarchy = cv2.findContours(wood_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # version opencv 3
    else:
        img, cntsR, hierarchy = cv2.findContours(wood_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    areas = []
    for cnt in cntsR:
        areas = [cv2.contourArea(c) for c in cntsR]
        # check saturation as well?
    max_index = np.argmax(areas)
    c = cntsR[max_index]
    return c


def getCenterContour(contour):
    # contour expressed as (number of points, 1, 2) where 2 represents the x and y coordinates
    # compute the center of the contour
    M = cv2.moments(contour)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return cX,cY


# source: https://stackoverflow.com/a/46391574
def whiteBalance(img):
    result = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    avg_a = np.average(result[:, :, 1])
    avg_b = np.average(result[:, :, 2])
    result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result = cv2.cvtColor(result, cv2.COLOR_LAB2BGR)
    return result


# source: https://www.pyimagesearch.com/2015/10/05/opencv-gamma-correction/
def gammaCorrection(img,gamma):
    inv_gamma = 1.0/gamma
    table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
    gamma_corrected_img = cv2.LUT(img, table)
    return gamma_corrected_img


# Contrast Limited Adaptive Histogram Equalization
# source: https://stackoverflow.com/a/47370615
def clahe(img, grid_size):
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    lab_planes = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(grid_size, grid_size))
    lab_planes[0] = clahe.apply(lab_planes[0])
    lab = cv2.merge(lab_planes)
    img = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    return img


def preprocessImage(img):

    # Brightness correction
    img = gammaCorrection(img,1.5) # gamma=1.5
    # cv2.imshow('After gamma correction',img)

    # white-balance correction
    img = whiteBalance(img)
    # cv2.imshow('After white balance', img)

    # # Histogram Equalization // not very good results
    # img = clahe(img,5) # tile size = 5x5
    # # cv2.imshow('After CLAHE', img)

    return img


def getBoundingBoxes (img,colors_dictionary,colorspace):

    # brightness correction, histogram equalization
    # img = preprocessImage(img)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Get all masks based on color ranges
    colors = list(colors_dictionary.keys()) # compatible with both Python 2 and 3
    color_masks = getAllMasks(img,colors_dictionary,colorspace)

    # Find wood blob
    global WOOD_MIN, WOOD_MAX
    wood_mask = getColorMask(WOOD_MIN, WOOD_MAX, hsv)
    wood_contour = getWoodContour(wood_mask)
    cv2.drawContours(img, [wood_contour], 0, (0, 255, 0), 2)
    rect_wood = cv2.minAreaRect(wood_contour)
    print('Wood center on x axis:',rect_wood[0][0])
    print('Wood center on y axis:', rect_wood[0][1])
    print('Wood width:', rect_wood[1][0])
    # height: rect_wood[1][1] rotation: rect_wood[2]


    # Detect contours around possible blobs of disks
    contours,found_colors = getContours(color_masks,colors)
    centers_x,centers_y,widths,heights,rotations = [],[],[],[],[]

    # Display each rectangle and save its details
    for i in range(len(contours)):
        contour = contours[i]

        # save info of bounding box
        rect = cv2.minAreaRect(contour)
        centers_x.append(rect[0][0])
        centers_y.append(rect[0][1])
        widths.append(rect[1][0])
        heights.append(rect[1][1])
        rotations.append(rect[2])

        # draw bounding box
        disk_box = cv2.boxPoints(rect)
        disk_box = np.int0(disk_box)
        cv2.drawContours(img, [disk_box], 0, (255, 191, 0), 2)

        # draw bounding box center
        # dest image, center of circle, radius, color, thickness = -1 (filled circle)
        cv2.circle(img, (int(rect[0][0]), int(rect[0][1])), 7, (255, 255, 255), -1)

        # # diplay color
        # # image, text, bottom-left corner of the text, font, font scale, color, thickness
        cv2.putText(img, colors[i], (int(rect[0][0]) + 20, int(rect[0][1]) + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)


    # Display original image and finals masks
    stacked_masks = np.zeros(img.shape, np.uint8)
    cv2.drawContours(stacked_masks, contours, -1, (255, 255, 255), -1)
    cv2.imshow('Original',img)
    cv2.imshow('Masks', stacked_masks)
    cv2.waitKey(1)

    # return information for bounding box (as arrays):
    # center on X-axis, center on Y-axis, width, height, angle rotation of rectangle, found colors
    # the list of found colors might be smaller than initial list of colors
    return centers_x,centers_y,widths,heights,rotations,found_colors

