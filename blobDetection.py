# https://www.pyimagesearch.com/2016/02/01/opencv-center-of-contour/

# Standard imports
import cv2
print(cv2.__version__)
import numpy as np
import  os
opencv_2 = cv2.__version__.startswith('2')


########################################################################################################################

# source: image from Baxter's camera
# H, S, V found manually (no image preprocessing)
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

col = ['pink','red','orange','yellow','green','blue','dark blue']

# source: http://www.tayloredmktg.com/rgb/
# theoretical colors
bgr_default = {
    col[0]: np.uint8([[[180, 105, 255]]]),     # hot pink
    col[1]: np.uint8([[[0, 0, 255]]]),         # red
    col[2]: np.uint8([[[0, 69, 255]]]),        # orange red
    col[3]: np.uint8([[[0, 255, 255]]]),       # yellow
    col[4]: np.uint8([[[152, 251, 152]]]),     # pale green
    col[5]: np.uint8([[[250, 206, 135]]]),     # light sky blue
    col[6]: np.uint8([[[255, 0, 0]]])          # dark blue
}

# source: found using colorspaceAnalysis.py
hsv_manual = {
    col[0]: [PINK_MIN,PINK_MAX],           # pink
    col[1]: [RED_MIN,RED_MAX],             # red
    col[2]: [ORANGE_MIN,ORANGE_MAX],       # orange
    col[3]: [YELLOW_MIN,YELLOW_MAX],       # yellow
    col[4]: [GREEN_MIN,GREEN_MAX],         # green
    col[5]: [BLUE_MIN,BLUE_MAX],           # blue
    col[6]: [DARKBLUE_MIN,DARKBLUE_MAX]    # dark blue
}

########################################################################################################################

# for testing purposes only
def testShow(to_show):
    cv2.imshow('',to_show)
    cv2.waitKey(0)


def getBGRavg(img):
    """Computes the average pixel value given an image of a blob.
    Args:
        img (ndarray): the image of the colored blob
    Returns:
        int,int,int: average of the B, G an R channels of the image
    """

    # BGR image => count is applied on each channel => divide by 3
    num_pixels = int(np.count_nonzero(img)/3)
    b, g, r = cv2.split(img)
    b_avg = int(np.sum(b) / num_pixels)
    g_avg = int(np.sum(g) / num_pixels)
    r_avg = int(np.sum(r) / num_pixels)
    return b_avg,g_avg,r_avg


def getBGRavgColorsFromFile(path):
    """Computes the average pixel values of all images in a directory.
        Args:
            path (string): path to directory
        Returns:
            bgr_average (dict): each key is a color (string), corresponding value is an np.uint8[b,g,r] (0-255) with 3
            values: average on B, average on G and average on R channel
    """
    bgr_average = {}
    disks = os.listdir(path)
    for d in disks:
        if d.endswith('.png') or d.endswith('.jpg'):
            color_name = d.split('.')[0]
            img = cv2.imread(os.path.join(path,d))
            b_avg, g_avg, r_avg = getBGRavg(img)
            bgr_average[color_name] = np.uint8([[[b_avg, g_avg, r_avg]]])
    return bgr_average

# # source: average pixel value of disks from baxter's camera
# # usage, if colors have already been calibrated, read directly from file
# bgr_average = getBGRavgColorsFromFile('video_disks')


# Hue: the type of chroma
# Saturation: the amount of white content in the chroma
# Value: the amount of blank content in the chroma
def getBoundsHSV(color,hsv_offset):
    """Computes the bounds in the HSV colospace of a color in the RGB colospace, based on an offset (internal).
    Args:
        color (np.uint8[r,g,b]): RGB values
        hsv_offset (int): above and below limit on the Hue channel
    Returns:
        lower,upper (np.uint8[h,s,v],np.uint8[h,s,v]): lower and upper bounds in HSV
    Note: checks for wraparound (red) in the Hue channel
    """

    # 1) transform to hsv colospace
    hsv = cv2.cvtColor(color,cv2.COLOR_BGR2HSV)
    #hsv = colors_dictionary.get(color_name)

    # 2) get color bounds
    # Note: hue has to be between [0,179]
    hue = int (hsv[0][0][0])

    # wrap around
    low_overflow = False
    up_overflow = False
    if(hue - hsv_offset) < 0:
        hue_low = (hue - hsv_offset) % 180
        low_overflow = True
    if (hue + hsv_offset) > 179:
        hue_up = (hue + hsv_offset) % 180
        up_overflow = True
    if low_overflow and not up_overflow:
        upper = np.uint8([hsv[0][0][0] + hsv_offset, 255, 255])
        lower = np.uint8([hue_low, 100, 100])
    if up_overflow and not low_overflow:
        upper = np.uint8([hue_up, 255, 255])
        lower = np.uint8([hsv[0][0][0] - hsv_offset, 100, 100])
    if not (low_overflow or up_overflow):
        lower = np.uint8([hsv[0][0][0] - hsv_offset, 100, 100])
        upper = np.uint8([hsv[0][0][0] + hsv_offset, 255, 255])

    return lower,upper


def getColorMask(colorLower, colorUpper, hsv):
    """Computes a mask for input image based on lower and upper bounds.
        Args:
            colorLower (np.uint8[h,s,v]): lower HSV bound
            colorUpper (np.uint8[h,s,v]): upper HSV bound
            hsv (ndarray): image in the HSV colorspace
        Returns:
            color_mask (ndarray): color mask of the HSV image
    """

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


def getAllMasks(img, hanoi_colors, colors_dictionary_hsv):
    """Computes masks for all colors in the colors dictionary.
        Args:
            img (ndarray): image with colored blobs in the RGB colorspace
            hanoi_colors ([]): list of colors to be searched for
            colors_dictionary_hsv (dict): colors of interest (keys as string, values as [np.uint8[h,s,v],np.uint8[h,s,v]])
        Returns:
            color_masks (list of ndarrays): masks belonging to each color blob found
    """

    # Hue range:[0,179]
    # Saturation range:[0,255]
    # Value range:[0,255]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    color_masks = []
    for color in hanoi_colors:
        hsv_minmax = colors_dictionary_hsv.get(color)
        low = hsv_minmax[0]
        up = hsv_minmax[1]
        mask = getColorMask(low, up, hsv)
        color_masks.append(mask)
    return color_masks


def getContours(color_masks,colors):
    """Finds contours based on input masks.
        Args:
            color_masks (list of ndarrays): masks of each colored blob
            colors  ([]): string array of colors of interest
        Returns:
            contours (list of ndarrays): largest contour found in each color mask
            found_colors (string array): list of actually found colors in masks
        Note1: ASSUMES disk contour is the largest.
        Note2: A color (corresponding contour) might not be found if mask is empty.
    """

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

# unused ATM
def getCenterContour(contour):
    """Finds the center of a blob."""

    # contour expressed as (number of points, 1, 2) where 2 represents the x and y coordinates
    # compute the center of the contour
    M = cv2.moments(contour)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return cX,cY


# White balance correction
# source: https://stackoverflow.com/a/46391574
def whiteBalance(img):
    result = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    avg_a = np.average(result[:, :, 1])
    avg_b = np.average(result[:, :, 2])
    result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result = cv2.cvtColor(result, cv2.COLOR_LAB2BGR)
    return result


# Brightness correction
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
    """Image preprocessing. Steps: Brightness correction, White-balance correction, Histogram Equalization."""

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



def getDiskY(disk_ind, max_y, disk_height, inter_space):
    """Recursively computes the coordinate on the y axis of disk's template.
        Args:
            disk_ind (int): the index of the disk in the colors array (0 for 'pink', 6 for 'dark blue')
        Returns:
            y (int): coordinate on y-axis
    """

    if disk_ind == 0:
        y = max_y
        return y
    else:
        y = getDiskY(disk_ind - 1, max_y, disk_height, inter_space) - disk_height - inter_space
        return y


def getBGRavgColorsFromBaxter(img,color_names,preprocess,save):
    if preprocess:
        img = preprocessImage(img)

    img_h = img.shape[0]                        # image height
    img_w = img.shape[1]                        # image width
    center_x = int(img_w/2)                     # image center on x axis
    center_y = int(img_h/2)                     # image center on y axis
    img_margin = int(img_h / 20)                # up and bottom unused space

    nr_disks = len(color_names)                 # number of disks
    disk_height = int(img_h/(nr_disks+15))      # disk height (trial and error)
    disk_width = int(img_w/15)                   # disk width
    min_y = img_margin + int(disk_height/2)     # minimum y coord of a disk
    max_y = img_h - min_y                       # maximum y coord of a disk
    inter_space = 25                            # distance between ellipses (trial and error)
    disks_y_coord = []                          # from top disk to bottom disk
    color = (255,255,255)                       # ellipse initial color
    bgr_baxter = {}                             # found BGR values


    # compute y coordinates of disk's template
    for d in range(nr_disks,0,-1):
        y_disk = getDiskY(d,max_y, disk_height, inter_space)
        disks_y_coord.append(y_disk)



    for d in range(nr_disks):
        # get masks
        color_mask = np.zeros((img_h, img_w, 1), np.uint8)
        cv2.ellipse(color_mask, (center_x, disks_y_coord[d]), (disk_width, disk_height), 0, 0, 360, color=color, thickness=-1)
        img_masked = cv2.bitwise_and(img, img, mask=color_mask)

        # display name of corresponding color
        # # diplay color
        # # image, text, bottom-left corner of the text, font, font scale, color, thickness
        cv2.putText(img, color_names[d], (center_x+int(0.3*img_w), disks_y_coord[d]), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 2)

        # compute average color under mask
        num_pixels = np.count_nonzero(color_mask)
        b,g,r = cv2.split(img_masked)


        b_avg = int(np.sum(b)/num_pixels)
        g_avg = int(np.sum(g)/num_pixels)
        r_avg = int(np.sum(r)/num_pixels)
        color = (b_avg,g_avg,r_avg)

        # save values and crops
        bgr_baxter[color_names[d]] = np.uint8([[[b_avg,g_avg,r_avg]]])

        if save:
            cv2.imwrite(os.path.join('video_disks',color_names[d]+'.png'),img_masked)

        # draw filled ellipse with average color under color mask
        cv2.ellipse(img, (center_x, disks_y_coord[d]), (disk_width, disk_height), 0, 0, 360, color=color, thickness=-1)
    cv2.imshow('Ellipses',img)
    # cv2.waitKey(0)

    return bgr_baxter


def calibrateColors(colors_dictionary_bgr,hsv_offset):
    colors_dictionary_hsv = {}
    for color_name, bgr_avg in colors_dictionary_bgr.items():
        lower, upper = getBoundsHSV(bgr_avg,hsv_offset)
        colors_dictionary_hsv[color_name] = [lower,upper]
    return colors_dictionary_hsv


def getBoundingBoxes (img, colors_dictionary_hsv,hanoi_colors):
    """Returns information about the bounding boxes surrounding detected colored blobs and draws boxes.

    Args:
        img (ndarray): image with colored blobs in the RGB colorspace
        colors_dictionary_hsv (dict): colors of interest (keys as color name, values as [np.uint8[h,s,v],np.uint8[h,s,v]])
    Returns:
        centers_x ([]): coordinates of bounding boxes on x axis
        centers_y ([]): coordinates of bounding boxes on y axis
        widths ([]): width of bounding boxes
        heights ([]): height of bounding boxes
        rotations ([]): angle rotations of bounding boxes
        found_colors ([]): name of colors detected
    Note1: Number of found colors can be smaller than the number of colors in the dictionary
    """

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Get all masks based on color ranges
    color_masks = getAllMasks(img, hanoi_colors, colors_dictionary_hsv)

    # Find wood blob
    # add here return from qr code code


    # Detect contours around possible blobs of disks
    contours,found_colors = getContours(color_masks,hanoi_colors)
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

        # # draw bounding box
        # disk_box = cv2.cv.BoxPoints(rect)
        # disk_box = np.int0(disk_box)
        # cv2.drawContours(img, [disk_box], 0, (255, 191, 0), 2)
        #
        # # draw bounding box center
        # # dest image, center of circle, radius, color, thickness = -1 (filled circle)
        # cv2.circle(img, (int(rect[0][0]), int(rect[0][1])), 7, (255, 255, 255), -1)
        #
        # # # diplay color
        # # # image, text, bottom-left corner of the text, font, font scale, color, thickness
        # cv2.putText(img, found_colors[i], (int(rect[0][0]) + 10, int(rect[0][1]) + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        #

    # # Display original image and finals masks
    # stacked_masks = np.zeros(img.shape, np.uint8)
    # cv2.drawContours(stacked_masks, contours, -1, (255, 255, 255), -1)
    # cv2.imshow('Original',img)
    # cv2.imshow('Masks', stacked_masks)
    # cv2.waitKey(0)

    # return information for bounding box (as arrays):
    # center on X-axis, center on Y-axis, width, height, angle rotation of rectangle, found colors
    # the list of found colors might be smaller than initial list of colors
    return centers_x,centers_y,widths,heights,rotations,found_colors,contours


def drawBoxes(img,contours,found_colors):
    # Display each rectangle and save its details
    for i in range(len(contours)):
        contour = contours[i]

        # save info of bounding box
        rect = cv2.minAreaRect(contour)

        # draw bounding box
        disk_box = cv2.cv.BoxPoints(rect)
        disk_box = np.int0(disk_box)
        cv2.drawContours(img, [disk_box], 0, (255, 191, 0), 2)

        # draw bounding box center
        # dest image, center of circle, radius, color, thickness = -1 (filled circle)
        cv2.circle(img, (int(rect[0][0]), int(rect[0][1])), 7, (255, 255, 255), -1)

        # # diplay color
        # # image, text, bottom-left corner of the text, font, font scale, color, thickness
        cv2.putText(img, found_colors[i], (int(rect[0][0]) + 10, int(rect[0][1]) + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Display original image and finals masks
    stacked_masks = np.zeros(img.shape, np.uint8)
    cv2.drawContours(stacked_masks, contours, -1, (255, 255, 255), -1)
    cv2.imshow('Bounding boxes',img)
    cv2.imshow('Masks', stacked_masks)
    # cv2.waitKey(0)