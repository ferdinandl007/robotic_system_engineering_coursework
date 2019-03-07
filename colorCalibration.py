#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from baxter_core_msgs.msg import DigitalIOState
import baxter_interface
import cv2
print(cv2.__version__)
import blobDetectionNEW as bd
#import Disk2

# color calibration
bgr_baxter = {}
hsv_baxter = {}
hanoi_colors = ['pink','red','orange','yellow','green','dark_green','blue','dark_blue']
#hanoi_colors = ['pink', 'red', 'orange', 'yellow', 'green', 'dark_green']
preprocess_img = True
hsv_offset = 3      # for hue ranges, default=3, range = [0,10] but upper limit can be extended
calib_button_1 = False
calib_button_2 = False
calibration_done = False
white_balance = False
gamma = 1           # for brightness correction, initially = 1 => no effect, limited to [-2,2] but can be changed
tile_size = 8       # for histogram equalization, max=40 in opencv, default=8, we set min=1
# NOTE1: histogram equalization does not work that well, might change it for the default method (equalization of the full image)

# publisher to which we post messages for Baxter's screen
colorPub = rospy.Publisher('/hanoi/colorCalibration', Bool)
ellipsePub = rospy.Publisher('/robot/xdisplay',Image)


def calibrationStartCallback(data):
    global calib_button_1
    if calibration_done:
        return
    if data.state == 1 and calib_button_1 == False:
        calib_button_1 = True
        rospy.loginfo("Color calibration started ...")


def calibrationStopCallback(data):
    global calib_button_2
    if calibration_done:
        return
    if data.state == 1 and calib_button_2 == False:
        calib_button_2 = True


def colorCalibrationCallbackKeyboard(key_pressed):
    global calib_button_1, calib_button_2
    if calibration_done:
        return
    if key_pressed == True and calib_button_1 == False:
        calib_button_1 = True
    elif key_pressed == True and calib_button_1 == True and calib_button_2==False:
        calib_button_2 = True


# changes gamma parameter (maps actual values from wheel)
def gammaCallbackProp(v):
    global gamma
    # middle 127
    if left_torso_nav.wheel == 127:
        gamma = 0
    else:
        gamma = bd.gammaConvert(left_torso_nav.wheel)
    print("Gamma value: %f" % gamma)


# changes gamma parameter (checks changes in wheel values)
def gammaCallbackStep(v):
    global gamma
    
    # middle 127
    if v > 0:
        gamma = gamma + 0.1
    elif v < 0:
        gamma = gamma - 0.1
    print("Gamma value: %f" % gamma)


# changes tile size parameter (maps actual values from wheel)
def tileSizeCallbackProp(v):
    global tile_size
    tile_size = bd.tileConvert(left_arm_nav.wheel)
    print("Tile size value: %f" % tile_size)


# changes tile size parameter (checks changes in wheel values)
def tileSizeCallbackStep(v):
    global tile_size
    
    # if there is an increase in wheel value
    if v > 0:
	# if increasing with 1 will make tile_size exceed upper bound, cap it at 40
        if (tile_size + 1) >= 40:
            tile_size = 40
	# if increasing with 1 will not make tile_size exceed bound, increase it
        else:
            tile_size = tile_size + 1
    # if there is a decrease in wheel value
    elif v < 0:
        # if decreasing with 1 will make tile_size exceed lower bound, cap it at 40
        if (tile_size - 1) <= 1:
            tile_size = 1
	# if decreasing with 1 will not make tile_size exceed lower bound, decrease it
        else:
            tile_size = tile_size -1
    print("Tile size value: %f" % tile_size)


# changes white_balance to ON/OFF
def wbONOFFCallback(v):
    global white_balance

    if v == True:
        if white_balance == False:
            white_balance = True
        else:
            white_balance = False
        print(white_balance)


# changes the hsv_offset parameter (maps actual values from wheel)
def hsvCallbackProp(v):
    global hsv_offset
    hsv_offset = bd.hsvConvert(right_torso_nav.wheel)
    print('Hue parameter: '+str(hsv_offset))


# changes the hsv_offset parameter (checks changes in wheel values)
def hsvCallbackStep(v):
    global hsv_offset
    # if there is an increase in the wheel value
    if v > 0:
        hsv_offset = hsv_offset + 1
    # if there is a decrease in the wheel value
    elif v < 0:
        if (hsv_offset - 1) <= 1:
            hsv_offset = 1
        else:
            hsv_offset = hsv_offset - 1
    print('Hue parameter: '+str(hsv_offset))


# callback function for camera subscriber, called by the camera subscriber for every frame.
def callback(data):
    global calib_button_1,calib_button_2,calibration_done,bgr_baxter,hsv_baxter,hanoi_colors,preproc_calib_img,hsv_offset,gamma, tile_size, white_balance
    bridge = CvBridge()

    # Convert incoming image from a ROS image message to a CV image that open CV can process.
    original_image = bridge.imgmsg_to_cv2(data, "bgr8")
    boxes_image = original_image.copy()

    # pre-process image:
    # brightness correction - if image too dark
    # white-balance correction - for more natural colors
    # histogram equalization - to improve contrast
    if preprocess_img:
        preprocessed_image = bd.preprocessImage(original_image, gamma, tile_size, white_balance)

    if not calibration_done:

        # start calibration
        if calib_button_1 and not calib_button_2:

            # compute, don't save masks
            bgr_baxter,out_image_ellipses = bd.getBGRavgColorsFromBaxter(preprocessed_image.copy(), hanoi_colors,False)
            hsv_baxter = bd.calibrateColors(bgr_baxter, hsv_offset)
            centers_x, centers_y, widths, heights, rotations, found_colors, contours = bd.getBoundingBoxes(preprocessed_image.copy(),
                                                                                                           hsv_baxter,
                                                                                                           hanoi_colors)
            stacked_masks,boxes_image = bd.drawBoxes(boxes_image, contours, found_colors)
            out_image_ellipses = bd.showParamOnImg(out_image_ellipses, hsv_offset, white_balance, gamma, tile_size)

            # show on screen
            cv2.imshow('Ellipses',out_image_ellipses)
            cv2.imshow('Masks',stacked_masks)
            cv2.imshow('Bounding boxes',boxes_image)

            # publish ellipses to Baxter's screen
            out_image_ellipses = bridge.cv2_to_imgmsg(out_image_ellipses, "bgr8")
            ellipsePub.publish(out_image_ellipses)

        # stop calibration, save info
        if calib_button_1 and calib_button_2:

            # compute and save masks
            bgr_baxter,out_image_ellipses = bd.getBGRavgColorsFromBaxter(preprocessed_image.copy(), hanoi_colors, True)
            hsv_baxter = bd.calibrateColors(bgr_baxter, hsv_offset)
            calibration_done = True
            bd.saveHSV(hsv_baxter)  # save HSV values in txt file
            bd.saveProcessParams(white_balance,gamma,tile_size)  # save image preprocess params

            cv2.destroyAllWindows()
            cv2.waitKey(100) # destroy all windows does not work really well
            colorPub.publish(True)
            rospy.loginfo("Color calibration done ...")




    # Display the converted cv image, this is the raw camera feed data.
    cv2.imshow("Raw Hand Camera Feed", original_image)


    # start and stop calibration from key press
    key = cv2.waitKey(1)
    if key == ord('c'):
        colorCalibrationCallbackKeyboard(True)








if __name__ == '__main__':
    rospy.init_node('calibration', anonymous=True)

    # create subscriber to the right hand camera, each frame recieved calls the callback function
    camera_sub = rospy.Subscriber("/cameras/right_hand_camera/image", Image, callback)

    # https://github.com/RethinkRobotics/baxter_examples/blob/master/scripts/navigator_io.py

    # Baxter buttons
    left_torso_nav = baxter_interface.Navigator('torso_left')       # for gamma parameter
    left_arm_nav = baxter_interface.Navigator('left')               # for tile_size
    right_torso_nav = baxter_interface.Navigator('torso_right')     # for Hue range

    # # to adjust camera settings
    # left_hand_camera_controller = baxter_interface.CameraController('left_hand_camera')
    # right_hand_camera_controller = baxter_interface.CameraController('right_hand_camera')
    # right_hand_camera_controller.exposure = 50 change it, maybe using a wheel as well, between 0 and 1
    # head_camera_controller = baxter_interface.CameraController('head_camera')

    # subscriber to right arm button0 press for color calibration start
    button_sub_start = rospy.Subscriber("/robot/digital_io/right_itb_button0/state", DigitalIOState,
                                        calibrationStartCallback)

    # subscriber to right arm button1 press for color calibration stop
    button_sub_stop = rospy.Subscriber("/robot/digital_io/right_itb_button1/state", DigitalIOState,
                                       calibrationStopCallback)

    # /robot/navigators/right_navigator/state
    # message type: NavigatorState from baxter_core_msgs.msg
    # check for left torso wheel changes - changes gamma parameter
    left_torso_nav.wheel_changed.connect(gammaCallbackStep)

    # check for left arm button 1 press for white balance on-off
    left_arm_nav.button1_changed.connect(wbONOFFCallback)

    # check for left arm wheel changes - changes tile size parameter
    left_arm_nav.wheel_changed.connect(tileSizeCallbackStep)

    # check for right torso wheel changes - changes the hsv_offset
    right_torso_nav.wheel_changed.connect(hsvCallbackStep)


    # prevents program from exiting, allowing subscribers and publishers to keep operating
    # in our case that is the camera subscriber and the image processing callback function
    rospy.spin()
