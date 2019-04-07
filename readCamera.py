#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def sensorInfoPublisher():
    bridge = CvBridge()
    # 'camera_head'
    si_publisher = rospy.Publisher('/cameras2/right_hand_camera/image', Image, queue_size=10)
    rospy.init_node('camera_cv2_baxter_head', anonymous=False)
    rate = rospy.Rate(30)
    cap = cv2.VideoCapture(1)  # might change: check with ls /dev/video tab tab

    while not rospy.is_shutdown():

        ret, frame = cap.read()
        if not ret:
            continue
        ros_img = bridge.cv2_to_imgmsg(frame, "bgr8")

        # publish the sensor information on the /sensor_info topic
        si_publisher.publish(ros_img)

        # print a lof message if all went well
        rospy.loginfo('Publishing image from head')

        rate.sleep()


if __name__ == '__main__':
    try:
        sensorInfoPublisher()
    except rospy.ROSInterruptException:
        pass
