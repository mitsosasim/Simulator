#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_cb(msg):
    # Convert to OpenCV image
    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    # Display or save
    cv2.imshow('front_camera', cv_img)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('camera_viewer')
    bridge = CvBridge()
    rospy.Subscriber('/automobile/image_raw', Image, image_cb, queue_size=1)
    rospy.spin()
