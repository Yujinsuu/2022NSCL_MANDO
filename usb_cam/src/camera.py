#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import glob
import rospy
import message_filters

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def callback_usb_cam(img):
    global cv_img, roi_img, count
    count +=1
    bridge = CvBridge()
    cv_img = bridge.imgmsg_to_cv2(img, desired_encoding='rgb8')
    roi_img = cv_img[0:320,200:520]
    #roi_img = cv_img[80:320,240:480]

    image_message = bridge.cv2_to_imgmsg(roi_img, encoding='rgb8')
    image_message.header.stamp = rospy.Time.now()

    camera_pub.publish(image_message)

if __name__ == '__main__':
    count=0
    rospy.init_node('Pretreat')
    camera_pub = rospy.Publisher("/pretreat_image", Image, queue_size = 1)
    
    rospy.Subscriber("/usb_cam/image_raw", Image, callback_usb_cam)  
    rospy.spin()