#!/usr/bin/env python
#coding=UTF-8

import rospy
import tf
from tf.transformations import *
from sensor_msgs.msg import Imu
from math import pi

def callback(data):
    #쿼터니언(Quaternion) 오일러 변환 (euler transform)
    (r,p,y) = tf.transformations.euler_from_quaternion((data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w))
    #라디안 방식으로 변환
    rospy.loginfo("Roll = %f, Pitch = %f, Yaw = %f",r*180/pi,p*180/pi,y*180/pi)

def get_imu():
    rospy.init_node('get_imu', anonymous=True)
    rospy.Subscriber("/handsfree/imu", Imu, callback)
    rospy.spin()

if __name__ == '__main__':
    get_imu()
