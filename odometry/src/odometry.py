#!/usr/bin/env python

import rospy
import serial
import time
import math
import pickle

import tf
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Header, Float64, Int8
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
from pyproj import Proj, transform
from nav_msgs.msg import Odometry

import numpy as np

vo_Pub=rospy.Publisher('/odom',Odometry,queue_size=1)

def gps_callback(msg):

	global cov1,cov2

	mando=Proj(init='epsg:5179')
	wgs84=Proj(init='epsg:4326')
	a,b=transform(wgs84,mando,msg.longitude,msg.latitude)

	rpose.header.stamp=rospy.Time.now()
	rpose.pose.pose.position.x=a
	rpose.pose.pose.position.y=b
	rpose.pose.covariance[0]=msg.position_covariance[0]
	rpose.pose.covariance[7]=msg.position_covariance[4]
	rpose.pose.covariance[14]=msg.position_covariance[8]
		

def gps_vel_callback(msg):
	
	rpose.twist.twist.linear.x = msg.twist.twist.linear.x
	rpose.twist.twist.linear.y = msg.twist.twist.linear.y
	rpose.twist.twist.linear.z = msg.twist.twist.linear.z
	rpose.pose.covariance[21]=99999
	rpose.pose.covariance[28]=99999
	rpose.pose.covariance[35]=msg.twist.covariance[0]
	#rate=rospy.Rate(1)
	#rate.sleep()


def imu_callback(msg):
	rpose.pose.pose.orientation.x = msg.orientation.x
	rpose.pose.pose.orientation.y = msg.orientation.y
	rpose.pose.pose.orientation.z = msg.orientation.z
	rpose.pose.pose.orientation.w = msg.orientation.w

	vo_Pub.publish(rpose)
	
	orientation_list = [rpose.pose.pose.orientation.x, rpose.pose.pose.orientation.y, rpose.pose.pose.orientation.z, rpose.pose.pose.orientation.w] 
	roll, pitch, yaw = euler_from_quaternion (orientation_list) 
	#print(roll,pitch,yaw)

if __name__=='__main__':
	
	rospy.init_node('gps_to_vo')

	global rpose
	rpose=Odometry()
	rpose.header.frame_id="odom"

	rospy.Subscriber("/gps/fix",NavSatFix,gps_callback)
	rospy.Subscriber("/ublox_gps/fix_velocity",TwistWithCovarianceStamped,gps_vel_callback)
	rospy.Subscriber("/handsfree/imu",Imu, imu_callback)
	
	rospy.spin()
