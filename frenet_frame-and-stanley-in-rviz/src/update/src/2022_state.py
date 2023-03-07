#! /usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import tf
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion
from object_msgs.msg import Object
from nav_msgs.msg import Odometry
import numpy as np

class ImuPub:
	def __init__(self):
		#self.object_gps_pub=rospy.Publisher("/objects/car_1/gps", Object,queue_size=1)
		self.object_pub=rospy.Publisher("/objects/car_1", Object,queue_size=1)
		self.marker_pub=rospy.Publisher("/objects/marker/car_1", Marker,queue_size=1)
		self.odom_sub=rospy.Subscriber("/odom", Odometry,  self.odometry_callback)
		self.tf_broadcaster = tf.TransformBroadcaster()

		#self.x_gps = 0
		#self.y_gps = 0
		#self.v_gps = 0
		#self.yaw_gps = 0

		self.x = 0
		self.y = 0
		self.v = 0
		self.yaw = 0

	
	def msg_pub(self):
		self.yaw+=2.3
		quat = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
		self.tf_broadcaster.sendTransform((self.x, self.y, 1.5), quat,rospy.Time.now(),"/car_1", "/map")

		o = Object()
		o.header.frame_id = "/map"
		o.header.stamp = rospy.Time.now()
		o.id = 1
		o.classification = o.CLASSIFICATION_CAR
		o.x = self.x
		o.y = self.y
		o.yaw = self.yaw
		#o.yaw=1.28713
		########
		o.v = self.v
		#######
		o.L = 1.600
		o.W = 1.160
		#o.WB = 1.06
		self.object_pub.publish(o)


		m = Marker()
		m.header.frame_id = "/map"
		m.header.stamp = rospy.Time.now()
		m.id = 1
		m.type = m.CUBE

		#m.pose.position.x = x + 1.3 * math.cos(yaw)
		#m.pose.position.y = y + 1.3 * math.sin(yaw)

		m.pose.position.x = self.x
		m.pose.position.y = self.y
		m.pose.position.z = 0.3
		m.pose.orientation = Quaternion(*quat)

		m.scale.x = 1.600
		m.scale.y = 1.160
		m.scale.z = 1.645

		m.color.r = 93 / 255.0
		m.color.g = 122 / 255.0
		m.color.b = 177 / 255.0
		m.color.a = 0.97

		self.marker_pub.publish(m)


	def odometry_callback(self, data):
		# sensor_msgs/Imu.msg 
		self.x = data.pose.pose.position.x 
		self.y = data.pose.pose.position.y 
		vx = data.twist.twist.linear.x
		vy = data.twist.twist.linear.y
		# vz = data.twist.twist.linear.z
		self.v = np.sqrt(vx**2+vy**2)

		orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w] 
		roll, pitch, self.yaw = euler_from_quaternion(orientation_list) 
		print(self.yaw)
		#self.yaw+=0.27#0.18#35047823265787037


if __name__ == "__main__":
    
	rospy.init_node("state")
	node = ImuPub()
	r = rospy.Rate(10)

	while not rospy.is_shutdown():
		node.msg_pub()
		r.sleep()