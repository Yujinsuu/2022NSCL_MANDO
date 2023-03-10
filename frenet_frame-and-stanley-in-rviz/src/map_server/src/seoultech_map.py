#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import rospkg
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import pickle


class Converter(object):
	def __init__(self, file_=None, waypoints=None, start_id=0, r=255/255.0, g=255/255.0, b=255/255.0, a= 0.5, scale=0.1):
		self.r = r
		self.g = g
		self.b = b
		self.a = a
		self.scale = scale
		self.start_id = start_id
		self.ma = None
		if file_ is not None:
			self.file = file_
			with open(file_, "rb") as f:
				self.waypoints = pickle.load(f)
			self.make_marker_array(self.waypoints)
		elif waypoints is not None:
			self.waypoints = waypoints
			self.make_marker_array(self.waypoints)
		else:
			self.waypoints = None

	def make_marker_array(self, waypoints):
		ma = MarkerArray()
		draw_step = 1
		if isinstance(self.waypoints, dict):
			# for A2_LINK_smoothed_near0.pkl
			waypoints = self.waypoints.values()
			draw_step = 3
		
		for i, waypoint in enumerate(waypoints):
			m = Marker()
			m.id = self.start_id + i
			m.header.frame_id = "/map"
			m.type = m.LINE_STRIP
			m.action = m.ADD

			m.scale.x = self.scale

			m.color.r = self.r
			m.color.g = self.g
			m.color.b = self.b
			m.color.a = self.a

			m.pose.orientation.x = 0
			m.pose.orientation.y = 0
			m.pose.orientation.z = 0
			m.pose.orientation.w = 1

			m.points = []
			for j in range(len(waypoint["x"])):
				if j % draw_step == 0 or j == len(waypoint["x"])-1:
					p = Point()
					p.x = waypoint["x"][j]
					p.y = waypoint["y"][j]
					m.points.append(p)

			ma.markers.append(m)
		self.ma = ma
		


if __name__ == "__main__":
	rospy.init_node("map_rviz_visualizer")
	rospack = rospkg.RosPack()
	path = rospack.get_path("map_server")

	# TODO: AS CONFIGURATION FILE
	htech_file = path + "/src/map/htech.pkl"
	boong_file = path + "/src/map/boong.pkl"

	htech_cv = Converter(htech_file, 2000, r=255/255.0, g=236/255.0, b=0/255.0, a=0.8, scale=0.5)
	boong_cv = Converter(boong_file, 2000, r=255/255.0, g=236/255.0, b=0/255.0, a=0.8, scale=0.5)
	
	htech_pub = rospy.Publisher("/rviz/htech_lane", MarkerArray, queue_size=1, latch=True)
	boong_pub = rospy.Publisher("/rviz/boong_lane", MarkerArray, queue_size=1, latch=True)

	rospy.sleep(1)
	while not rospy.is_shutdown():
		htech_pub.publish(htech_cv.ma)
		boong_pub.publish(boong_cv.ma)
		rospy.sleep(1)
