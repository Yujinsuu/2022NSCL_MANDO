#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import sys
import rospkg
import math
import time

rospack = rospkg.RosPack()
path_map = rospack.get_path("map_server")
sys.path.append(path_map + "/src/")
path_frenet=rospack.get_path("cv_agents")
sys.path.append(path_frenet+"/src/")

from object_msgs.msg import Object, PathArray
from std_msgs.msg import Int32MultiArray, Int16

#from frenet import *
from path_map import *

def get_dist(x, y, _x, _y):
	return np.sqrt((x - _x)**2 + (y - _y)**2)

def get_closest_waypoints(x, y, mapx, mapy, prev_wp, start_wp):
	min_len = 1e10
	closest_wp = 0
	index = 0 if prev_wp <= 0 else prev_wp
	if index <= start_wp: index = start_wp

	for i in range(index, len(mapx) - 1):
			_mapx = mapx[i]
			_mapy = mapy[i]
			dist = get_dist(x, y, _mapx, _mapy)
			if dist < min_len:
				min_len = dist
				closest_wp = i
	
	return closest_wp

class State:
	def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, dt=0.1, WB=0.72):
		self.x = x
		self.y = y
		self.yaw = yaw
		self.v = v
		self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
		self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))
		self.dt = dt
		self.WB = WB


obj_msg = Object(x=use_map.waypoints[0]['x'][0],
				 y=use_map.waypoints[0]['y'][0], 
				 yaw=use_map.waypoints[0]['x'][0], 
				 v=1, L=1.400, W=0.72 ) 

def callback_car(msg):
	global obj_msg
	obj_msg=msg

def TFlight_callback(msg):
	global light
	light = msg.data[0]
	# -1 : None, 0 : green,  1 : left, 
	#  2 : red,  3 : stleft, 4 : yellow

def check_link_speed_flag(my_wp, link_ind) :

	global flag, mode, tf_count, escape
	if flag >= 0:
		if my_wp >= (use_map.link_wp[link_ind]) - 5:
			if link_ind != len(use_map.link_change) - 1 :
				link_ind += 1
			else: flag = -1
	
	if flag == 0: tf_count = 0; escape = 0
	
	mode = use_map.car_mode[link_ind]
	if mode == 'T': 
		escape+=1
		if escape <= 40: flag =3
		elif escape <= 440:
			target_speed = use_map.target_speed[7]
			if light == 2: tf_flag= 1
			if tf_flag == 1 and (light != 2 and light!= 4 and light != -1):
				link_ind += 1
		else: link_ind += 1
	
	elif mode == 'St': 
		target_speed = use_map.target_speed[1]
		if my_wp >= use_map.brake_to_stop[0]: 
			flag = 4
			link_ind += 1

	elif mode == 'Sl': 
		target_speed = use_map.target_speed[6]
		if my_wp >= use_map.slope[0]: 
			flag = 5
			link_ind += 1

	elif mode == 'B': 
		target_speed = use_map.target_speed[3]
		if my_wp >= use_map.stop_to_park[0]: 
			flag = 6
			link_ind += 1
		
	elif mode == 'P': 
		target_speed = use_map.target_speed[4]
		if my_wp >= use_map.stop_to_park[1]:
			flag = 7
			link_ind += 1

	elif mode == 'F': target_speed = use_map.target_speed[0]; flag = 1
	elif mode == 'R': target_speed = use_map.target_speed[0]
	elif mode == 'Kt': target_speed = use_map.target_speed[1]; flag = 2
	elif mode == 'Ks': target_speed = use_map.target_speed[6]
	elif mode == 'L': target_speed = use_map.target_speed[3]
	elif mode == 'A': target_speed = use_map.target_speed[5]; flag = 0
	elif mode == 'E': target_speed = use_map.target_speed[7]; flag = -1
	else: target_speed = use_map.target_speed[1]; flag = 0
	
	# flag = 0 : global, 1 : lidar,  2 : tf_go,    3 : tf_stop
	# flag = 4 : stop,   5 : slope,  6 : backward, 7 : park, 
	# flag =-1 : end

	return link_ind, target_speed, flag 


def path_array(x, y, yaw):
	p=PathArray()
	p.x.data= x
	p.y.data= y
	p.yaw.data = yaw
	return p

def my_state_array(ind, wp, speed, flag):
	m = Int32MultiArray()
	m.data = [ ind, wp, speed, flag ]
	return m

if __name__ == "__main__":
    
	rospy.init_node("path")
	rospy.Subscriber("/objects/car_1", Object, callback_car, queue_size=1)
	rospy.Subscriber("/traffic_light", Int32MultiArray, TFlight_callback, queue_size=1)

	global_path_pub = rospy.Publisher("/path_global", PathArray, queue_size=1)
	waypoint_pub = rospy.Publisher("/waypoint", Int32MultiArray, queue_size=1)
	
	r = rospy.Rate(10)

	mode='G'
	my_wp = 0
	flag = 0
	link_ind = start_index
	opt_ind = 0
	light = 2
	count = 0

	state=State(x=obj_msg.x, y=obj_msg.y, yaw=obj_msg.yaw, v=1, dt=0.1)
	my_wp=get_closest_waypoints( 
		state.x, state.y, 
		use_map.waypoints[0]['x'][:use_map.link_wp[link_ind]], 
		use_map.waypoints[0]['y'][:use_map.link_wp[link_ind]], 
		my_wp-10, 0)
	
	while not rospy.is_shutdown():

		if link_ind <= 1: link_start_wp = 0
		elif mode == 'B' or mode == 'L': link_start_wp = use_map.link_change[link_ind-1] + 1
		else: link_start_wp = use_map.link_change[link_ind-2] + 1
		
		state = State(x=obj_msg.x, y=obj_msg.y, yaw=obj_msg.yaw, v=obj_msg.v, dt=0.1)	
		path_msg = path_array(
			use_map.waypoints[0]['x'][link_start_wp:use_map.link_wp[link_ind]],
			use_map.waypoints[0]['y'][link_start_wp:use_map.link_wp[link_ind]], 
			use_map.waypoints[0]['yaw'][link_start_wp:use_map.link_wp[link_ind]])
		

		my_wp = get_closest_waypoints(
			state.x, state.y, 
			use_map.waypoints[0]['x'][:use_map.link_wp[link_ind]], 
			use_map.waypoints[0]['y'][:use_map.link_wp[link_ind]], 
			my_wp-5, link_start_wp)

		link_ind, target_speed, flag = check_link_speed_flag(my_wp,link_ind)

		print("link num : "+ str(link_ind)+ ", my wp : "+ str(my_wp) + ", speed : "+str(target_speed) + ", flag: "+str(flag))

		waypoint_pub.publish( my_state_array(link_ind, my_wp, target_speed, flag) ) # 
		global_path_pub.publish(path_msg)

		r.sleep()
