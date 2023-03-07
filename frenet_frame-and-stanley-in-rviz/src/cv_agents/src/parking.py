#!/usr/bin/python
#-*- coding: utf-8 -*

from object_msgs.msg import Object
import rospy
import numpy as np
import math, time
import tf
import matplotlib.pyplot as plt
import rospkg
import sys

from ackermann_msgs.msg import AckermannDriveStamped
from scipy.interpolate import interp1d
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion
from object_msgs.msg import Object, ObjectArray
from std_msgs.msg import String, Float64, Int32, Int16
from rocon_std_msgs.msg import StringArray

import pickle
import argparse

from frenet import *
from stanley import *

rospack = rospkg.RosPack()
path_map = rospack.get_path("map_server")
sys.path.append(path_map + "/src/")
from map_visualizer import Converter

rn_id = dict()

rn_id[5] = { 'right': [ i for i in range(7)] }  # ego }

#link_dir={'straight':[0,4,6],'parking':[1,2,3],'fast':[5]}

def find_dir(link_dict, link_ind): # straight 안에 있는 숫자들이랑 비교해서 맞춰봄 맞으면 리턴
	for i in link_dict.keys():
		for j in link_dict[i]:
			if link_ind == j:
				return i

# link_len =[ 9, 19, 29, 39, 49, 59, 89] 각 링크에 포함된 x값들의 개수
# link_ind = 링크 번호 

def find_link(link_len, my_wp):
	for i in range(len(link_len)-1):
		if my_wp > link_len[i] and my_wp <= link_len[i+1]:
			return i+1
		elif my_wp < link_len[0]:
			return i

def pi_2_pi(angle):
	return (angle + math.pi) % (2 * math.pi) - math.pi


def interpolate_waypoints(wx, wy, space=0.5):
	_s = 0
	s = [0]
	for i in range(1, len(wx)):
		prev_x = wx[i - 1]
		prev_y = wy[i - 1]
		x = wx[i]
		y = wy[i]

		dx = x - prev_x
		dy = y - prev_y

		_s = np.hypot(dx, dy)
		s.append(s[-1] + _s)

	fx = interp1d(s, wx)
	fy = interp1d(s, wy)
	ss = np.linspace(0, s[-1], num=int(s[-1] / space) + 1, endpoint=True)

	dxds = np.gradient(fx(ss), ss, edge_order=1)
	dyds = np.gradient(fy(ss), ss, edge_order=1)
	wyaw = np.arctan2(dyds, dxds)

	return {
		"x": fx(ss),
		"y": fy(ss),
		"yaw": wyaw,
		"s": ss
	}


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

	def update(self, a, delta):
		dt = self.dt
		WB = self.WB

		self.x += self.v * math.cos(self.yaw) * dt
		self.y += self.v * math.sin(self.yaw) * dt
		self.yaw += self.v / WB * math.tan(delta) * dt
		self.yaw = pi_2_pi(self.yaw)
		self.v += a * dt
		pwm_vel = int(self.v * 3.6 * 16)
		self.v = pwm_vel / 16 / 3.6
		if self.v > 16/3.6: self.v = 16/3.6
		self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
		self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

		obj_msg.x = self.x
		obj_msg.y = self.y
		obj_msg.yaw = self.yaw
		obj_msg.v = self.v
		obj_msg.W = self.WB


def get_ros_msg(x, y, yaw, v, a, steer, id):
	quat = tf.transformations.quaternion_from_euler(0, 0, yaw)  

	m = Marker()
	m.header.frame_id = "/map"
	m.header.stamp = rospy.Time.now()

	m.id = id
	m.type = m.CUBE 
	m.pose.position.x = x + 1.3 * math.cos(yaw)
	m.pose.position.y = y + 1.3 * math.sin(yaw)
	m.pose.position.z = 0.75
	m.pose.orientation = Quaternion(*quat)  

	m.scale.x = 1.4
	m.scale.y = 0.78
	m.scale.z = 0.53

	m.color.r = 93 / 255.0
	m.color.g = 122 / 255.0
	m.color.b = 180 / 255.0
	m.color.a = 0.97

	o = Object()
	o.header.frame_id = "/map"
	o.header.stamp = rospy.Time.now()
	o.id = id
	o.classification = o.CLASSIFICATION_CAR
	o.x = x
	o.y = y
	o.yaw = yaw
	o.v = v
	o.L = m.scale.x
	o.W = m.scale.y

	c = AckermannDriveStamped()
	c.header.frame_id = "/map"
	c.header.stamp = rospy.Time.now()
	if parking_flag == 2:
		c.drive.steering_angle = int(-steer)
		c.drive.acceleration = a
		c.drive.speed = int(-v * 3.6 * 16)
	else:
		c.drive.steering_angle = int(steer)
		c.drive.acceleration = a
		c.drive.speed = int(v * 3.6 * 16)

	return {
		"object_msg": o,
		"marker_msg": m,
		"quaternion": quat,
		"ackermann_msg" : c
	}

def yaw_reverse(yaw):
    if yaw<0:
        yaw = yaw + math.pi
    else:
        yaw = yaw - math.pi
    return yaw

def auto_callback(mode):
	global auto_drive
	auto_drive = mode.data

obj_msg = Object(x=962582.237483, y=1959236.98703, yaw=5.49779, v=1, L=1.400, W=0.72 ) # 주차미션
obs_info = [ ]

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='Spawn a CV agent')
    
	parser.add_argument("--id", "-i", type=int, help="agent id", default=1)
	parser.add_argument("--route", "-r", type=int,
					help="start index in road network. select in [1, 3, 5, 10]", default=5)
	parser.add_argument("--dir", "-d", type=str, default="right", help="direction to go: [left, straight, right]")
	args, unknown = parser.parse_known_args()
    
	rospy.init_node("three_cv_agents_node_" + str(args.id))
	rospy.Subscriber("/objects/marker/car_1", Marker, queue_size=1)
	rospy.Subscriber("/auto_mode", Int16, auto_callback, queue_size=1)

	global auto_drive
	auto_drive = 0

	id = args.id
	tf_broadcaster = tf.TransformBroadcaster()
	
	object_pub = rospy.Publisher("/objects/car_1", Object, queue_size=1)
	opt_frenet_pub = rospy.Publisher("/rviz/optimal_frenet_path", MarkerArray, queue_size=1)
	cand_frenet_pub = rospy.Publisher("/rviz/candidate_frenet_paths", MarkerArray, queue_size=1)
	control_pub = rospy.Publisher("/ackermann_cmd", AckermannDriveStamped, queue_size=1)
	pid_pub = rospy.Publisher("/cte_data", Float64, queue_size=1)
	path_mode_pub = rospy.Publisher("/path_mode", Int32, queue_size=1)
	
	r = rospy.Rate(10)

	cte=Float64()
	path_flag=Int32()
	path_flag.data = 0
    
	start_node_id = args.route
	route_id_list = rn_id[start_node_id][args.dir]

	with open(path_map + "/src/map/fr_park_s.pkl", "rb") as f:
		nodes = pickle.load(f)

	node_wp_num=[]
	node_wp_num=[0,63,76,122,130, 500, 600, 700]
    #node_wp_num=[754,706,600,511,390,200,100,0]
	for i in reversed(range(len(node_wp_num)-1)):
		nodes[i]={
			'x'  :nodes[0]['x'][node_wp_num[i]:node_wp_num[i+1]], 
			'y'  :nodes[0]['y'][node_wp_num[i]:node_wp_num[i+1]], 
			's'  :nodes[0]['s'][node_wp_num[i]:node_wp_num[i+1]], 
			'yaw':nodes[0]['yaw'][node_wp_num[i]:node_wp_num[i+1]]
			}
        
    #to SPlite link (pickle file of 'x')
	link_i=-1
	link_len=[]
	for i in range(len(nodes)):
		link_i+=len(nodes[i]["x"])
		link_len.append(link_i)
    
	link_ind=2
    
	wx = []
	wy = []
	wyaw = []
	for _id in route_id_list:
		wx.append(nodes[_id]["x"][1:])
		wy.append(nodes[_id]["y"][1:])
		wyaw.append(nodes[_id]["yaw"][1:])
	wx = np.concatenate(wx)
	wy = np.concatenate(wy)
	wyaw = np.concatenate(wyaw)

	# ws = np.zeros(wx.shape)
	# for i in range(len(ws)):
	# 	x = wx[i]
	# 	y = wy[i]
	# 	sd = get_frenet(x, y, wx, wy)
	# 	ws[i] = sd[0]

	waypoints = interpolate_waypoints(wx, wy, space=0.5)
	#waypoints = {"x": wx, "y": wy, "yaw": wyaw, "s" : ws}  
    
	mapx = waypoints["x"]
	mapy = waypoints["y"]
	mapyaw = waypoints["yaw"]
	maps = waypoints["s"]   
	
	ind = 10
	target_speed = 5.0 / 3.6  # km/h -> m/s
	basic_speed = target_speed
	back_speed = 3.0 / 3.6
	state = State(x=mapx[ind], y=mapy[ind], yaw=mapyaw[ind], v=1, dt=0.1)

	state.x=obj_msg.x
	state.y=obj_msg.y
	state.yaw=obj_msg.yaw
	state.v=obj_msg.v
	
	my_wp = 0 # waypoint initialization
	my_wp = get_first_waypoints(state.x, state.y, mapx, mapy)
	prev_wp = my_wp - 1
    
	prev_v = state.v
	error_ia = 0

	ai = 0

	prev_ind = link_ind-2 # link_ind = 2
	
	s, d = get_frenet(state.x, state.y, mapx, mapy, prev_wp)
	x, y, road_yaw = get_cartesian(s, d, mapx, mapy, maps, prev_wp)
	yawi = state.yaw - road_yaw
	si = s
	si_d = state.v * math.cos(yawi)
	si_dd = ai * math.cos(yawi)
	sf_d = target_speed
	sf_dd = 0

	di = d
	di_d = state.v * math.sin(yawi)
	di_dd = ai * math.sin(yawi)
	df_d = 0
	df_dd = 0
	
	opt_d = d
	prev_opt_d = d

	opt_frenet_path = Converter(r=0, g=255/255.0, b=100/255.0, a=1, scale=0.7)
	cand_frenet_paths = Converter(r=0, g=100/255.0, b=100/255.0, a=0.4, scale= 0.2)

	global parking_flag
	parking_flag =0

	while not rospy.is_shutdown():
		# generate acceleration ai, and steering di
		# YOUR CODE HERE
		path, opt_ind = frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs_info, mapx, mapy, maps, opt_d, target_speed, prev_wp)
		# update state with acc, delta

		if opt_ind == -1: ## no solution
			s, d = get_frenet(state.x, state.y, mapx, mapy, prev_wp)
			x, y, road_yaw = get_cartesian(s, d, mapx, mapy,maps, prev_wp)
			steer = road_yaw - state.yaw
			a = 0
			opt_d = prev_opt_d

		else:
			error_pa = target_speed - state.v
			error_da = state.v - prev_v
			error_ia += target_speed - state.v
			kp_a = 0.5
			kd_a = 0.7
			ki_a = 0.01
			a = kp_a * error_pa + kd_a * error_da + ki_a * error_ia
			if parking_flag == 2: # 후진
				#steer, _, cte.data = stanley_control_back(state.x, state.y, state.yaw, state.v, path[opt_ind].x, path[opt_ind].y, path[opt_ind].yaw, state.WB)
				steer, _, cte.data = stanley_control(state.x, state.y, state.yaw, state.v, path[opt_ind].x, path[opt_ind].y, path[opt_ind].yaw, state.WB)
			else:
				steer, _, cte.data = stanley_control(state.x, state.y, state.yaw, state.v, path[opt_ind].x, path[opt_ind].y, path[opt_ind].yaw, state.WB)
			
			ways = []
			for p in path:
				way = {
					"x" : p.x,
					"y" : p.y
				}	
				ways.append(way)
			opt_frenet_path.make_marker_array([ways[opt_ind]])
			cand_frenet_paths.make_marker_array(ways)

			opt_d = path[opt_ind].d[-1]
			prev_opt_d = path[opt_ind].d[-1]

		steer_deg = steer * 180 / np.pi
		if steer_deg > 25: steer_deg = 25
		elif steer_deg < -25: steer_deg = -25
		steer = int(steer_deg) * np.pi /180
	
		ai = a
		if auto_drive == 1: state.update(a, steer)
		print("speed = " + str(int(state.v * 3.6)) + " km/h ,steer = " + str(int(steer_deg)) + ",a = "+str(a))

		state.x = obj_msg.x
		state.y = obj_msg.y
		state.yaw = obj_msg.yaw
		state.v = obj_msg.v
		
		my_wp = get_closest_waypoints(state.x,state.y, mapx, mapy, prev_wp)
		prev_wp = my_wp - 1

		if parking_flag == 0 and my_wp >= 62: # 후진 준비
			target_speed = 0
			parking_flag+=1
		
		elif parking_flag == 1 and my_wp >= 76: # 후진
			target_speed = back_speed
			state.yaw = yaw_reverse(state.yaw)
			parking_flag+=1
		
		elif parking_flag == 2 and my_wp >= 122: # 주정차
			target_speed = 0
			state.yaw = yaw_reverse(state.yaw)
			path_flag.data = 4
			parking_flag+=1
		
		elif parking_flag == 3 and auto_drive == 1: # 출발
			target_speed = basic_speed
			state.yaw = yaw_reverse(state.yaw)
			path_flag.data = 0
			parking_flag+=1
		

		s, d = get_frenet(state.x, state.y, mapx, mapy, prev_wp)
		x, y, road_yaw = get_cartesian(s, d, mapx, mapy, maps, prev_wp)
		yaw_diff = state.yaw - road_yaw

		si = s
		si_d = state.v * math.cos(yaw_diff)
		si_dd = ai * math.cos(yaw_diff)
		sf_d = target_speed
		sf_dd = 0
		
		di = d
		di_d = state.v * math.sin(yaw_diff)
		di_dd = ai * math.sin(yaw_diff)
		df_d = 0
		df_dd = 0

		# vehicle state --> topic msg
		msg = get_ros_msg(state.x, state.y, state.yaw, state.v, ai, steer_deg, id=id)
		speed_msg = msg["ackermann_msg"]

		# send tf
		tf_broadcaster.sendTransform(
			(state.x, state.y, 1.5),
			msg["quaternion"],
			rospy.Time.now(),
			"/car_" + str(id), "/map"
		)

		# publish vehicle state in ros msg
		object_pub.publish(msg["object_msg"])
		opt_frenet_pub.publish(opt_frenet_path.ma)
		cand_frenet_pub.publish(cand_frenet_paths.ma)
		control_pub.publish(msg["ackermann_msg"])
		pid_pub.publish(cte)
		path_mode_pub.publish(path_flag)

		r.sleep()