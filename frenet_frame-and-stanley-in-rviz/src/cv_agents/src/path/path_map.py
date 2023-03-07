#!/usr/bin/python
#-*- coding: utf-8 -*-

import pickle
import sys
import rospkg
import numpy as np
from scipy.interpolate import interp1d
from object_msgs.msg import Object

rospack = rospkg.RosPack()
path_map = rospack.get_path("map_server")
path_frenet=rospack.get_path("cv_agents")
sys.path.append(path_frenet+"/src/")
sys.path.append(path_map + "/src/")

class MakingPath:
	def __init__(self):
		self.x = []
		self.y = []
		self.yaw = []

class Path:
	def __init__(self,pc_route):
	
		self.global_route=pc_route #global pkl파일 경로
		self.waypoints={} 

		with open(pc_route, 'rb') as f:
			self.waypoints=pickle.load(f)
		
		self.target_speed = [ 5, 5, 3, 4, 3, 7, 8, 0 ] # [start, global, traffic, park, back, accel, slope, end]
		self.link_wp = []
		self.link_change =[]
		self.car_mode = []
		# F  : first_speed, G  : global,    A  : accel,     E : end
		# T  : traffic,     Kt : to_global, B  : backward,  P : parking,  L : low_speed, 
		# Sl : slope,       Ks : to_global, St : stop,      R : railroad
	

		# set waypoint
		self.brake_to_stop = [0]
		self.slope = [0]
		self.traffic_light = [0,0]
		self.stop_to_park = [0,0]
		
	def set_global_link(self, waypoint_list=[0]): # waypoint 안에 마지막 waypoint 추가 및 0 제거하는 함수
		wp_len=len(self.waypoints[0]['x']) 
		waypoint_list.append(wp_len)
		#waypoint_list.remove(0) # 초기링크 삭제 
		self.link_wp=waypoint_list


def hightech():
	hightech=Path(path_map + "/src/map/htech.pkl")
	hightech.link_change = [30,40,100,150,200,260,370,480,590,700]
	hightech.car_mode = ['G','G','G','G','G','G','G','S','G','G','G']
	hightech.set_global_link(hightech.link_change)

	hightech.traffic_light = [35,350]

	return hightech

def final1():
	final1=Path(path_map + "/src/map/final1.pkl")
	final1.link_change = [20,40,70,80,90,110,120,130,230,336,375,420,560]
	final1.car_mode = ['F','A','G','T','Kt','G','St','R','G','B','P','L','G','E']
	final1.set_global_link(final1.link_change) 

	final1.brake_to_stop=[115]
	final1.stop_to_park = [324, 355]

	return final1

def final2():
	final2=Path(path_map + "/src/map/final2.pkl")
	final2.link_change = [24,86,303,330,350,372,420,440]
	final2.car_mode = ['G','A','G','P','B','L','G','S','E']
	final2.set_global_link(final2.link_change)

	final2.stop_to_park = [324, 343]

	return final2

def ht_park():
	ht_park=Path(path_map + "/src/map/ht_park.pkl")
	ht_park.link_change =[32,52,80]
	ht_park.car_mode = ['P','B','L','E']
	ht_park.set_global_link(ht_park.link_change)
	
	ht_park.stop_to_park = [27, 45]
	
	return ht_park

def fr_park():
	fr_park=Path(path_map + "/src/map/fr_park.pkl")
	fr_park.link_change =[37,67,100]
	fr_park.car_mode = ['B','P','L','E']
	fr_park.set_global_link(fr_park.link_change)
	
	fr_park.stop_to_park = [31, 59]
	
	return fr_park

def slope():
	slope=Path(path_map + "/src/map/slope.pkl")
	slope.link_change=[20,50,80]
	slope.car_mode=['G','S','K','E']
	slope.set_global_link(slope.link_change)

	slope.slope = [40]

	return slope

def mando():
	mando=Path(path_map + "/src/map/mando.pkl")
	#mando.link_change = [85,157,186,476,651,748,830,1425,1435,1454,1517,1690,1855]
	#mando.car_mode = ['F','Sl','Ks','G','B','P','L','G','St','R','G','A','G','E']
	
	#mando.link_change = [85,157,186,424,435,445,476,651,748,830,1243,1253,1263,1425,1435,1454,1517,1690,1855]
	mando.link_change = [85,157,186,425,435,445,476,651,748,830,1243,1253,1263,1425,1435,1454,1517,1690,1855]
	mando.car_mode = ['F','Sl','Ks','G','T','Kt','G','B','P','L','G','T','Kt','G','St','R','G','G','G','E']
	mando.set_global_link(mando.link_change)

	mando.slope = [111]
	mando.brake_to_stop = [1425]
	#mando.traffic_light = [526,1193]
	mando.stop_to_park = [613, 730]

	return mando

def vworld1():
	vworld1=Path(path_map + "/src/map/vworld1.pkl")
	vworld1.link_change = [160,190,196,500,548,607,707,769,800,1165,1211,1500,1700,1900]
	vworld1.car_mode = ['G','S','K','G','T_st','G','B','P','L','G','T_l','G','A','G','E']
	vworld1.set_global_link(vworld1.link_change)

	vworld1.slope = [175]
	vworld1.traffic_light = [526,1193]
	vworld1.stop_to_park = [674, 756]

	return vworld1

def S_course():
	S_course=Path(path_map + "/src/map/S1.pkl")
	S_course.link_change = [65]
	S_course.car_mode = ['G','E']
	S_course.set_global_link(S_course.link_change)

	return S_course

def T_course():
	T_course=Path(path_map + "/src/map/T2.pkl")
	T_course.link_change = [500]
	T_course.car_mode = ['G','E']
	T_course.set_global_link(T_course.link_change)

	return T_course

use_map=final1()

start_index=0

if start_index==0:
    obj_msg=Object(
		x=use_map.waypoints[0]['x'][:use_map.link_wp[start_index]],
		y=use_map.waypoints[0]['y'][:use_map.link_wp[start_index]],
		yaw=use_map.waypoints[0]['yaw'][:use_map.link_wp[start_index]]
		,v=0,L=1.400,W=0.72)

else:
	obj_msg=Object(
		x=use_map.waypoints[0]['x'][use_map.link_wp[start_index-1]:use_map.link_wp[start_index]],
		y=use_map.waypoints[0]['y'][use_map.link_wp[start_index-1]:use_map.link_wp[start_index]],
		yaw=use_map.waypoints[0]['yaw'][use_map.link_wp[start_index-1]:use_map.link_wp[start_index]],
		v=0,L=1.400,W=0.72)
