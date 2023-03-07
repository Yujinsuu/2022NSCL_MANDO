#!/usr/bin/python
#-*- coding: utf-8 -*-

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

from frenet import *
from stanley import *

rospack = rospkg.RosPack()
path_map = rospack.get_path("map_server")
sys.path.append(path_map + "/src/")
from map_visualizer import Converter


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
    
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, dt=0.1, WB=0.72 ):
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
        
        if self.v > 16/3.6: 
            self.v = 16/3.6
        
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

        if parking_flag == 2:
            self.yaw = yaw_reverse(self.yaw)
            
        
    
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
    m.color.b = 177 / 255.0
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

def state_callback(msg):
    global obj_msg
    obj_msg = msg

def auto_callback(mode):
	global auto_drive
	auto_drive = mode.data

obs_info = []
obj_msg = Object(x=962587.18002729, y=1959280.6349655, yaw=-1.83849268, v=1, L=1.400, W=0.72 ) 


target_speed_list  = [10/3.6, 5/3.6, 3/3.6, 15/3.6] # KM/H 


node_wp_num=[ 0, 50, 60, 100, 120, 150, 165, 170, 180, 220, 300, 320, 680, 720, 753 ]

stop_list = [50, 60]
trafficlight_list = [100, 120, 300, 320]
parking_list = [150, 165, 170, 180, 220]
fast_list = [ 680, 720 ]

if __name__ == "__main__":
    
    auto_drive = 0
    WB = 0.72
    tf_broadcaster = tf.TransformBroadcaster()

    rospy.init_node("three_cv_agents_node_1")
    rospy.Subscriber("/objects/car_1", Object, state_callback, queue_size=1)
    rospy.Subscriber("/auto_mode", Int16, auto_callback, queue_size=1)

    opt_frenet_pub = rospy.Publisher("/rviz/optimal_frenet_path", MarkerArray, queue_size=1)
    cand_frenet_pub = rospy.Publisher("/rviz/candidate_frenet_paths", MarkerArray, queue_size=1)
    control_pub = rospy.Publisher("/ackermann_cmd", AckermannDriveStamped, queue_size=1)
    pid_pub = rospy.Publisher("/cte_data", Float64, queue_size=1)
    path_mode_pub = rospy.Publisher("/path_mode", Int32, queue_size=1)

    r = rospy.Rate(10)

    cte=Float64()
    path_flag=Int32()
    path_flag.data = 0
    
    ################### 맵 넣고, link 지정해주기 ##########
    with open(path_map + "/src/htech.pkl", "rb") as f:
        nodes = pickle.load(f)
    
    for i in reversed(range(len(node_wp_num)-1)):
        nodes[i] = {
            'x'  :nodes[0]['x'][node_wp_num[i]:node_wp_num[i+1]],
            'y'  :nodes[0]['y'][node_wp_num[i]:node_wp_num[i+1]], 
            's'  :nodes[0]['s'][node_wp_num[i]:node_wp_num[i+1]], 
            'yaw':nodes[0]['yaw'][node_wp_num[i]:node_wp_num[i+1]]
            }
        
    wx = []
    wy = []
    wyaw = []
    
    for _id in nodes.keys():
        wx.append(nodes[_id]["x"][1:])
        wy.append(nodes[_id]["y"][1:])
        wyaw.append(nodes[_id]["yaw"][1:])
    
    wx = np.concatenate(wx)
    wy = np.concatenate(wy)
    wyaw = np.concatenate(wyaw)

    waypoints = interpolate_waypoints(wx, wy, space=0.5)
    #waypoints = {"x": wx, "y": wy, "yaw": wyaw, "s" : ws}
    mapx = waypoints["x"]
    mapy = waypoints["y"]
    mapyaw = waypoints["yaw"]
    maps = waypoints["s"]
    
    target_speed = target_speed_list[0] # km/h -> m/s

    state=State(x=obj_msg.x, y=obj_msg.y, yaw=obj_msg.yaw, v=1, dt=0.1)
    #state.x=obj_msg.x
    #state.y=obj_msg.y
    #state.yaw=obj_msg.yaw
    state.v=obj_msg.v
    

    my_wp = 0 # waypoint initialization
    my_wp = get_closest_waypoints(state.x, state.y, mapx, mapy)
    
    prev_v = state.v
    error_ia = 0
    ai = 0

    s, d = get_frenet(state.x, state.y, mapx, mapy)
    x, y, road_yaw = get_cartesian(s, d, mapx, mapy,maps)
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
    cand_frenet_paths = Converter(r=0, g=100/255.0, b=100/255.0, a=0.4, scale= 0.3)
    
    parking_flag = 0

    while not rospy.is_shutdown():        
        path, opt_ind = frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs_info, mapx, mapy, maps, opt_d, target_speed)

        if opt_ind == -1: ## no solution
            target_speed = 0
            my_wp = get_closest_waypoints(state.x,state.y, mapx, mapy)
            s, d = get_frenet(state.x, state.y, mapx, mapy)
            x, y, road_yaw = get_cartesian(s, d, mapx, mapy,maps)
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
                steer, _, cte.data = stanley_control_back(state.x, state.y, state.yaw, state.v, path[opt_ind].x, path[opt_ind].y, path[opt_ind].yaw, state.WB)
            else:
                steer, _, cte.data = stanley_control(state.x, state.y, state.yaw, state.v, path[opt_ind].x, path[opt_ind].y, path[opt_ind].yaw, state.WB)
			
            ways = []
            for p in path:
                way = { "x" : p.x, "y" : p.y }	
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

        prev_v = state.v
        state.x=obj_msg.x
        state.y=obj_msg.y
        state.yaw=obj_msg.yaw
        state.v=obj_msg.v
		
        my_wp = get_closest_waypoints(state.x,state.y, mapx, mapy)
        
        ######################## traffic light track ###############################
        if my_wp >= trafficlight_list[0] and my_wp < trafficlight_list[1]:   # 신호등 직진 구간
            path_flag.data = 1

        elif my_wp >= trafficlight_list[2] and my_wp < trafficlight_list[3]: # 신호등 좌회전 구간
            path_flag.data = 2

        ######################## stop mode #########################################
        elif my_wp >= stop_list[0] and my_wp < stop_list[1]:
            path_flag.data = 3

        ######################## parking mode ######################################
        elif parking_flag == 0 and my_wp >= parking_list[0] and my_wp < parking_list[1]: ## 감속
            target_speed = target_speed_list[1]

        elif parking_flag == 0 and my_wp >= parking_list[1]: 
            target_speed = 0
            parking_flag += 1

        elif parking_flag == 1 and my_wp >= parking_list[2]: # 후진
            target_speed = target_speed_list[2]
            parking_flag += 1

        elif parking_flag == 2 and my_wp >= parking_list[3]: # 주정차
            target_speed = 0
            path_flag.data = 4
            parking_flag += 1

        elif parking_flag == 3 and auto_drive == 1 and my_wp < parking_list[4]: # 출발
            target_speed = target_speed_list[1]
            path_flag.data = 0

        ######################## accel track #######################################
        elif my_wp >= fast_list[0] and my_wp < fast_list[1]: # 가속 구간 
            target_speed = target_speed_list[3]
        
        else:
            target_speed = target_speed_list[0]
            path_flag.data = 0


        s, d = get_frenet(state.x, state.y, mapx, mapy)
        x, y, road_yaw = get_cartesian(s, d, mapx, mapy,maps)
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

        msg = get_ros_msg(state.x, state.y, state.yaw, state.v, ai, steer_deg, id=1)
        speed_msg = msg["ackermann_msg"]

        # send tf
        tf_broadcaster.sendTransform(
        	(state.x, state.y, 1.5),
        	msg["quaternion"],
        	rospy.Time.now(),
        	"/car_" + str(id), "/map"
        )

        # publish vehicle state in ros msg
        opt_frenet_pub.publish(opt_frenet_path.ma)
        cand_frenet_pub.publish(cand_frenet_paths.ma)
        control_pub.publish(msg["ackermann_msg"])
        pid_pub.publish(cte)
        path_mode_pub.publish(path_flag)

        r.sleep()