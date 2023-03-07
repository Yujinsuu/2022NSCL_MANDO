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
from std_msgs.msg import String, Float64, Int32
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
        if self.v > 16/3.6: self.v = 16/3.6
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))
        
    
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
    c.drive.steering_angle = int(steer)
    c.drive.acceleration = a
    c.drive.speed = int(v * 3.6 * 16)
    
    return {
		"object_msg": o,
		"marker_msg": m,
		"quaternion": quat,
        "ackermann_msg" : c
	}
    
obj_msg = Object(x=962587.18002729, y=1959280.6349655, yaw=-1.83849268, v=1, L=1.400, W=0.72 ) # 차량 첫 위치
obs_info = [ ]

def state_callback(msg):
    global obj_msg
    obj_msg = msg

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Spawn a CV agent')
    
    parser.add_argument("--id", "-i", type=int, help="agent id", default=1)
    parser.add_argument("--route", "-r", type=int,
					help="start index in road network. select in [1, 3, 5, 10]", default=5)
    parser.add_argument("--dir", "-d", type=str, default="right", help="direction to go: [left, straight, right]")
    args, unknown = parser.parse_known_args()
    
    rospy.init_node("three_cv_agents_node_" + str(args.id))
    #sub_obs = rospy.Subscriber("/objects/marker/car_1", Marker, callback3, queue_size=1)
    sub_state = rospy.Subscriber("/objects/car_1", Object, state_callback, queue_size=1)
    WB = 0.72
    
    
    id = args.id
    tf_broadcaster = tf.TransformBroadcaster()
    
    opt_frenet_pub = rospy.Publisher("/rviz/optimal_frenet_path", MarkerArray, queue_size=1)
    cand_frenet_pub = rospy.Publisher("/rviz/candidate_frenet_paths", MarkerArray, queue_size=1)
    control_pub = rospy.Publisher("/ackermann_cmd", AckermannDriveStamped, queue_size=1)
    pid_pub = rospy.Publisher("/cte_data", Float64, queue_size=1)
    flag_traffic = rospy.Publisher("/flag_traffic",Int32, queue_size=1 )
    cte=Float64()
    
    start_node_id = args.route
    route_id_list = rn_id[start_node_id][args.dir]
    
    with open(path_map + "/src/hightech_clockwise.pkl", "rb") as f:
        nodes = pickle.load(f)
    
    node_wp_num=[]
    node_wp_num=[0,100,200,390,511,600,706,753]
    #node_wp_num=[754,706,600,511,390,200,100,0]
    for i in reversed(range(len(node_wp_num)-1)):
        nodes[i]={'x':nodes[0]['x'][node_wp_num[i]:node_wp_num[i+1]], 'y':nodes[0]['y'][node_wp_num[i]:node_wp_num[i+1]], 's':nodes[0]['s'][node_wp_num[i]:node_wp_num[i+1]], 'yaw':nodes[0]['yaw'][node_wp_num[i]:node_wp_num[i+1]]}
        
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


    waypoints = interpolate_waypoints(wx, wy, space=0.5)
    #waypoints = {"x": wx, "y": wy, "yaw": wyaw, "s" : ws}
    mapx = waypoints["x"]
    mapy = waypoints["y"]
    mapyaw = waypoints["yaw"]
    maps = waypoints["s"]
    
    target_speed = 5.0 / 3.6 # km/h -> m/s
    state=State(x=obj_msg.x, y=obj_msg.y, yaw=obj_msg.yaw, v=1, dt=0.1)
    state.x=obj_msg.x
    state.y=obj_msg.y
    
    state.yaw=obj_msg.yaw
    state.v=obj_msg.v
    

    my_wp = 0 # waypoint initialization
    my_wp = get_closest_waypoints(state.x, state.y, mapx, mapy)
    
    prev_v = state.v
    error_ia = 0
    r = rospy.Rate(5)
    ai = 0
    
    prev_ind = link_ind-2 # link_ind = 2
    
    
    #s, d = get_frenet(state.x, state.y, mapx, mapy)
    #x, y, road_yaw = get_cartesian(s, d, mapx, mapy, maps)
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
    
    opt_frenet_path = Converter(r=0, g=255/255.0, b=100/255.0, a=1, scale=0.9)
    cand_frenet_paths = Converter(r=0, g=100/255.0, b=100/255.0, a=0.4, scale= 0.5)
    
    while not rospy.is_shutdown():
		# generate acceleration ai, and steering di
		# YOUR CODE HERE
        
        path, opt_ind = frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs_info, mapx, mapy, maps, opt_d, target_speed)
		# update state with acc, delta
        if opt_ind == -1: ## no solution
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
            
            steer, _, cte.data = stanley_control(state.x, state.y, state.yaw, state.v, path[opt_ind].x, path[opt_ind].y, path[opt_ind].yaw, state.WB)
            steer_deg = steer * 180 / np.pi
            if steer_deg >= 0.1 and steer_deg <= 1: steer_deg = 1
            elif steer_deg <= -0.1 and steer_deg >=-1: steer_deg = -1
            elif abs(steer_deg) < 0.1: steer_deg = 0
            steer = steer_deg * np.pi /180

            ways = []
            for p in path:
                way = { "x" : p.x, "y" : p.y }	
                ways.append(way)
            opt_frenet_path.make_marker_array([ways[opt_ind]])
            cand_frenet_paths.make_marker_array(ways)
            
            opt_d = path[opt_ind].d[-1]
            prev_opt_d = path[opt_ind].d[-1]
		
        '''
        
		# vehicle state --> topic msg
		# state.update(a, steer)
		if ((my_wp < (link_len[-1] -10)) & (obj_msg.v <= 1)):
			msg = state.get_ros_msg(0, steer, 1.0)

		else:
			msg = state.get_ros_msg(a, steer, obj_msg.v)
		control_pub.publish(msg)
		#a_list.append(a)
		#steer_list.append(steer)
		#v_list.append(v)'''
        
        ai=a
        state.update(a, steer)
         # 차량 첫 위치
        print(" 현재 speed = " + str(int(state.v * 3.6)) + ",steer = " + str(int(steer_deg)) + ",a = "+str(a))
        prev_v = state.v
        state.x=obj_msg.x
        state.y=obj_msg.y
        state.yaw=obj_msg.yaw
        #state.v=obj_msg.v
        my_wp = get_closest_waypoints(state.x,state.y, mapx, mapy)
        #my_wp = get_closest_waypoints(state.x,state.y, mapx[:link_len[link_ind]], mapy[:link_len[link_ind]],my_wp)
        if (my_wp > nodes['x'][37] and my_wp < nodes['x'][60] ) :
            flag =1
        elif (my_wp > nodes['x'][80] and my_wp < nodes['x'][120] ) :
            flag =2
        else : 
            flag =0   

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
        # vehicle state --> topic msg
        msg = get_ros_msg(state.x, state.y, state.yaw, state.v, ai, steer_deg, id=id)
        
        # send tf	
        #tf_broadcaster.sendTransform(
        #(state.x, state.y, 1.5),
        #msg["quaternion"],
        #rospy.Time.now(),
        #"/car_" + str(id), "/map"#)
        # publish vehicle state in ros msg
        #object_pub.publish(msg["object_msg"])
        flag_traffic.publish(flag)
        opt_frenet_pub.publish(opt_frenet_path.ma)
        cand_frenet_pub.publish(cand_frenet_paths.ma)
        control_pub.publish(msg["ackermann_msg"])
        pid_pub.publish(cte)

        r.sleep()
