#!/usr/bin/python
#-*- coding: utf-8 -*-
import rospy
import math
import rospkg
import sys
from ackermann_msgs.msg import AckermannDriveStamped
import time
import numpy as np

from object_msgs.msg import Object, PathArray
from std_msgs.msg import Int32MultiArray

from frenet import *
from stanley import *

rospack = rospkg.RosPack()
path_map = rospack.get_path("cv_agents")
sys.path.append(path_map + "/src/path")

from path_map import *

def yaw_reverse(yaw):
    index = math.pi if yaw<=0 else -math.pi
    return yaw + index

def pi_2_pi(angle):
	return (angle + math.pi) % (2 * math.pi) - math.pi

def state_callback(msg):
    global obj_msg
    obj_msg = msg
    
    if flag == 6 : obj_msg.yaw = yaw_reverse(obj_msg.yaw)

path_x=[]
path_y=[]
path_yaw=[]

def path_callback(msg):
	global path_x,path_y,path_yaw
	path_x   = msg.x.data
	path_y   = msg.y.data
	path_yaw = msg.yaw.data

def link_callback(msg):
    global link_ind, my_wp, target_speed, flag
    link_ind=msg.data[0]
    my_wp=msg.data[1]
    target_speed =msg.data[2]
    flag = msg.data[3]

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, dt=0.1, WB=0.72 ):
        self.x = x
        self.y = y
        self.yaw = pi_2_pi(yaw)
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))
        self.dt = dt
        self.WB = WB

def rad_to_deg(steer):
    steer_deg = pi_2_pi(steer) * 180 / np.pi
    if steer_deg > 25: steer_deg = 25
    elif steer_deg < -25: steer_deg = -25
    
    return steer_deg

def get_ros_msg(v, steer):   
    c = AckermannDriveStamped()
    c.header.frame_id = "/map"
    c.header.stamp = rospy.Time.now()

    rev = -1 if flag == 6 else 1
    c.drive.steering_angle = int(rev * steer)
    c.drive.acceleration = 0
    c.drive.speed = int(rev * v * 16)

    return c
    
obj_msg = Object(x=use_map.waypoints[0]['x'][0],
				 y=use_map.waypoints[0]['y'][0], 
				 yaw=use_map.waypoints[0]['yaw'][0], 
				 v=1, L=1.400, W=0.72 ) 

if __name__ == "__main__":

    WB = 0.72

    rospy.init_node("control")
    rospy.Subscriber("/objects/car_1", Object, state_callback, queue_size=1)
    rospy.Subscriber("/waypoint", Int32MultiArray, link_callback, queue_size=1)
    rospy.Subscriber("/path_global", PathArray, path_callback, queue_size=1)

    control_pub = rospy.Publisher("/ackermann_cmd", AckermannDriveStamped, queue_size=1)

    r = rospy.Rate(10)

    target_speed = use_map.target_speed[0]
    steer_deg = 0
    flag =0
    link_ind=0
    my_wp=0

    path_x = use_map.waypoints[0]['x'][:use_map.link_wp[link_ind]]
    path_y = use_map.waypoints[0]['y'][:use_map.link_wp[link_ind]] 
    path_yaw = use_map.waypoints[0]['yaw'][:use_map.link_wp[link_ind]]

    state = State(x=obj_msg.x, y=obj_msg.y, yaw=obj_msg.yaw, v=1, dt=0.1)

    while not rospy.is_shutdown():

        state = State(x=obj_msg.x, y=obj_msg.y, yaw=obj_msg.yaw, v=target_speed, dt=0.1)

        if flag == 6:
            steer,_,_ = stanley_control_back(state.x, state.y, state.yaw, state.v / 3.6, path_x, path_y, path_yaw, state.WB)
        else:
            steer,_,_ = stanley_control(state.x, state.y, state.yaw, state.v / 3.6, path_x, path_y, path_yaw, state.WB)

        steer_deg = rad_to_deg(steer)
        
        print("speed = " + str(int(state.v)) + " km/h, steer = " + str(int(steer_deg)) + ", yaw = " + str(state.yaw))
    
        msg = get_ros_msg(state.v, steer_deg)
        control_pub.publish(msg)

        r.sleep()