#-*- coding: utf-8 -*-
from distutils.errors import DistutilsClassError
import numpy as np
import pickle
import matplotlib.pyplot as plt
from copy import deepcopy
import sys
import math
from numpy import *
from matplotlib import *
from scipy.interpolate import interp1d

import rospkg
rospack = rospkg.RosPack()
path = rospack.get_path("obstacles")
sys.path.append(path + "/src/")
path_frenet=rospack.get_path("cv_agents")
sys.path.append(path_frenet+"/src/path")
from separation_axis_theorem import *
from path_map import *


# initialize
# initialize
LANE_WIDTH = 3.6-0.7  # lane width [m]
WB = 0.72
gain = 0

## defalt
MIN_T = 2.0 # minimum terminal time [s]
MAX_T = 8.0 # maximum terminal time [s]
DT_T = 3.0 # dt for terminal time [s]
DT = 0.5 # timestep for update

# km/h MAXT	speed_gap
# 16	3.0		0
# 15	3.0		1
# 14	3.5		2
# 13	3.5		3
# 12	4.0		4
# 11	4.0		5
# 10	4.5		6
# 09	4.5		7
# 08	5.0		8
# 07	5.5		9
# 06	6.0		10
# 05	6.5		11
# 04	7.0		12
# 03	7.5		13


V_MAX = 18/3.6	  # maximum velocity [m/s]
# ACC_MAX=2.0
# ACC_MAX=VMAX/DT_T

ACC_MAX=V_MAX/0.1
#ACC_MAX = VMAX / MINT # maximum acceleration [m/ss]
#ACC_MAX = 99999999999999999999999999999999999999999999

STEER_MAX = math.radians(25)

K_MAX = STEER_MAX / WB	 # maximum curvature [1/m]
#K_MAX = 100
# cost weights
K_J = 0.1 # weight for jerk, default = 0.1 (가가속도를 위한 웨이트)
K_T = 0.1 # weight for terminal time (터미널 타임을 위한 웨이트)
K_D = 1.0 # weight for consistency, default = 1.0 (일관성을 위한 웨이트?)
K_GD = 7.0 # weight for global path tracking (글로벌 패스를 따르는 것에 대한 웨이트)
K_V = 1.0 # weight for getting to target speed (목표 속도로 도달하는 것을 위한 웨이트)
K_LAT = 1.0 # weight for lateral direction, default = 1.0 (횡방향을 위한 웨이트)
K_LON = 1.0 # weight for longitudinal direction (종방향을 위한 웨이트)

# lateral planning 시 terminal position condition 후보  (양 차선 중앙), default len(DF_SET) = 2
DF_SET = np.array([0, LANE_WIDTH/2, -LANE_WIDTH/2, -LANE_WIDTH/7*5])



def next_waypoint(x, y, mapx, mapy, prev_wp):
	closest_wp = get_closest_waypoints(x, y, mapx, mapy, prev_wp)

	map_vec = [mapx[closest_wp + 1] - mapx[closest_wp], mapy[closest_wp + 1] - mapy[closest_wp]]
	ego_vec = [x - mapx[closest_wp], y - mapy[closest_wp]]

	direction  = np.sign(np.dot(map_vec, ego_vec))

	if direction >= 0:
		next_wp = closest_wp + 1
	else:
		next_wp = closest_wp

	return next_wp

def get_first_waypoints(x, y, mapx, mapy):
	min_len = 1e10
	closest_wp = 0
	dist_list = []
	for i in range(len(mapx)):
		_mapx = mapx[i]
		_mapy = mapy[i]
		dist = get_dist(x, y, _mapx, _mapy)
		dist_list.append(dist)
		
		if dist < min_len:
			min_len = dist
			closest_wp = i
	#print(dist_list)
	return closest_wp

def get_closest_waypoints(x, y, mapx, mapy, prev_wp):
	min_len = 1e10
	closest_wp = 0
	index = 0 if prev_wp <= 10 else prev_wp

	for i in range(index, len(mapx) - 1):
			_mapx = mapx[i]
			_mapy = mapy[i]
			dist = get_dist(x, y, _mapx, _mapy)
			if dist < min_len:
				min_len = dist
				closest_wp = i
	
	'''if prev_wp <= 10:
		for i in range(0, len(mapx) - 1):
			_mapx = mapx[i]
			_mapy = mapy[i]
			dist = get_dist(x, y, _mapx, _mapy)
			if dist < min_len:
				min_len = dist
				closest_wp = i
				
	else:
		for i in range(prev_wp-10, len(mapx) - 1):
			_mapx = mapx[i]
			_mapy = mapy[i]
			dist = get_dist(x, y, _mapx, _mapy)
			if dist < min_len:
				min_len = dist
				closest_wp = i'''

	return closest_wp

def get_dist(x, y, _x, _y):
	# print(np.sqrt((x - _x)**2 + (y - _y)**2))
	return np.sqrt((x - _x)**2 + (y - _y)**2)

def get_frenet(x, y, mapx, mapy, prev_wp):
	next_wp = next_waypoint(x, y, mapx, mapy, prev_wp)
	prev_wp = next_wp -1

	print("prev_wp,next_wp: %d, %d"%(prev_wp,next_wp))
	if mapx[next_wp] == mapx[prev_wp]:
		n_x = mapx[next_wp+1] - mapx[prev_wp]
	else:
		n_x = mapx[next_wp] - mapx[prev_wp]
	
	if mapy[next_wp] == mapy[prev_wp]:
		n_y = mapy[next_wp+1] - mapy[prev_wp]
	else:
		n_y = mapy[next_wp] - mapy[prev_wp]
	
	x_x = x - mapx[prev_wp]
	x_y = y - mapy[prev_wp]

	proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y)
	proj_x = proj_norm*n_x
	proj_y = proj_norm*n_y

	#-------- get frenet d
	frenet_d = get_dist(x_x,x_y,proj_x,proj_y)

	ego_vec = [x-mapx[prev_wp], y-mapy[prev_wp], 0]
	map_vec = [n_x, n_y, 0]
	d_cross = np.cross(ego_vec,map_vec)
	if d_cross[-1] > 0:
		frenet_d = -frenet_d

	#-------- get frenet s
	frenet_s = 0
	for i in range(prev_wp):
		frenet_s = frenet_s + get_dist(mapx[i],mapy[i],mapx[i+1],mapy[i+1])

	frenet_s = frenet_s + get_dist(0,0,proj_x,proj_y)
	# print(get_dist(0,0,proj_x,proj_y),proj_x,proj_y,proj_norm,x_x,x_y,n_x,n_y)
	# (nan, nan, nan, nan, 3.5368640741799027, -0.5265787104144692, 0.0, 0.0)
	return frenet_s, frenet_d


def get_cartesian(s, d, mapx, mapy, maps, prev_wp):
	s = np.mod(s, maps[-2])

	while(s > maps[prev_wp+1]) and (prev_wp < len(maps)-2):
		prev_wp = prev_wp + 1
	
	next_wp = np.mod(prev_wp+1,len(mapx))
	# print(prev_wp,next_wp)
	dx = (mapx[next_wp]-mapx[prev_wp])
	dy = (mapy[next_wp]-mapy[prev_wp])

	heading = np.arctan2(dy, dx) # [rad]

	# the x,y,s along the segment
	seg_s = s - maps[prev_wp]

	seg_x = mapx[prev_wp] + seg_s*np.cos(heading)
	seg_y = mapy[prev_wp] + seg_s*np.sin(heading)

	perp_heading = heading + 90 * np.pi/180
	x = seg_x + d*np.cos(perp_heading)
	y = seg_y + d*np.sin(perp_heading)

	return x, y, heading

class QuinticPolynomial:
	def __init__(self, xi, vi, ai, xf, vf, af, T):
		# calculate coefficient of quintic polynomial
		# used for lateral trajectory
		self.a0 = xi
		self.a1 = vi
		self.a2 = 0.5*ai

		A = np.array([[T**3, T**4, T**5],
					  [3*T**2, 4*T**3, 5*T** 4],
					  [6*T, 12*T**2, 20*T**3]])
		b = np.array([xf - self.a0 - self.a1*T - self.a2*T**2,
					  vf - self.a1 - 2*self.a2*T,
					  af - 2*self.a2])
		x = np.linalg.solve(A, b)

		self.a3 = x[0]
		self.a4 = x[1]
		self.a5 = x[2]

	# calculate postition info.
	def calc_pos(self, t):
		x = self.a0 + self.a1*t + self.a2*t**2 + self.a3*t**3 + self.a4*t**4 + self.a5 * t ** 5
		return x

	# calculate velocity info.
	def calc_vel(self, t):
		v = self.a1 + 2*self.a2*t + 3*self.a3*t**2 + 4*self.a4*t**3 + 5*self.a5*t**4
		return v

	# calculate acceleration info.
	def calc_acc(self, t):
		a = 2*self.a2 + 6*self.a3*t + 12*self.a4*t**2 + 20*self.a5*t**3
		return a

	# calculate jerk info.
	def calc_jerk(self, t):
		j = 6*self.a3 + 24*self.a4*t + 60*self.a5*t**2
		return j

class QuarticPolynomial:

	def __init__(self, xi, vi, ai, vf, af, T):
		# calculate coefficient of quartic polynomial
		# used for longitudinal trajectory
		self.a0 = xi
		self.a1 = vi
		self.a2 = 0.5*ai

		A = np.array([[3*T**2, 4*T**3], [6*T, 12*T**2]])
		b = np.array([vf - self.a1 - 2*self.a2*T, af - 2*self.a2])

		x = np.linalg.solve(A, b)

		self.a3 = x[0]
		self.a4 = x[1]

	# calculate postition info.
	def calc_pos(self, t):
		x = self.a0 + self.a1*t + self.a2*t**2 + self.a3*t**3 + self.a4*t**4
		return x

	# calculate velocity info.
	def calc_vel(self, t):
		v = self.a1 + 2*self.a2*t + 3*self.a3*t**2 + 4*self.a4*t**3
		return v

	# calculate acceleration info.
	def calc_acc(self, t):
		a = 2*self.a2 + 6*self.a3*t + 12*self.a4*t**2
		return a

	# calculate jerk info.
	def calc_jerk(self, t):
		j = 6*self.a3 + 24*self.a4*t
		return j

class FrenetPath:

	def __init__(self):
		# time
		self.t = []

		# lateral traj in Frenet frame
		self.d = []
		self.d_d = []
		self.d_dd = []
		self.d_ddd = []

		# longitudinal traj in Frenet frame
		self.s = []
		self.s_d = []
		self.s_dd = []
		self.s_ddd = []

		# cost
		self.c_lat = 0.0
		self.c_lon = 0.0
		self.c_tot = 0.0

		# combined traj in global frame
		self.x = []
		self.y = []
		self.yaw = []
		self.ds = []
		self.kappa = []

def calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d, target_speed, prev_wp):
# def calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d, target_speed):
	frenet_paths = []

	# generate path to each offset goal
	for df in DF_SET:

		# Lateral motion planning
		for T in np.arange(MIN_T, MAX_T+DT_T, DT_T):
			fp = FrenetPath()
			lat_traj = QuinticPolynomial(di, di_d, di_dd, df, df_d, df_dd, T)

			fp.t = [t for t in np.arange(0.0, T, DT)] ## delta time
			fp.d = [lat_traj.calc_pos(t) for t in fp.t]
			fp.d_d = [lat_traj.calc_vel(t) for t in fp.t]
			fp.d_dd = [lat_traj.calc_acc(t) for t in fp.t]
			fp.d_ddd = [lat_traj.calc_jerk(t) for t in fp.t]

			# Longitudinal motion planning (velocity keeping)
			tfp = deepcopy(fp)
			lon_traj = QuarticPolynomial(si, si_d, si_dd, sf_d, sf_dd, T)

			tfp.s = [lon_traj.calc_pos(t) for t in fp.t]
			tfp.s_d = [lon_traj.calc_vel(t) for t in fp.t]
			tfp.s_dd = [lon_traj.calc_acc(t) for t in fp.t]
			tfp.s_ddd = [lon_traj.calc_jerk(t) for t in fp.t]

			# 경로 늘려주기 (In case T < MAX_T)
			for _t in np.arange(T, MAX_T, DT): ## delta time
				tfp.t.append(_t)
				tfp.d.append(tfp.d[-1])
				_s = tfp.s[-1] + tfp.s_d[-1] * DT ## delta time
				tfp.s.append(_s)

				tfp.s_d.append(tfp.s_d[-1])
				tfp.s_dd.append(tfp.s_dd[-1])
				tfp.s_ddd.append(tfp.s_ddd[-1])

				tfp.d_d.append(tfp.d_d[-1])
				tfp.d_dd.append(tfp.d_dd[-1])
				tfp.d_ddd.append(tfp.d_ddd[-1])

			J_lat = sum(np.power(tfp.d_ddd, 2))  # lateral jerk
			J_lon = sum(np.power(tfp.s_ddd, 2))  # longitudinal jerk

			# cost for consistency
			d_diff = (tfp.d[-1] - opt_d) ** 2
			# cost for target speed
			v_diff = (target_speed - tfp.s_d[-1]) ** 2
			#cost for global path tracking
			d_track = (tfp.d[-1]) ** 2 
			# print("cost for global path tracking",d_track)
			# lateral cost
			tfp.c_lat = K_J * J_lat + K_T * T + K_D * d_diff + K_GD * d_track 
			# logitudinal cost
			tfp.c_lon = K_J * J_lon + K_T * T + K_V * v_diff

			# total cost combined
			tfp.c_tot = K_LAT * tfp.c_lat + K_LON * tfp.c_lon

			frenet_paths.append(tfp)

	return frenet_paths

def calc_global_paths(fplist, mapx, mapy, maps,prev_wp):

	# transform trajectory from Frenet to Global
	for fp in fplist:
		for i in range(len(fp.s)):
			_s = fp.s[i]
			_d = fp.d[i]
			_x, _y, _ = get_cartesian(_s, _d, mapx, mapy, maps,prev_wp)
			fp.x.append(_x)
			fp.y.append(_y)

		for i in range(len(fp.x) - 1):
			dx = fp.x[i + 1] - fp.x[i]
			dy = fp.y[i + 1] - fp.y[i]
			fp.yaw.append(np.arctan2(dy, dx))
			fp.ds.append(np.hypot(dx, dy))

		fp.yaw.append(fp.yaw[-1])
		fp.ds.append(fp.ds[-1])

		# calc curvature
		for i in range(len(fp.yaw) - 1):
			yaw_diff = fp.yaw[i + 1] - fp.yaw[i]
			yaw_diff = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))
			fp.kappa.append(yaw_diff / fp.ds[i])

	return fplist


'''def collision_check(fp, obs_info, mapx, mapy, maps):
	
	# get obstacle's position (x,y)
	#obs_xy = get_cartesian( obs[i, 0], obs[i, 1], mapx, mapy, maps)
	car1s = [[f[0], f[1], f[2], 1.600, 1.160] for f in zip(fp.x, fp.y, fp.yaw)]
	
	for obs in obs_info:
		for car1 in car1s:
			car_vertices = get_vertice_rect(car1)
			obs_vertices = get_vertice_rect(obs)

			is_collide = separating_axis_theorem(car_vertices, obs_vertices)
			if is_collide:
				return True

	return False'''


def check_path(fplist, obs_info, mapx, mapy, maps):
	ok_ind = []
	vel = 0
	a = 0
	curv = 0
	col = 0
	
	for i, _path in enumerate(fplist):
		acc_squared = [(abs(a_s**2 + a_d**2)) for (a_s, a_d) in zip(_path.s_dd, _path.d_dd)]
		# print("kappa:" + str(fplist[i].kappa))
		if any([v > V_MAX for v in _path.s_d]):  # Max speed check
			vel += 1
			#print("a")
			continue
		elif any([acc > ACC_MAX**2 for acc in acc_squared]):
			a += 1
			#print("a:" + str(a))
			continue
		#if any([abs(kappa) > K_MAX for kappa in fplist[i].kappa]):  # Max curvature check
		#	#print("curv:"+str(abs(kappa)))
		#	curv += 1
		#	continue
		#elif collision_check(_path, obs_info, mapx, mapy, maps):
			#print("col")
		#	col += 1
		#	continue
		ok_ind.append(i)

	print("v = " + str(vel) + ", a = " + str(a) + ", curv = " + str(curv) + ", col = "+ str(col))
	print("total = " + str(len(fplist)) + ", selected = " + str(len(fplist) - curv - col - vel - a))
	return [fplist[i] for i in ok_ind], col
	# return fplist, col

def frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs_info, mapx, mapy, maps, opt_d, target_speed, prev_wp):
# def frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs_info, mapx, mapy, maps, opt_d, target_speed):
	fplist = calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d, target_speed, prev_wp)
	# fplist = calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d, target_speed)
	fplist = calc_global_paths(fplist, mapx, mapy, maps,prev_wp)
	col=0
	fplist, col = check_path(fplist, obs_info, mapx, mapy, maps)
	# find minimum cost path
	min_cost = float("inf")
	opt_traj = None
	opt_ind = 0
	for fp in fplist:
		if min_cost >= fp.c_tot:
			min_cost = fp.c_tot
			opt_traj = fp
			_opt_ind = opt_ind
		opt_ind += 1

	try:
		_opt_ind
	except NameError:
		print(" No solution ! ")
		_opt_ind = -1

	return fplist, _opt_ind
