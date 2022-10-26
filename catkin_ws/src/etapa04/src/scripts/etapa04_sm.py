#!/usr/bin/env python3

#Modulos generales
import rospy
import time
import os
import numpy as np
from math import sqrt, atan, inf
from utils_etapa04 import *
from etapa04.msg import signal, path
from tf.transformations import quaternion_from_euler as q2e
from tf.transformations import euler_from_quaternion as e2q
from scipy.io import savemat

import heapq
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Path
from nav_msgs.srv import *
from collections import deque
from MapSolver import *

#Initialization
rospy.init_node('etapa04_node')
pos = Posicion()
mov = Movimiento()
cntrl = MovController(pos, mov)
sm = StateMachine()

#dir 0 (field) - dir 1 (path)
path_pub = rospy.Publisher('/path_or_field', path, queue_size=10)

CC_id = 0
CC_state = False
S_id = 0
S_state = False

def close_collision_callback(msg):
	global CC_id, CC_state
	CC_id = msg.id
	CC_state = msg.indicator

def is_CC():
	global CC_id, CC_state
	
	#last = CC_id
	#while last + 3 > CC_id:
	#	None
	
	return CC_state

def stuck_callback(msg):
	global S_id, S_state
	S_id = msg.id
	S_state = msg.indicator

def is_stuck():
	global S_id, S_state
	
	last = S_id
	while last + 3 > S_id:
		None
	
	return S_state
	

def get_path_var(x, y, wz, option):
	path_v = path()
	path_v.pose.position.x = x
	path_v.pose.position.y = y
	path_v.pose.position.z = 0
	
	path_v.pose.orientation.x,path_v.pose.orientation.y, path_v.pose.orientation.z, path_v.pose.orientation.w = q2e(0,0,wz)
	
	path_v.dir = option
	return path_v

def save_map_data(mapa, name):
	res = mapa.info.resolution
	
	w = mapa.info.width
	h = mapa.info.height
	
	pos_x = mapa.info.origin.position.x
	pos_y = mapa.info.origin.position.y
	pos_z = mapa.info.origin.position.z
	pos_array = np.array([pos_x, pos_y, pos_z])
	
	ori_x = mapa.info.origin.orientation.x
	ori_y = mapa.info.origin.orientation.y
	ori_z = mapa.info.origin.orientation.z
	ori_w = mapa.info.origin.orientation.w
	
	euler_x, euler_y, euler_z = e2q([ori_x, ori_y, ori_z, ori_w])
	ori_array = np.array([euler_x, euler_y, euler_z])
	
	array = np.reshape(np.asarray(mapa.data), (h,w))
	
	dic = {'resolution' : res, 'width' : w, 'height' : h, 'position' : pos_array, "orientation" : ori_array, 'map' : array}
	
	savemat(name+'.mat', dic)
	
def run_sweep():
	print("Starting Move")
	ang_sweep(cntrl)
	print("Finished")

def planning_map():
	print("Waiting for maps")
	time.sleep(30)	
	
	print("Getting inflated map...")
	rospy.wait_for_service('/inflated_map')
	clt_inflated   = rospy.ServiceProxy("/inflated_map", GetMap)
	
	while True:
		try:
			inflated_map = clt_inflated().map
			print("Using inflated map with " +str(len(inflated_map.data)) + " cells.")
			break
		except:
			pass
	
	print("Saving Maps")
	save_map_data(inflated_map, "inflated")
	print("Map is saved")
	
	print("Solving for moving points")
	showTimeMark()
	mapa = obtain_data("inflated.mat")
	view_points = solve_map(mapa)
	print("View Points are found")
	showTimeMark()
	if 0 in view_points.shape:
		return False, get_path_var(0,0,0,True)
		
	init_coord = pos.get_coords()
	distance = np.sqrt(np.sum((view_points - init_coord) ** 2,axis=1))
	index = np.argmin(distance)
	target = view_points[index]
	print("View Points are found")
	return True, get_path_var(target[0],target[1],0,False)

#System Signalling
angular_flag = False
map_flag = False

target_point = get_path_var(0,0,0,True)
target_flag = True
reached_flag = False
ended_flag = False
initial_point = pos.get_coords()

# Angular Scan
def S1_exec():
	global angular_flag, reached_flag
	reached_flag = False
	
	if not angular_flag:
		run_sweep()
		angular_flag = True

def S1_end():
	global angular_flag
	
	if angular_flag:
		return 'S2'
	else:
		return 'S1'

# Planning view_point
def S2_exec():
	global angular_flag, map_flag, target_flag, target_point
	angular_flag = False
	
	if not map_flag:
		target_flag, target_point = planning_map()
		reached_flag = False
		rospy.set_param('/reached_flag', False)
		map_flag = True
	
def S2_end():
	global map_flag, target_flag
	
	if map_flag and not target_flag:
		return 'S5'
	elif map_flag:
		return 'S3'
	else:
		return 'S2'

# Potential Fields Movement
def S3_exec():
	global target_point, reached_flag, path_pub, map_flag
	map_flag = False
	rospy.set_param('/path_planning_go',False)
	rospy.set_param('/pot_fields_go',True)
	loop = rospy.Rate(10)
	
	reached_flag = rospy.get_param('/reached_flag')
	
	if not reached_flag:
		target_point.dir = False
		path_pub.publish(target_point)
		loop.sleep()

def S3_end():
	global reached_flag
	reached_flag = rospy.get_param('/reached_flag')
	print(reached_flag)
	if reached_flag:
		return 'S1'
	elif is_stuck():
		return 'S4'
	else:
		return 'S3'

# Path Planning Movement
def S4_exec():
	global target_point, reached_flag, path_pub
	rospy.set_param('/path_planning_go',True)
	rospy.set_param('/pot_fields_go',False)
	loop = rospy.Rate(10)
	
	reached_flag = rospy.get_param('/reached_flag')
	
	if not reached_flag:
		target_point.dir = True
		path_pub.publish(target_point)
		loop.sleep()

def S4_end():
	global reached_flag
	reached_flag = rospy.get_param('/reached_flag')
	if reached_flag:
		return 'S1'
	elif is_CC():
		return 'S3'
	else:
		return 'S4'

# Returning to Origin
def S5_exec():
	global input_point, reached_flag, path_pub
	path_var = get_path_var(initial_point[0],initial_point[1],0,True)
	loop = rospy.Rate(10)
	
	reached_flag = rospy.get_param('/reached_flag')
	
	if not reached_flag:
		path_pub.publish(path_var)
		loop.sleep()
	else:
		rospy.set_param('/map_request', True)

def S5_end():
	global ended_flag
	if rospy.get_param('/map_response',False):
		print("Process Ended")
		ended_flag = True
		print("Process Ended")
		sm.stop_machine()
		return 'S5'
	else:
		return 'S5'
	
sm.add_state("S1", S1_exec, S1_end) #Sweep Angular
sm.add_state("S2", S2_exec, S2_end) #Planeacion del punto de vista
sm.add_state("S3", S3_exec, S3_end) #Campos Potenciales
sm.add_state("S4", S4_exec, S4_end) #Planeacion de Ruta
sm.add_state("S5", S5_exec, S5_end) #Retorno al origen

def main():
	rospy.Subscriber('/close_collision_indicator', signal, close_collision_callback)
	rospy.Subscriber('/stuck_indicator', signal, stuck_callback)
	
	time.sleep(30)
	print("Process Started")
	showTimeMark()
	sm.start_machine('S1')
	
	while (not rospy.is_shutdown()) and (not ended_flag):
		None
	
	"""
	print("Starting Move")
	ang_sweep(cntrl)
	print("Finished")
	
	print("Waiting for maps")
	time.sleep(30)
	"""
	"""
	print("Getting inflated and cost maps...")
	rospy.wait_for_service('/static_map')
	rospy.wait_for_service('/cost_map')
	rospy.wait_for_service('/inflated_map')
	clt_static_map = rospy.ServiceProxy("/static_map"  , GetMap)
	clt_cost_map   = rospy.ServiceProxy("/cost_map"    , GetMap)
	clt_inflated   = rospy.ServiceProxy("/inflated_map", GetMap)
	
	while True:
		try:
			static_map = clt_static_map().map
			break
		except:
			print("Cannot get static map. Terminating program. ")
			pass
	
	while True:
		try:
			inflated_map = clt_inflated().map
			cost_map     = clt_cost_map().map
			print("Using inflated map with " +str(len(inflated_map.data)) + " cells.")
			print("Using cost map with "     +str(len(cost_map.data))     + " cells.")
			break
		except:
			pass
	
	print("Saving Maps")
	save_map_data(static_map, "static")
	save_map_data(inflated_map, "inflated")
	save_map_data(cost_map, "cost")
	print("Maps are saved")
	
	rospy.spin()
	"""
	
	"""
	rospy.Subscriber('/close_collision_indicator', signal, close_collision_callback)
	rospy.Subscriber('/stuck_indicator', signal, stuck_callback)
	
	#dir 0 (field) - dir 1 (path)
	pub = rospy.Publisher('/path_or_field', path, queue_size=10)
	
	loop = rospy.Rate(1)
	
	#Test
	path_var = get_path_var(2,-1,0.5,True)
	pub.publish(path_var)
	while not rospy.is_shutdown():
		pub.publish(path_var)
		#path_var.dir = not path_var.dir
		print("Last Stuck Data")
		print(S_id, S_state)
		print("Last CC Data")
		print(CC_id, CC_state)
		print()
		loop.sleep()
	"""
	
		
		

if __name__ == '__main__':
	try:
		main()
		if sm.operational:
			sm.stop_machine()
	except rospy.ROSInterruptException:
		if sm.operational:
			sm.stop_machine()
		pass
