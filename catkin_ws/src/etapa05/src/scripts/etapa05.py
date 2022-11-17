#!/usr/bin/env python3

#Modulos generales
import rospy
import time
import os
import numpy as np
from math import sqrt, atan, inf
from utils_etapa05 import *
import cv2 as cv

#Initialization
rospy.init_node('etapa05_node')

time.sleep(20)
#Delay de 5 Segundos
now = rospy.Time.now().to_sec()
last = rospy.Time.now().to_sec()
while now - last < 5:
	now = rospy.Time.now().to_sec()

pos = Posicion()
mov = Movimiento()
cntrl = MovController(pos, mov)
laser = Laser()
pp = PickPlace(cntrl)
rgbd = RGBD()

target_coords = np.array([[0.0,1.21],[-3.0,4.0],[3.9,5.6]])
moving_delay = 10

start_time = time.strftime("%d%m%y_%H%M")
folder = "Log_"+start_time
try:
	os.mkdir(folder)
except:
	None
	
path = folder+'/'+"Objects.txt"

def main():
	backward = []
	
	#Movimiento del brazo
	pp.grip([1,0,0.3])
	pp.go_default()
	
	#Back to O
	cntrl.start_control(2,[0,0])
	print("Moving to origin")
	while cntrl.is_controlling():
		None
		
	point = target_coords[0]
	target = get_gaze_target(pos, point, 2.5)
	backward.append(target)
	cntrl.start_control(2,target)
	print("Moving to first target")
	while cntrl.is_controlling():
		None
		
	pos.final = point
	angle = pos.find_angle()
	cntrl.start_control(1,angle)
	print("Correcting angle to first target")
	while cntrl.is_controlling():
		None
	
	#Moving Delay
	time.sleep(moving_delay)
	
	rgbd.set_measure()
	while not rgbd.get_measure():
		None
	
	rgb_img = rgbd.get_image()#Image
	dist_img = rgbd.get_dist_image() #Image
	(maxi, data) = rgbd.get_dist_data() #Float + Array
	cloud = rgbd.get_cloud() #Array
	rgbd.tf_depth.get_transform()
	tfs = rgbd.tf_depth._global_trans
	
	print("Calculating Objects")
	cents = object_segmentation(dist_img, rgb_img, maxi, cloud)
	print(cents)
	coords = []
	for i in range(len(cents)):
		(t, r) = rgbd.tf_depth.send_trans_by_obj(cents[i], (0,0,0,1), f'Objeto{i+1}_Zona01',tfs)
		coords.append(t)
	
	print("Objects Found")
	print(np.array(coords))
	
	save_rgbd_data(rgb_img, dist_img, maxi, data, cloud, 1, folder)
	
	
	#Logging data
	trans_list = rgbd.tf_depth._trans_list
	quantity = len(trans_list)
	for i in range(quantity):
		tfs = trans_list[i]
		coord = [tfs.transform.translation.x,tfs.transform.translation.y,tfs.transform.translation.z]
		save_coords(path,'A',i+1,coord)

	
	pos.final = np.array([1,0])
	angle = pos.find_angle()
	cntrl.start_control(1,angle)
	print("Correcting angle to first sub-target")
	while cntrl.is_controlling():
		None
	
	print("Moving to first sub-target")
	backward.append([1,0])
	cntrl.start_control(2,[1,0])
	while cntrl.is_controlling():
		None
	
	pos.final = np.array([1,2])
	angle = pos.find_angle()
	cntrl.start_control(1,angle)
	print("Correcting angle to second sub-target")
	while cntrl.is_controlling():
		None
	
	print("Moving to second sub-target")
	backward.append([1,2])
	cntrl.start_control(2,[1,2])
	while cntrl.is_controlling():
		None
	
	point = target_coords[1]
	
	print("Correcting angle to second target")
	pos.final = point
	angle = pos.find_angle()
	cntrl.start_control(1,angle)
	while cntrl.is_controlling():
		None
	
	print("Moving to second target")
	target = get_gaze_target(pos, point, 2.5)
	backward.append(target)
	cntrl.start_control(2,target)
	while cntrl.is_controlling():
		None
	
	print("Correcting angle to second target")
	pos.final = point
	angle = pos.find_angle()
	cntrl.start_control(1,angle)
	while cntrl.is_controlling():
		None
	
	#Moving Delay
	time.sleep(moving_delay)
	
	rgbd.set_measure()
	while not rgbd.get_measure():
		None
	
	rgb_img = rgbd.get_image()#Image
	dist_img = rgbd.get_dist_image() #Image
	(maxi, data) = rgbd.get_dist_data() #Float + Array
	cloud = rgbd.get_cloud() #Array
	rgbd.tf_depth.get_transform()
	tfs = rgbd.tf_depth._global_trans
	
	print("Calculating Objects")
	cents = object_segmentation(dist_img, rgb_img, maxi, cloud)
	print(cents)
	coords = []
	for i in range(len(cents)):
		(t, r) = rgbd.tf_depth.send_trans_by_obj(cents[i], (0,0,0,1), f'Objeto{i+1}_Zona02', tfs)
		coords.append(t)
	
	print("Objects Found")
	print(np.array(coords))
	
	save_rgbd_data(rgb_img, dist_img, maxi, data, cloud, 2, folder)
	
	#Logging data
	trans_list = rgbd.tf_depth._trans_list
	quantity2 = len(trans_list) - quantity 
	for i in range(quantity2):
		tfs = trans_list[i+quantity]
		coord = [tfs.transform.translation.x,tfs.transform.translation.y,tfs.transform.translation.z]
		save_coords(path,'B',i+1,coord)
	
	#Going backward
	for i in range(1,len(backward)):
		target = np.array(backward[len(backward) - i])
		print("Correcting angle to backward target")
		pos.final = target
		angle = pos.find_angle()
		cntrl.start_control(1,angle)
		while cntrl.is_controlling():
			None

		print("Moving to backward target")
		cntrl.start_control(2,target)
		while cntrl.is_controlling():
			None
	
	target = np.array([0,0])
	print("Correcting angle to origin target")
	pos.final = target
	angle = pos.find_angle()
	cntrl.start_control(1,angle)
	while cntrl.is_controlling():
		None

	print("Moving to origin target")
	cntrl.start_control(2,target)
	while cntrl.is_controlling():
		None
if __name__ == '__main__':
	main()
	
	
	
