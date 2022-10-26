#!/usr/bin/env python3

#Modulos generales
import rospy
import time
import os
import numpy as np
from math import sqrt, atan, inf
from utils_etapa04 import *
from etapa04.msg import signal

#Initialization
rospy.init_node('close_collision_node')
laser = Laser()
ind = signal()

xy_th = 0.05 #m/s
ang_th = 0.05 #rad/s
sample_rate = 10 #hz

while not laser.meas_state:
	None
laser.set_obs_lim(0.5)

velocity_cond = False

def velocity_callback(msg):
	global velocity_cond
	
	vel_x = msg.linear.x
	vel_y = msg.linear.y
	vel_z = msg.angular.z
	
	vel_xy = sqrt(vel_x**2 + vel_y**2)
	
	velocity_cond = vel_xy > xy_th and vel_z < ang_th
	

def main():
	loop = rospy.Rate(sample_rate)
	pub = rospy.Publisher('/close_collision_indicator', signal, queue_size=10)
	sub = rospy.Subscriber('/hsrb/command_velocity', Twist, velocity_callback)
	
	t = 0
	ind.indicator = False
	ind.id = 0
	
	while not rospy.is_shutdown():
		pub.publish(ind)
		
		loop.sleep()
		
		obs = laser.get_obs_params(0, 0.47)
		
		ind.id = t
		cond = velocity_cond and obs.criteria
		
		if cond != ind.indicator:
			t += 1
		ind.indicator = cond
		

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass



