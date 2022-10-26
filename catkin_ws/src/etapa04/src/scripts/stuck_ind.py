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
rospy.init_node('stuck_node')
pos = Posicion()
ind = signal()

time_window = 5 #seconds
variation_th = 0.2 #meters
sample_rate = 10 #hz

def main():
	loop = rospy.Rate(sample_rate)
	pub = rospy.Publisher('/stuck_indicator', signal, queue_size=10)
	
	t = 0
	ind.indicator = False
	ind.id = 0
	coord_1 = pos.get_coords()
	
	while not rospy.is_shutdown():
		pub.publish(ind)
		
		loop.sleep()
		t += 1/sample_rate
		
		if t >= time_window:
			coord = pos.get_coords()
			dist = np.linalg.norm(coord - coord_1)
			ind.indicator = dist < variation_th
			ind.id += 1
			
			t = 0
			coord_1 = coord

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
