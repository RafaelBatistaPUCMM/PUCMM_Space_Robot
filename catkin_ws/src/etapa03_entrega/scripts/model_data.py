#!/usr/bin/env python3

#Modulos generales
import rospy
import time
import numpy as np
from math import sqrt, atan, inf
from utils_etapa03 import *
import csv

#Initialization
rospy.init_node('model_data_node')
pos = Posicion()
mov = Movimiento()

#Experiment Parameters
period = 0.1 #seconds
values = [0, 0.3, 0.15, 0, -0.15, -0.3, 0, 0.15, 0.3, 0]
times = [3, 20, 40, 60, 80, 90, 110, 115, 125, 128]

#Input calculation
xt = []
t = 0
while t < times[len(times) - 1]:
	for i in range(len(times)):
		if t < times[i]:
			set_value = values[i]
			break
		else:
			set_value = 0
	xt.append(set_value)
	t = t + period

#Execution
yt = []
t = []
print("Execution Started")
last = rospy.Time.now().to_sec()

for x in xt:
	print('Commanding', x, 'm/s at ', end= "")
	mov.move_base(0,x,0)
	coords = pos.get_coords()
	angle = pos.get_current_angle()
	yt.append(coords[1])
	
	#Esperar periodo de muestreo
	now = rospy.Time.now().to_sec()
	t.append(now)
	print(now-t[0])
	while (now - last) < period:
		now = rospy.Time.now().to_sec()

	#Ultimo tiempo
	last = rospy.Time.now().to_sec()
	
mov.move_base_vel(0,0,0)
print("Execution Ended")
#Save Data
with open('y_move_data_2.csv', mode='w') as file:
	fieldnames = ['t','xt','yt']
	writer = csv.DictWriter(file, fieldnames=fieldnames)
	writer.writeheader()
	
	for i in range(len(t)):
		d = {'t':t[i], 'xt':xt[i], 'yt':yt[i]}
		writer.writerow(d)
print("Data has been saved")



