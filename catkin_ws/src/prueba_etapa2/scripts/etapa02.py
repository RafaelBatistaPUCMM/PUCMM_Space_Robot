#!/usr/bin/env python3

#Modulos generales
import rospy
import time
import numpy as np
from math import sqrt, atan, inf
from utils_etapa02 import *

#Initialization
reached = False
vel_vector = np.array([0, 0])
rospy.init_node('etapa02_node')
pos = Posicion('/meta_competencia')
mov = Movimiento()
cntrl = MovController(pos, mov)
laser = Laser()
sm = StateMachine()

#Get Target
time.sleep(5)
angle = pos.find_angle()
time.sleep(5)
angle = pos.find_angle()

#State Machine Functions
def S1_exec():
	angle = pos.find_angle()
	
	if cntrl.ControlMode != 1:
		mov.move_base(0,0,0)
		cntrl.output = None
		cntrl.start_control(1, angle)
		time.sleep(0.5)
	
	cntrl.change_SP(angle)
	time.sleep(0.5)
	
def S1_end():
	if cntrl.ControlMode == 1 and cntrl.output == 0:
		return "S2"
	return "S1"
	
def S2_exec():
	if cntrl.ControlMode != 0:
		cntrl.ControlMode = 0
	mov.move_base(0.3,0,0)
	time.sleep(1)

def S2_end():
	pos.find_angle()
	obs = laser.get_obs_params(0, 0.47)
	
	coord_1 = pos.actual
	coord_2 = pos.final
	distance = sqrt(sum((coord_2 - coord_1) ** 2))
    
	angle_dif = pos.target_angle - pos.current_angle
	if angle_dif > np.pi:
		angle_dif -= 2*np.pi
	elif angle_dif < -np.pi:
		angle_dif += 2*np.pi
		
	if abs(angle_dif) > 0.5 and distance > sqrt(2):
		return "S1"
	elif obs.criteria:
		return "S4"
	elif distance <= sqrt(2):
		return "S3"
	return "S2"
	
def S3_exec():
	if cntrl.ControlMode != 2:
		mov.move_base(0,0,0)
		cntrl.start_control(2, pos.final)

def S3_end():
	if cntrl.output[0] != 0:
		angle = atan(cntrl.output[1] / cntrl.output[0])
		if cntrl.output[0] < 0 and cntrl.output[1] < 0:
			angle = angle - np.pi
		elif cntrl.output[0] < 0 and cntrl.output[1] > 0:
			angle = angle + np.pi
	elif cntrl.output[1] > 0:
		angle = np.pi/2
	else:
		angle = -np.pi/2
		
	if abs(angle) < 2.1:
		obs = laser.get_obs_params(angle, 0.47)
	else:
		obs = Obstacle()
	if cntrl.ControlMode == 2 and cntrl.output[0] == 0 and cntrl.output[1] == 0:
		return "S6"
	elif obs.criteria:
		return "S4"
	return "S3"

def S4_exec():
	global vel_vector
	
	if sm.previous_state != "S4":
		vel_vector = np.array([0, 0])
		
	if sm.previous_state == "S3":
		cntrl.stop_control()
		while cntrl.ControlMode != 0:
			None
		pos.find_angle()
		cntrl.start_control(1, pos.target_angle)
		while not (cntrl.ControlMode == 1 and cntrl.output == 0):
			None
		mov.move_base(0,0,0)
		cntrl.ControlMode = 0
		sm.previous_state = "S4"
	
	if sm.previous_state == "S2":
		mov.move_base(0,0,0)
		sm.previous_state = "S4"
		
	
	obs = laser.get_obs_params(0, 0.47)
	if obs.criteria:
		pos.find_angle()
		new_angle_1 = pos.current_angle - obs.angle_shift
		new_angle_2 = pos.current_angle - np.pi - obs.angle_shift #Complementario
		
		if new_angle_1 > np.pi:
			new_angle_1 = new_angle_1 - 2*np.pi
		elif new_angle_1 < -np.pi:
			new_angle_1 = new_angle_1 + 2*np.pi
			
		if new_angle_2 > np.pi:
			new_angle_2 = new_angle_2 - 2*np.pi
		elif new_angle_2 < -np.pi:
			new_angle_2 = new_angle_2 + 2*np.pi
			
		dif_angle_1 = abs(new_angle_1 - pos.target_angle)
		dif_angle_2 = abs(new_angle_2 - pos.target_angle)
		
		if dif_angle_1 < dif_angle_2:
			new_angle = new_angle_1
		else:
			new_angle = new_angle_2
		
		#Obstacle velocity 0.1 m/s
		vel_vector = 0.1 * np.array([np.cos(new_angle), np.sin(new_angle)])
		
		theta = pos.current_angle
		T_matrix = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
		vel_vector = np.dot(vel_vector, T_matrix)
		
		mov.move_base(vel_vector[0], vel_vector[1], 0)

		time.sleep(2)
	else:
		mov.move_base(vel_vector[0], vel_vector[1], 0)
		time.sleep(2)

once = True
def S4_end():
	pos.find_angle()
	dif = pos.target_angle - pos.current_angle
	if dif > np.pi:
		dif = dif - 2*np.pi
	elif dif < -np.pi:
		dif = dif + 2*np.pi
		
	obs1 = laser.get_obs_params(dif, 0.47)
	vel_vector = np.array([mov.obj.linear.x, mov.obj.linear.y])
	
	if vel_vector[0] != 0:
		angle = atan(vel_vector[1]/vel_vector[0])
		if vel_vector[0] < 0 and vel_vector[1] < 0:
			angle = angle - np.pi
		elif vel_vector[0] < 0 and vel_vector[1] > 0:
			angle = angle + np.pi
	elif vel_vector[1] > 0:
		angle = np.pi/2
	else:
		angle = -np.pi/2
	
	angle -= pos.current_angle
	if angle > np.pi:
		angle -= 2*np.pi
	elif dif < -np.pi:
		angle += 2*np.pi
	
	if abs(angle) < 2.1 and (vel_vector[0] != 0 or vel_vector[1] != 0):	
		obs2 = laser.get_obs_params(angle, 0.47)
	else:
		obs2 = Obstacle()
	
	"""
	global once
	if once and obs2.criteria:
		print("Attempted to go S5")
		print("Looking for obs at: ", angle, " rad")
		print("Moving at: ", vel_vector, "m/s")
		print("Looking at: ", pos.current_angle, "rad")
		print("OBS DETAILS")
		print(obs2.distance, obs2.angle, obs2.angle_shift)
		once = False
	
	if obs2.criteria:
		#return "S5"
		None #Test No S5
	"""
	if not obs1.criteria:
		return "S1"
	return "S4"
	
def S5_exec():
	global vel_vector
	obs = laser.get_obs_params(0, 0.47)
	if obs.criteria:
		pos.find_angle()
		new_angle_1 = pos.current_angle - obs.angle_shift
		new_angle_2 = pos.current_angle - np.pi - obs.angle_shift #Complementario
		
		if new_angle_1 > np.pi:
			new_angle_1 = new_angle_1 - 2*np.pi
		elif new_angle_1 < -np.pi:
			new_angle_1 = new_angle_1 + 2*np.pi
			
		if new_angle_2 > np.pi:
			new_angle_2 = new_angle_2 - 2*np.pi
		elif new_angle_2 < -np.pi:
			new_angle_2 = new_angle_2 + 2*np.pi
			
		dif_angle_1 = abs(new_angle_1 - pos.target_angle)
		dif_angle_2 = abs(new_angle_2 - pos.target_angle)
		
		if dif_angle_1 < dif_angle_2:
			new_angle = new_angle_1
		else:
			new_angle = new_angle_2
		
		#Obstacle velocity 0.1 m/s
		vel_vector = 0.1 * np.array([np.cos(new_angle), np.sin(new_angle)])
		
		theta = pos.current_angle
		T_matrix = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
		vel_vector = np.dot(vel_vector, T_matrix)
		
		mov.move_base(vel_vector[0], vel_vector[1], 0)

		time.sleep(2)
	else:
		mov.move_base(vel_vector[0], vel_vector[1], 0)
		time.sleep(2)

def S5_end():
	pos.find_angle()
	dif = pos.target_angle - pos.current_angle
	if dif > np.pi:
		dif = dif - 2*np.pi
	elif dif < -np.pi:
		dif = dif + 2*np.pi
	
	obs = laser.get_obs_params(dif, 0.47)
	
	if not obs.criteria:
		return "S1"
	return "S5"
	
def S6_exec():
	print("Target Acquired")
	global reached
	reached = True
	time.sleep(1)

def S6_end():
	return "S6"
	
#Creating State Machine
sm.add_state("S1", S1_exec, S1_end) #Control Angular
sm.add_state("S2", S2_exec, S2_end) #Movimiento en X
sm.add_state("S3", S3_exec, S3_end) #Control de Coordenadas
sm.add_state("S4", S4_exec, S4_end) #Manejo de obstaculos 1
sm.add_state("S5", S5_exec, S5_end) #Manejo de obstaculos 2
sm.add_state("S6", S6_exec, S6_end) #End
sm.start_machine("S1")
       
def main():
	rate = rospy.Rate(2)
	while (not rospy.is_shutdown()) and (not reached):
		try:
			angle = pos.find_angle()
			print("Target Angle (rad): ",end="")
			print(angle)
			print("Target Pos: ",end="")
			print(pos.final)
			print("Pos Actual: ",end="")
			print(pos.actual)
			print("Ang Actual: ",end="")
			print(pos.current_angle)
			print("Ang Vel Actual: ",end="")
			print(mov.obj.angular.z)
			print("Lin X Vel Actual: ",end="")
			print(mov.obj.linear.x)
			print("Lin Y Vel Actual: ",end="")
			print(mov.obj.linear.y)
			print("")
			rate.sleep()
		except:
			sm.stop_machine()
			break
	if sm.operational:
		sm.stop_machine()
    		   
if __name__ == '__main__':
    main()
