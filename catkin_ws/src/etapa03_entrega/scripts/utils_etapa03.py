#!/usr/bin/env python3

#Modulos generales
import rospy
import numpy as np
import ros_numpy
import tf2_ros as tf
import tf2_geometry_msgs as tfg
import pandas as pd

#Modulos de los topicos
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, TransformStamped, Point
from sensor_msgs.msg   import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker , MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import String

#Modulos de calculo
from tf.transformations import euler_from_quaternion
from math import atan, sqrt, inf

#Modulos especificos
import threading
import time
import os
import itertools
import actionlib
import subprocess
from IPython.display import Image
import cv2 as cv
import sys
from sklearn.decomposition import PCA

class TF:
	"""
	"""
	def __init__(self, ref='map', obj='base_link'):
		self.ref = ref
		self.obj = obj
		
		self._tfBuffer = tf.Buffer()
		self._listener = tf.TransformListener(self._tfBuffer)
		self._broadcaster = tf.TransformBroadcaster()
		self._global_trans = None
		self._trans_list = []
		
		self._update_task = threading.Thread(target=self.repeat_transform)
		self._alive_task = False
		
	def set_reference(self, ref):
		self.ref = ref
	
	def set_obj(self, obj):
		self.obj = obj
	
	def get_transform(self):
		while True:
			try:
				self._global_trans = self._tfBuffer.lookup_transform(self.ref, self.obj, rospy.Time())
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
			break
			
		return (self._global_trans.transform.translation, self._global_trans.transform.rotation)
		
			
	def get_transform_array(self):
		trans, rot = get_transform()
		trans_array = np.array([trans.x, trans.y, trans.z])
		q_array = np.array([rot.x, rot.y, rot.z, rot.w])
		rot_array = euler_from_quaternion(q_array)
		return (trans_array, rot_array)
		
	def send_transform(self, transl, rot, frame, ref=None):
		if self._update_task.is_alive():
			self._alive_task = False
			while self._update_task.is_alive():
				None
		
		if ref == None:
			ref = self.ref
		tfs = TransformStamped()
		tfs.header.stamp = rospy.Time.now()
		tfs.header.frame_id = ref
		tfs.child_frame_id = frame
		tfs.transform.translation.x = transl[0]
		tfs.transform.translation.y = transl[1]
		tfs.transform.translation.z = transl[2]
		tfs.transform.rotation.x = rot[0]
		tfs.transform.rotation.y = rot[1]
		tfs.transform.rotation.z = rot[2]
		tfs.transform.rotation.w = rot[3]
		self._broadcaster.sendTransform(tfs)
		trans = None
		while trans != tfs:
			tfs.header.stamp = rospy.Time.now()
			self._broadcaster.sendTransform(tfs)
			try:
				trans = self._tfBuffer.lookup_transform(ref, frame, rospy.Time())
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

		self._trans_list.append(tfs)
		
		self._update_task = threading.Thread(target=self.repeat_transform)
		self._alive_task = True
		self._update_task.start()
		
	
	def send_trans_by_ref(self, transl, rot, frame):
		self.send_transform(transl, rot, frame)
	
	def send_trans_by_obj(self, transl, rot, frame, ref_trans=None):
		original_pose = PoseStamped()
		original_pose.pose.position.x = transl[0]
		original_pose.pose.position.y = transl[1]
		original_pose.pose.position.z = transl[2]
		original_pose.pose.orientation.x = rot[0]
		original_pose.pose.orientation.y = rot[1]
		original_pose.pose.orientation.z = rot[2]
		original_pose.pose.orientation.w = rot[3]
		original_pose.header.frame_id = self.obj
		original_pose.header.stamp = rospy.Time.now()
		
		if ref_trans == None:
			self.get_transform()
			ref_trans = self._global_trans
			
		new_pose = tfg.do_transform_pose(original_pose, ref_trans)
		
		transl = (new_pose.pose.position.x, new_pose.pose.position.y, new_pose.pose.position.z)
		rot = (new_pose.pose.orientation.x, new_pose.pose.orientation.y, new_pose.pose.orientation.z, new_pose.pose.orientation.w)
		self.send_transform(transl, rot, frame)
		return (transl, rot)
	
	def repeat_transform(self):
		while self._alive_task:
			for i in range(len(self._trans_list)):
				tfs = self._trans_list[i]
				trans = None
				while trans != tfs:
					tfs.header.stamp = rospy.Time.now()
					self._broadcaster.sendTransform(tfs)
					try:
						trans = self._tfBuffer.lookup_transform(tfs.header.frame_id, tfs.child_frame_id, rospy.Time())
					except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
						continue
			
			#Delay de 3 Segundos
			now = rospy.Time.now().to_sec()
			last = rospy.Time.now().to_sec()
			while now - last < 3:
				now = rospy.Time.now().to_sec()
				
	


class Posicion:
    """
    La clase Posicion se encarga de:
     1. Leer la posicion meta,
     2. leer la posicion actual,
     3. calcular el angulo de giro para el movimiento
     
    La informacion de la meta se actualiza con el nodo publicador
    
    La informacion de la posicion actual se obtiene con el metodo get_position o
    se puede actualizar automaticamente con la metodo start_timer
    
    El angulo se obtiene con la metodo find_angle
    """
    def __init__(self, nodoMeta = None):
        if nodoMeta != None:
            #Subscripcion al nodo que publica la meta
            goal_node = rospy.Subscriber(nodoMeta, PoseStamped, self.read_goal)
        
        #Mediciones periodicas
        self.delta = 1 #Periodo
        self.UpdateThread = threading.Thread(target=self.Timer)	#Tarea
        self.task_state = False
        
        #Variables para la obtencion de coordenadas
        self.tfBuffer = tf.Buffer()
        self.listener = tf.TransformListener(self.tfBuffer)
        
        
        #Posiciones
        self.inicial = np.zeros([1,2])[0]
        self.actual = np.zeros([1,2])[0]
        self.final = np.zeros([1,2])[0]
        
        #Angulos
        self.current_angle = 0
        self.target_angle = 0
        self.final_angle = 0
    
    def read_goal(self, msg):
        #Posicion objetivo
        self.final = np.array([msg.pose.position.x, msg.pose.position.y])
        
        #Orientacion objetivo
        orientation = msg.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        self.final_angle = yaw
     
    def leer_objetivo(self):
        return self.final
        
    def get_coords(self):
        while True:
            try:
                #Transformada
                trans = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            except:
                continue
            
            #Posicion actual
            self.actual = np.array([trans.transform.translation.x,trans.transform.translation.y])
            
            #Angulo actual
            rot = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
            (roll, pitch, yaw) = euler_from_quaternion(rot)
            self.current_angle = yaw
            
            return self.actual
    
    def get_current_angle(self):
    	self.get_coords()
    	return self.current_angle
    
    #Tarea de mediciones periodicas     
    def Timer(self):
        while self.task_state:
            self.get_coords()
            time.sleep(self.delta)
            
    
    #Iniciar mediciones periodicas
    def start_timer(self, delta):
        self.task_state = True
        if not self.UpdateThread.is_alive():
            self.delta = delta
            self.UpdateThread = threading.Thread(target=self.Timer)
            self.UpdateThread.start()
    
    #Parar mediciones periodicas
    def stop_timer(self):
    	self.task_state = False
    	while self.UpdateThread.is_alive():
    		None
    
    #Calcular el angulo de destino
    def find_angle(self):
    	#Actualizar posicion actual
        self.get_coords()
        
        #Calcular el vector de desplazamiento
        vector = self.final - self.actual
        
        if vector[0] != 0:								#Si el angulo del vector es diferente de +/- 90 grados
            vector_angle = atan(vector[1] / vector[0]) 		#El angulo se calcula con la tangente inversa
            if vector[0] < 0 and vector[1] < 0: 			#Si el angulo esta en el cuadrante III
                vector_angle = vector_angle - np.pi				#El angulo resultante es el calculado menos pi
            elif vector[0] < 0 and vector[1] > 0:			#Si el angulo esta en el cuadrante II
                vector_angle = vector_angle + np.pi				#El angulo resultante es el calculado mas pi
        elif vector[1] > 0:								#Si el angulo resultante es +90 grados
        	vector_angle = np.pi/2							#El angulo resultante es +90 grados
        else:											#Si el angulo resultante es +90 grados
            vector_angle = -np.pi/2							#El angulo resultante es -90 grados
        
        self.target_angle = vector_angle
        return self.target_angle

class Movimiento:
	"""
	La clase Movimiento se encarga de inicializar el objeto Twist
	y posee metodos para comandar velocidades a la base:
	  1. move_base_vel permite comandar un conjunto de velocidades de forma instantanea
	  2. move_base permite comandar velocidades durante un tiempo definido (timeout)
	"""
	def __init__(self):
		self.pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10) #Publicador
		self.obj = Twist()														   #Objeto Twist
		
		self.obj.linear.x = 0		#en m/s
		self.obj.linear.y = 0 		#en m/s
		self.obj.angular.z = 0		#en rad/s
		
		#Para comandar velocidades durante un tiempo
		self.moveThread = threading.Thread(target=self._move_task)	#Tarea
		self.task_state = False
		self.auto_vx = 0 #velocidad lineal en x (m/s)
		self.auto_vy = 0 #velocidad lineal en y (m/s)
		self.auto_vz = 0 #velocidad angular en z (rad/s)
		self.timeout = 5
		
	def move_base_vel(self, vx, vy, vw):
		self.obj.linear.x = vx 		#en m/s
		self.obj.linear.y = vy 		#en m/s
		self.obj.angular.z = vw		#en rad/s
		self.pub.publish(self.obj)  #publicando velocidades (objeto twist)
		
		
	def move_base(self,vx,vy,vz,timeout=5):
		if self.moveThread.is_alive():	#Si la tarea esta activa
			self.task_state = False		#Detenerla
			while self.moveThread.is_alive():
				None
		
		#Actualizar velocidades y timeout
		self.auto_vx = vx				
		self.auto_vy = vy
		self.auto_vz = vz
		self.timeout = timeout
		
		#Iniciar tarea
		self.task_state = True
		self.moveThread = threading.Thread(target=self._move_task)	#Tarea
		self.moveThread.start()
	
	def _move_task(self):
		start_time = rospy.Time.now().to_sec() #Tiempo de inicio
		while rospy.Time.now().to_sec() - start_time < self.timeout:   #Mientras no haya pasado timeout
			self.move_base_vel(self.auto_vx, self.auto_vy, self.auto_vz) #Comandar velocidades
			if not self.task_state:
				break
		
class Obstacle:
	"""
	La clase Obstacle contiene los parametros de un obstaculo:
		1. criteria: criterio de si hay o no colision
		2. index: posicion en el arreglo de la lectura en el que ocurre la colision
		3. distance: distancia minima de colision
		4. angle: angulo en el que se observa la colision mas cercana
		5. angle_shift: angulo en el que se encuentra el obstaculo respecto al robot
	"""
	def __init__(self):
		self.criteria = False
		self.index = None
		self.distance = inf
		self.angle = None
		self.angle_shift = None
		self.is_set = False
	
	def set(self, criteria, index, distance, angle, angle_shift):
		self.criteria = criteria
		self.index = index
		self.distance = distance
		self.angle = angle
		self.angle_shift = angle_shift
		self.is_set = True
	
	def clear(self):
		self.is_set = False
	
class Laser:
	"""
	La clase Laser suscribe el nodo al topico del LaserScan y recolecta la informacion
	
	La informacion se preprocesa obteniendo rangos diferentes de infinito y un arreglo
	de los angulos
	
	La clase cuenta con los siguientes metodos:
		1. set_obs_lim --> permite fijar la distancia en la que se detectan obstaculos
		2. _find_angle --> identifica la posicion en el arreglo de angulos en la que se encuentra
						   el angulo mas cercano al valor dado
		3. get_obs_params --> retorna un objeto Obstacle con los parametros de obstaculo
	"""
	def __init__(self, ref_space = 0.47):
		self.sub = rospy.Subscriber('/hsrb/base_scan', LaserScan, self._laser_cb) #Subscriptor
		
		#Informacion del mensaje
		self.angle_min = None
		self.angle_max = None
		self.angle_increment = None
		self.range_min = None
		self.range_max = None
		self.ranges = None
		
		#Estado de la medicion
		self.meas_state = False
		
		#Variables de preprocesamiento
		self.lectura = None
		self.angles = None
		
		#Limite de obstaculo
		self.obs_lim = 0.5
		
		#Obstacle watchout
		self.ObstacleThread = threading.Thread(target=self._obstacle_task)	#Tarea
		self.WatchoutState = False 	
		self.pos = None
		self.ref_space = ref_space
		self.obs_flag = False
		self.obs_obj = None
		
	def _laser_cb(self, msg):
		#Extraer la data
		self.angle_min = msg.angle_min
		self.angle_max = msg.angle_max
		self.angle_increment = msg.angle_increment
		self.range_min = msg.range_min
		self.range_max = msg.range_max
		self.ranges = msg.ranges
		
		#Elimininar infinitos truncando al valor maximo
		self.lectura = np.asarray(self.ranges)
		self.lectura = np.where(self.lectura > self.range_max, self.range_max,self.lectura)
		
		#Crear arreglo de angulos
		self.angles = np.zeros(len(self.lectura))
		angle_0 = self.angle_min
		for i in range(0,len(self.angles)):
			self.angles[i] = angle_0
			angle_0 = angle_0 + self.angle_increment
		
		#Estado de medicion activado
		self.meas_state = True
		
	def set_obs_lim(self, lim):
		if lim > self.range_max:			#Si el limite supera el rango maximo
			self.obs_lim = self.range_max		#Truncar al maximo (No hay obstaculos)
		elif lim < self.range_min:			#Si el limite es inferior al rango minimo
			self.obs_lim = self.range_min		#Truncar al minimo (Todo es obstaculo)
		else:
			self.obs_lim = lim
	
	def _find_angle(self, mov_angle):
		#Buscar el indice del angulo cuya diferencia sea minima
		dif = abs(self.angles - mov_angle) 
		index = np.argmin(dif)
		return index
		
	def get_obs_params(self, ref_angle, ref_space):
		angle_index = self._find_angle(ref_angle) #Indice el angulo del vector de movimiento
		angles_tf = self.angles - self.angles[angle_index] #Transformar angulos respecto a la referencia
		d = self.lectura * np.cos(angles_tf) #Componente vertical de la distancia
		c = abs(self.lectura * np.sin(angles_tf)) #Componente horizontal de la distancia	
		d = np.where(c <= ref_space/2, d, inf) #Eliminando (inf) los valores que no estan dentro de la zona de choque
		
		
		obs = Obstacle() #Declarar objecto Obstaculo
		criteria = min(d) < self.obs_lim #Hay colision?
		
		if criteria: #Si es asi
			index = np.argmin(d)		#Indice de la colision mas cercana
			distance = d[index]			#Distancia de la colision mas cercana
			angle = self.angles[index]	#Angulo de la colision mas cercana
			
			#Buscar los limites del obstaculo
			obs_border_min = index #Indice del Borde inferior
			while obs_border_min >= 0 and d[obs_border_min] != inf:
				obs_border_min = obs_border_min - 1
			
			obs_border_max = index #Indice del Borde superior
			while obs_border_max < len(d) and d[obs_border_max] != inf:
				obs_border_max = obs_border_max + 1
			
			#Distancias de los bordes
			obs_border_dmin = d[obs_border_min + 1]
			obs_border_dmax = d[obs_border_max - 1]
			shift = obs_border_dmax - obs_border_dmin #ancho del obstaculo
			
			#Evitar division en infinito
			if shift != 0:
				angle_shift = -atan(ref_space/shift)
			else:
				angle_shift = -np.pi/2 #90 grados
			
			#Dar valores al objeto
			obs.set(criteria, index, distance, angle, angle_shift)
		else:
			obs.clear()
		
		"""
		Nota:
			El angle_shift es una referencia de direccion que puede colapsar en un sentido 
			o el otro en la misma direccion. Esta a decision del control de obstaculos
			a que sentido tomara el angulo
		"""
		return obs
	
	def _obstacle_task(self):
		while self.WatchoutState:
			self.pos.get_coords()
			self.obs_obj = self.get_obs_params(self.pos.current_angle, self.ref_space)
			self.obs_flag = sef.obs_obj.criteria
			time.sleep(0.01)
				
	def start_obs_task(self, pos):
		if self.ObstacleThread.is_alive():
			self.WatchoutState = False
			while self.ObstacleThread.is_alive():
				None
		
		self.pos = pos		
		self.WatchoutState = True
		self.ObstacleThread = threading.Thread(target=self._obstacle_task)
		self.ObstacleTread.start()
	
	def stop_obs_task(self):
		if self.ObstacleThread.is_alive():
			self.WatchoutState = False
			while self.ObstacleThread.is_alive():
				None
		else:
			self.WatchoutState = False
		self.obs_flag = False
		
class RGBD:
	"""
	La clase RGBD se subscribe al topico de nube de puntos y exrae la informacion:
		Imagen RGB
		Nube de puntos XYZ
		Imagen de distancia
	
	Tambien cuenta con la capacidad de filtrar imagenes en funcion a una transformacion
	y a un criterio de seleccion sobre la imagen, creando una mascara
	
	Tambien cuenta con una funcion de movimiento de la mirada (gaze_point)
	"""
	def __init__(self):
		self._points_data = None
		self._image_data = None
		self._pos_data = None
		self._h_image = None
		self._region = None
		self._dist_data = None
		self._trans_cb = None
		self._crit_cb = None
		self._region_detect = False
		self._max_range = None
		self._first_measure = False

		self.tf_obj = TF(ref='map', obj='head_rgbd_sensor_gazebo_frame')
		#self.tf_depth = TF(ref='map', obj='head_rgbd_sensor_depth_frame')
		self.tf_depth = TF(ref='map', obj='head_rgbd_sensor_link')
		
		#Position Initialization
		self._head = moveit_commander.MoveGroupCommander('head',wait_for_servers=10)
		arm =  moveit_commander.MoveGroupCommander('arm',wait_for_servers=10)

		#Arm movement
		print("Moving Arm to Go Position...")
		arm.set_named_target('go')
		arm.go()
		print("Go Position Reached")

		#Head initial position
		print("Moving Head To Init Position...")
		self.move_head(np.array((0,-.15*np.pi)))
		print("Init Position Reached")
		
		#Subscription
		self._cloud_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2, self._cloud_cb)

	def _cloud_cb(self, msg):
		self._points_data = ros_numpy.numpify(msg)
		self._image_data = self._points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
		hsv_image = cv.cvtColor(self._image_data, cv.COLOR_RGB2HSV_FULL)
		self._image_data = cv.cvtColor(self._image_data, cv.COLOR_RGB2BGR)
		self._h_image = hsv_image[..., 0]
		self._pos_data = np.dstack((self._points_data['x'],self._points_data['y'],self._points_data['z']))
		
		if self._region_detect:
			trans_img = self._trans_cb(self._image_data,self._pos_data)
			self._region = self._crit_cb(trans_img)
		
		self._calculate_dist(self._pos_data)
		self._first_measure = True
		
	def detect_region(self, transform_cb, criteria_cb):
		self._trans_cb = transform_cb
		self._crit_cb = criteria_cb
		self._region_detect = True

	def stop_region(self):
		self._region_detect = False
		self._region = None
	
	def get_image(self):
		return self._image_data

	def get_all_data(self):
		return self._points_data

	def get_cloud(self):
		return self._pos_data

	def get_h_image(self):
		return self._h_image

	def get_region(self):
		while self._region_detect and np.any(self._region == None):
			None
		return self._region

	def get_dist_data(self):
		return (self._max_range,self._dist_data)

	def _calculate_dist(self, pos_data):
		D = np.sqrt(np.sum(pos_data**2,axis=2))
		D = np.where(np.isnan(D), -inf, D)
		self._max_range = D.max()
		self._dist_data = np.where(D==-inf, self._max_range, D)
		return self.get_dist_data()
	
	def move_head(self, move_array):
		return self._head.go(move_array)
	
	def set_measure(self):
		self._first_measure = False
	
	def get_measure(self):
		return self._first_measure
		
	def get_dist_image(self):
		D = self._dist_data / self._max_range * 255
		D = D.astype(np.uint8)
		return np.dstack((D,D,D))
	
	def gaze_point(self,x,y,z):
		#Corrige la posicion de la cabeza para que mire al punto deseado
		head_pose = self._head.get_current_joint_values()
		head_pose[0]=0.0
		head_pose[1]=0.0
		self._head.set_joint_value_target(head_pose)
		self._head.go()

		trans, e = self.tf_obj.get_transform_array()
		
		x_rob,y_rob,z_rob,th_rob= trans[0], trans[1] ,trans[2] ,  e[2]
		D_x=x_rob-x
		D_y=y_rob-y
		D_z=z_rob-z

		D_th= np.arctan2(D_y,D_x)
		print('relative to robot',(D_x,D_y,np.rad2deg(D_th)))

		pan_correct= (- th_rob + D_th + np.pi) % (2*np.pi)

		if(pan_correct > np.pi):
			pan_correct=-2*np.pi+pan_correct
		if(pan_correct < -np.pi):
			pan_correct=2*np.pi+pan_correct

		if ((pan_correct) > .5 * np.pi):
			print ('Exorcist alert')
			pan_correct=.5*np.pi
			
		head_pose[0]=pan_correct
		tilt_correct=np.arctan2(D_z,np.linalg.norm((D_x,D_y)))

		head_pose [1]=-tilt_correct

		self._head.set_joint_value_target(head_pose)
		succ=self._head.go()
		return succ

class MovController:
	"""
	La clase controlador de movimiento crea y corre una tarea de control, 
	ya sea para controlar el angulo o controlar las coordenadas
	
	Para declarar un objeto se debe especificar un objeto de posicion, 
	un objeto de movimiento y un periodo de muestreo (en segundos) 
	- si no se especifica toma un valor de 0.1 seg
	
	Para iniciar el control, se debe especificar el modo y el SP usando el metodo
	start_control
	
	Para parar el control de forma forzada, se utiliza el metodo stop_control
	
	El sistema detiene el proceso de control automaticamente cuando el error
	es menor a un limite:
		El error limite del control angular es 0.01 rad
		El error limite del control de coordenadas es 0.1 m 
		
	La diferencia entre el paro automatico y el paro forzado es que en el automatico
	se conserva la velocidad de las variables no controladas
	"""
	def __init__(self, pos, mov, obs = None, T=0.1):
		self.pos = pos 	#Objeto de Posicion
		self.mov = mov 	#Objeto de Movimiento
		self.obs = obs 	#Objeto de Obstaculos
		self.T = T 		#Periodo de muestreo
		
		self.MovControlThread = threading.Thread(target=self._control_task)	#Tarea
		self.ControlMode = 0 	#0: OFF, 1: Angle Control, 2: Coordinate Control
		self.ControlState = False
		#self.ObstacleState = False
		
		#Variables del sistema de control
		self.SP = None			#Set point
		self.error = None 		#Error e(t) = SP - x(t)
		self.output = None		#Control Variable y(t) = L-1{E(s)*TF(s)}
		self.input = None		#Variable sensada x(t)
			
		
	def _control_task(self):
		#Variables de Tiempo
		last = rospy.Time.now().to_sec()
		now = rospy.Time.now().to_sec()
		
		while True:
			if self.ControlMode == 1: 	#Angle Control	
				self.input = self.pos.get_current_angle() 	#Medir angulo

				self.error = self.SP - self.input		#Calcular error
				if self.error > np.pi:
					self.error -= 2*np.pi
				elif self.error < -np.pi:
					self.error += 2*np.pi
				
				#Constante de control proporcional
				if abs(self.error) > 0.03:
					KP = 0.3/abs(self.error)
				else:
					KP = 10
					
				self.output = KP * self.error 		#Calcular salida Kp * e(t) 
				
				#Truncar salida a 0.3 rad/s
				self.output = self.output if self.output < 0.3 else 0.3
				self.output = self.output if self.output > -0.3 else -0.3
				
				#Comandar velocidad
				self.mov.move_base(self.mov.obj.linear.x, self.mov.obj.linear.y, self.output)
				
				#Esperar periodo de muestreo
				now = rospy.Time.now().to_sec()
				while (now - last) < self.T:
					now = rospy.Time.now().to_sec()
				
				#Ultimo tiempo
				last = rospy.Time.now().to_sec()

				#Paro automatico 
				if abs(self.error) < 0.01:
					self.output = 0
					self.mov.move_base(self.mov.obj.linear.x,self.mov.obj.linear.y,0);	
					break		
				
			elif self.ControlMode == 2: #Coordinate Control
				KP = [0.15,0.15]	#Constante de control proporcional
				
				self.input = self.pos.get_coords()		#Medir coordenadas
	
				self.error = self.SP - self.input	#Calcular errores
				
				if abs(self.error[0]) > 0.25:
					KP[0] = 0.3/abs(self.error[0])
				else:
					KP[0] = 1.2
				
				if abs(self.error[1]) > 0.25:
					KP[1] = 0.3/abs(self.error[1])
				else:
					KP[1] = 1.2
				
				self.output = KP * self.error 	#Calcular salidas
				
				#Parar salida a 1cm
				self.output[0] = self.output[0] if abs(self.error[0]) >= 0.01 else 0
				self.output[1] = self.output[1] if abs(self.error[1]) >= 0.01 else 0
				
				#Transformar en base al nuevo sistema de coordenadas
				theta = self.pos.current_angle
				T_matrix = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
				self.output = np.dot(self.output, T_matrix)
				
				#Truncar salidas a 0.3 m/s
				self.output[0] = self.output[0] if self.output[0] < 0.3 else 0.3
				self.output[1] = self.output[1] if self.output[1] < 0.3 else 0.3
				self.output[0] = self.output[0] if self.output[0] > -0.3 else -0.3
				self.output[1] = self.output[1] if self.output[1] > -0.3 else -0.3
				
				#Comandar velocidades
				self.mov.move_base(self.output[0], self.output[1], self.mov.obj.angular.z)
				
				#Esperar periodo de muestreo
				now = rospy.Time.now().to_sec()
				while (now - last) < self.T:
					now = rospy.Time.now().to_sec()
					#self._obstacle_manage()
					
				#Ultimo tiempo
				last = rospy.Time.now().to_sec()
				
				#Paro automatico 
				if abs(self.output[0]) < 0.01*KP[0] and abs(self.output[1]) < 0.01*KP[1]:
					self.output[0] = 0
					self.output[1] = 0
					self.mov.move_base(0,0,self.mov.obj.angular.z);	
					break
					
			else:
				self.mov.move_base(0,0,0) #Stop
				break
		self.ControlState = False
	
	def start_control(self, mode, SP):
		self.ControlState = True
		if self.MovControlThread.is_alive():
			self.ControlMode = 0;
			while self.MovControlThread.is_alive(): #Esperar Terminacion
				None
		self.ControlMode = mode
		self.SP = SP
		self.MovControlThread = threading.Thread(target=self._control_task)
		self.MovControlThread.start()
	
	def stop_control(self):
		if self.MovControlThread.is_alive():
			self.ControlMode = 0;
			while self.MovControlThread.is_alive(): #Esperar Terminacion
				None
	
	def change_SP(self, SP):
		self.SP = SP
		
	def is_controlling(self):
		return self.ControlState

class SM_state:
	def __init__(self, identifier, exec_cb, end_cb):
		self.identifier = identifier
		self.exec_cb = exec_cb
		self.end_cb = end_cb
		
		self.stateThread = threading.Thread(target=self._state_task)
		self.interrupt = threading.Thread(target=self._interrupt_task)
		
		self.stateBreak = False
		self.next = identifier
		
	def start_state(self):
		if self.stateThread.is_alive():
			self.stateBreak = False
			while self.stateThread.is_alive():
				None
		self.stateBreak = True
		
		self.stateThread = threading.Thread(target=self._state_task)
		self.stateThread.start()
		
		time.sleep(0.1)
		if not self.interrupt.is_alive():
			self.interrupt = threading.Thread(target=self._interrupt_task)
			self.interrupt.start()
	
	def stop_state(self):
		if self.stateThread.is_alive():
			self.stateBreak = False
			while self.stateThread.is_alive():
				None
		self.stateBreak = False
	
	def _state_task(self):
		while self.stateBreak:
			self.exec_cb()
	
	def _interrupt_task(self):
		while self.stateBreak:
			self.next = self.end_cb()
					
class StateMachine:
	def __init__(self):
		self.operational = False
		self.statesN = 0
		self.statesDict = {}
		self.current_state = None
		self.previous_state = None
		self.machineThread = threading.Thread(target=self._machine_task)
	
	def add_state(self, identifier, exec_cb, end_cb):
		state = SM_state(identifier, exec_cb, end_cb)
		self.statesDict[identifier] = state
		self.stateN = self.statesN + 1
	
	def delete_state(self, identifier):
		self.stateN = self.statesN - 1
		self.statesDict.pop(identifier, None)
	
	def start_machine(self,initial_state):
		if self.machineThread.is_alive():
			self.operational = False
			while self.machineThread.is_alive():
				None
		self.operational = True
		self.current_state = initial_state
		self.previous_state = initial_state
		self.machineThread = threading.Thread(target=self._machine_task)
		self.machineThread.start()
	
	def stop_machine(self):
		if self.machineThread.is_alive():
			self.operational = False
			while self.machineThread.is_alive():
				None
	
	def _machine_task(self):
		while self.operational:
			self.statesDict[self.current_state].start_state()
			
			##Debugging
			print("Entered State ", self.current_state)
			
			while self.statesDict[self.current_state].next == self.current_state and self.operational:
				None
			self.statesDict[self.current_state].stop_state()
			self.previous_state = self.current_state
			self.current_state = self.statesDict[self.current_state].next
			
def showTimeMark():
	tim = 0
	while tim == 0:
		tim = rospy.get_time()
	minute = tim // 60
	seconds = tim - minute*60
	print("Tiempo: %d minutos, %.3f segundos" % (minute, seconds))	
	
def save_coords(path,grupo,ID,coord):	
	os.path.exists('./final_data_2020.csv')
	if not os.path.exists(path):
		file = open(path, "w")
		file.write('Grupo\tID\tCoordenadas\n')
		file.close()
	file = open(path, "a")
	file.write(grupo+"\t\t"+str(ID)+"\t"+"["+str(coord[0])+","+str(coord[1])+","+str(coord[2])+"]\n")
	file.close()
	
def best_route(list_cor,init_cor):
	iterat = list(map(list,itertools.permutations(list_cor)))
	list_ht=[]
	
	for a in range (0,len(iterat)):
		list_h=[]
		next_cor = init_cor
		for i in range (0,len(list_cor)):
			list_h.append(sqrt((iterat[a][i][0]- next_cor[0])**2+(iterat[a][i][1]- next_cor[1])**2))
			next_cor = iterat[a][i]
		list_ht.append(sum(list_h))
	mejor_ruta = iterat[list_ht.index(min(list_ht))]
	return mejor_ruta
	
def get_gaze_target(pos, point, offset):
	point = np.array(point)
	coords = pos.get_coords()
	displaz = point - coords
	distance = np.linalg.norm(displaz)
	disp_slice = (distance - offset) / distance
	target = coords + displaz*disp_slice
	return target

def object_segmentation(dist_img, rgb_img, max_range, cloud, zone=1, display=False):
	if zone == 1:
		sat_loop = 4
		area_th = 150
	elif zone == 2:
		sat_loop = 2
		area_th = 50
	else:
		sat_loop = 2
		area_th = 100
    
	#Distance Filtering
	D_mask = dist_img[:,:,0]
	D_th_max = int(255*5/max_range) #5 meter threashold
	D_th_min = int(255*1.5/max_range) #2 meter threashold

	logic = np.bitwise_and(D_mask <= D_th_max, D_mask >= D_th_min)
	mask_D = np.where(logic, 255, 0).astype('uint8')
	rgb_masked_D = cv.bitwise_and(rgb_img, rgb_img, mask=mask_D)

	#Display
	if display:
		cv.imshow("D mask",mask_D)
		cv.imshow("RGB masked", rgb_masked_D)

	#Saturation Filtering
	hsv_img = cv.cvtColor(rgb_masked_D,cv.COLOR_BGR2HSV)
	sat = hsv_img[:,:,1]

	#sat = np.where(sat > 128, 255, 0).astype('uint8')
	ret, sat1 = cv.threshold(sat,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)

	#Display
	if display:
		cv.imshow("Sat Img 1", sat1)

	SE = np.ones((3,3),np.uint8)
	sat2 = cv.morphologyEx(sat1,cv.MORPH_CLOSE,SE, iterations = 2)
	sat2 = cv.morphologyEx(sat2,cv.MORPH_OPEN,SE, iterations = 1)

	#Display
	if display:
		cv.imshow("Sat Img 2", sat2)

	sat3 = cv.GaussianBlur(sat2,(7,7), cv.BORDER_DEFAULT)
	ret, sat3 = cv.threshold(sat3,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)

	#Display
	if display:
		cv.imshow("Sat Img 3", sat3)

	#Sat1 + Sat3
	sat4 = cv.bitwise_or(sat1, sat3)
	sat4 = cv.GaussianBlur(sat4,(3,3), cv.BORDER_DEFAULT)
	ret, sat4 = cv.threshold(sat4,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
	sat4 = cv.morphologyEx(sat4,cv.MORPH_CLOSE,SE, iterations = 2)
	sat4 = cv.GaussianBlur(sat4,(5,5), cv.BORDER_DEFAULT)
	ret, sat4 = cv.threshold(sat4,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
	
	for i in range(sat_loop):
		sat4 = cv.bitwise_or(sat1, sat4)
		sat4 = cv.GaussianBlur(sat4,(3,3), cv.BORDER_DEFAULT)
		ret, sat4 = cv.threshold(sat4,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
		sat4 = cv.morphologyEx(sat4,cv.MORPH_CLOSE,SE, iterations = 2)
		sat4 = cv.GaussianBlur(sat4,(5,5), cv.BORDER_DEFAULT)
		ret, sat4 = cv.threshold(sat4,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)

	"""
	sat4 = cv.bitwise_or(sat1, sat4)
	sat4 = cv.GaussianBlur(sat4,(3,3), cv.BORDER_DEFAULT)
	ret, sat4 = cv.threshold(sat4,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
	sat4 = cv.morphologyEx(sat4,cv.MORPH_CLOSE,SE, iterations = 2)
	sat4 = cv.GaussianBlur(sat4,(5,5), cv.BORDER_DEFAULT)
	ret, sat4 = cv.threshold(sat4,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)

	#Added
	sat4 = cv.bitwise_or(sat1, sat4)
	sat4 = cv.GaussianBlur(sat4,(3,3), cv.BORDER_DEFAULT)
	ret, sat4 = cv.threshold(sat4,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
	sat4 = cv.morphologyEx(sat4,cv.MORPH_CLOSE,SE, iterations = 2)
	sat4 = cv.GaussianBlur(sat4,(5,5), cv.BORDER_DEFAULT)
	ret, sat4 = cv.threshold(sat4,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)

	sat4 = cv.bitwise_or(sat1, sat4)
	sat4 = cv.GaussianBlur(sat4,(3,3), cv.BORDER_DEFAULT)
	ret, sat4 = cv.threshold(sat4,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
	sat4 = cv.morphologyEx(sat4,cv.MORPH_CLOSE,SE, iterations = 2)
	sat4 = cv.GaussianBlur(sat4,(5,5), cv.BORDER_DEFAULT)
	ret, sat4 = cv.threshold(sat4,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
	"""
	
	#Display
	if display:
		cv.imshow("Sat Img 4", sat4)

	contours, hierarchies = cv.findContours(sat4, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
	centroids = []

	for cnt in contours:
		area = cv.contourArea(cnt)
		print(area)
		if area > area_th:
			mask = np.zeros(sat4.shape, dtype=np.uint8)
			mask = cv.drawContours(mask,[cnt],0,255,-1)

			xyz = []
			for i in range(mask.shape[0]):
				for j in range(mask.shape[1]):
					if mask[i][j] == 255:
						x_px = cloud[i][j][0]
						y_px = cloud[i][j][1]
						z_px = cloud[i][j][2]
						xyz_px = np.array([x_px, y_px, z_px])
						if not np.any(np.isnan(xyz_px)):
							xyz.append(xyz_px)
			xyz = np.array(xyz)
			cent = xyz.mean(axis=0)
			centroids.append(cent)
			
	zone = "Zona"+str(zone)+"_"
	cv.imwrite(zone+"last_RGB.jpg", rgb_img)
	cv.imwrite(zone+"last_mask.jpg",sat4)
	return np.array(centroids)
    

def save_rgbd_data(image_rgb, dist_image, max_range, dist_data, cloud, zone, folder):
	zone = "Zona"+str(zone)+"_"
	
	cv.imwrite(folder+'/'+zone+"RGB.jpg",image_rgb)#Image
	cv.imwrite(folder+'/'+zone+"Dist.jpg",dist_image) #Image
	
	f = open(folder+'/'+zone+"MaxRange.npy", 'w+b')
	np.save(f, max_range)
	f.close()
	f = open(folder+'/'+zone+"DistData.npy", 'w+b')
	np.save(f, dist_data)
	f.close()
	f = open(folder+'/'+zone+"CloudXYZ.npy", 'w+b')
	np.save(f, cloud)
	f.close()

def test_save_coords():
	path = 'test.txt'
	testG = 'A'
	testID = 1
	testC = [1,2,3]
	save_coords(path, testG, testID, testC)
	save_coords(path, 'B', 2, [2,3,5])

def test_best_route():
	resultado = best_route([[-1,2],[4,-1],[2,5],[6,6]],[6,2])
	print(resultado)
	
def test_move(pos, mov, cntrl):
	target_coords = np.array([[0.0,1.21],[-3.0,4.0],[3.9,5.6]])
	
	test_best_route()
	route = best_route(target_coords, [0,0])
	
	for point in route:
		print("Targeting to", point)
		now = pos.get_coords()
		distance = sqrt(sum((now - point)**2))
		while distance > 0.01*sqrt(2):
			pos.final = point
			target_angle = pos.find_angle()
			
			angle_dif = target_angle - pos.current_angle
			if angle_dif > np.pi:
				angle_dif -= 2*np.pi
			elif angle_dif < -np.pi:
				angle_dif += 2*np.pi
				
			if abs(angle_dif) > 0.1:
				print("Controlling Angle")
				cntrl.start_control(1, target_angle)
				while cntrl.is_controlling():
					None
				print("Angle Adquired")
			
			else:
				print("Controlling Position")
				cntrl.start_control(2, point)
				while cntrl.is_controlling():
					None
				print("Position Adquired")
			
			now = pos.get_coords()
			distance = sqrt(sum((now - point)**2))
		print("Point", point, "has been reached")

def test_rgbd(rgbd):
	rgbd.set_measure()
	while not rgbd.get_measure():
		None
	
	image = rgbd.get_image()
	D = rgbd.get_dist_image()
	image = cv.cvtColor(image,cv.COLOR_RGB2BGR)
	cv.imwrite("Image.jpg", image)
	cv.imwrite("Distance.jpg", D)

def test_tf(cntrl):
	tf_inst = TF()
	
	cntrl.start_control(2, [0,1])
	while cntrl.is_controlling():
		None
	print("Point reached")
	tf_inst.send_trans_by_obj((0,-1,0),(0,0,0,1),'test1')
	
	print("Sent")
	#tf_inst.set_obj("test1")
	#tf_inst.get_transform()
	#print("Read")

def test_obj_seg():
    #Test Values Start
    f = 'Drill_Test_01'
    cloud = np.load(f+'/CloudXYZ.npy')
    dist = np.load(f+'/DistData.npy')
    max_range = np.load(f+'/MaxRange.npy')
    dist_img = cv.imread(f+'/Dist.jpg')
    rgb_img = cv.imread(f+'/RGB.jpg')
    rgb_img = cv.cvtColor(rgb_img, cv.COLOR_RGB2BGR)
    #Test Values End

    c = object_segmentation(dist_img, rgb_img, max_range, cloud)
    print(c)

"""
cv.imwrite("RGB.jpg",rgbd.get_image())#Image
cv.imwrite("Dist.jpg",rgbd.get_dist_image()) #Image

(maxi, data) = rgbd.get_dist_data() #Float + Array
cloud = rgbd.get_cloud() #Array

f = open("MaxRange.npy", 'w+b')
np.save(f, maxi)
f.close()
f = open("DistData.npy", 'w+b')
np.save(f, data)
f.close()
f = open("CloudXYZ.npy", 'w+b')
np.save(f, cloud)
f.close()
"""


