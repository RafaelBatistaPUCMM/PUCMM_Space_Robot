#!/usr/bin/env python3

#Modulos generales
import rospy
import numpy as np
import ros_numpy
import tf2_ros

#Modulos de los topicos
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist

#Modulos de calculo
from tf.transformations import euler_from_quaternion
from math import atan, sqrt

#Modulos especificos
import threading
import time

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
    def __init__(self, nodoMeta):
    	#Subscripcion al nodo que publica la meta
        goal_node = rospy.Subscriber(nodoMeta, PoseStamped, self.read_goal)
        
        #Mediciones periodicas
        self.delta = 1											#Periodo
        self.UpdateThread = threading.Thread(target=self.Timer)	#Tarea
        self.task_state = False
        
        #Variables para la obtencion de coordenadas
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        #self.listener = tf.TransformListener()
        
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
		3.get_obs_params --> retorna un objeto Obstacle con los parametros de obstaculo
	"""
	def __init__(self):
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
		self.obs_lim = 1
		
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
		c = self.lectura * np.sin(angles_tf) #Componente horizontal de la distancia
		d = np.where(c <= ref_space, d, inf) #Eliminando (inf) los valores que no estan dentro de la zona de choque
		
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
				angle_shift = 90
			
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
	y la salida son	inferiores a un limite:
		El error limite del control angular es 0.01 rad
		La velocidad limite del control angular es 0.01 rad/s
		El error limite del control de coordenadas es 0.01 m 
		La velocidad limite del control de coordenadas es 0.001 m/s
		
	La diferencia entre el paro automatico y el paro forzado es que en el automatico
	se conserva la velocidad de las variables no controladas
	"""
	def __init__(self, pos, mov, T=0.1):
		self.pos = pos 	#Objeto de Posicion
		self.mov = mov 	#Objeto de Movimiento
		self.T = T 		#Periodo de muestreo
		
		self.MovControlThread = threading.Thread(target=self._control_task)	#Tarea
		self.ControlMode = 0 	#0: OFF, 1: Angle Control, 2: Coordinate Control
		
		#Variables del sistema de control
		self.SP = None			#Set point
		self.error = None 		#Current Error e(t) = SP - x(t)
		self.output = None		#Control Variable y(t) = L-1{E(s)*TF(s)}
		self.output_1 = 0		#Anterior Control Variable y(t) = L-1{E(s)*TF(s)}
		self.input = None		#Current Variable sensada x(t)
		self.input_1 = 0		#Anterior Variable sensada x(t)
			
		
	def _control_task(self):
		#Variables de Tiempo
		last = rospy.Time.now().to_sec()
		now = rospy.Time.now().to_sec()
		while True:
			if self.ControlMode == 1: 	#Angle Control	
				KP = 0.3	#Constante de control proporcional
				self.input = self.pos.get_current_angle() 	#Medir angulo
				self.error = self.SP - self.input		#Calcular error
				self.output = KP * self.error			#Calcular salida Kp * e(t)
				
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
				KP = 0.15	#Constante de control proporcional
				KI = 0.01
				self.input = self.pos.get_coords()		#Medir coordenadas
				self.error = self.SP - self.input	#Calcular errores
				self.output = KP * self.error		#Calcular salidas
										
				#Parar salida a 10cm
				self.output[0] = self.output[0] if abs(self.error[0]) >= 0.1 else 0
				self.output[1] = self.output[1] if abs(self.error[1]) >= 0.1 else 0
				
				if(abs(self.error[1]) <= 0.1):
					self.output = KI*(self.output_1+(self.T/2)*(self.input+self.input_1))
					self.output[0] = self.output[0] if abs(self.error[0]) >= 0.05 else 0
					self.output[1] = self.output[1] if abs(self.error[1]) >= 0.05 else 0
					self.input_1=self.input
					self.output_1=self.output
				
				#Transformar en base al nuevo sistema de coordenadas
				theta = self.pos.current_angle
				T_matrix = np.array([[np.cos(theta), np.sin(theta)],[-np.sin(theta), np.cos(theta)]])
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
					
				#Ultimo tiempo
				last = rospy.Time.now().to_sec()
				
				#Paro automatico 
				if abs(self.output[0]) < 0.1*KP and abs(self.output[1]) < 0.1*KP:
					self.output[0] = 0
					self.output[1] = 0
					self.mov.move_base(0,0,self.mov.obj.angular.z);	
					break
						
			else:
				self.mov.move_base(0,0,0) #Stop
				break
	
	def start_control(self, mode, SP):
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
		

	
def test_no_obs():
    rospy.init_node('etapa02_node')
    frame = Posicion('/meta_competencia');
    mov = Movimiento()
    cntrl = MovController(frame, mov)
    time.sleep(5)
    angle = frame.find_angle()
    time.sleep(5)
    angle = frame.find_angle()
    print("Target Angle (rad): ")
    print(angle)
    print("Initiating Control")
    print("")
    cntrl.start_control(1, angle)
    laser = Laser()
    rate = rospy.Rate(2)
    state = 1
    count = 0
    while not rospy.is_shutdown():

        #mov.move_base(0,0,-0.3)
        angle = frame.find_angle()
        print("Target Angle (rad): ",end="")
        print(angle)
        print("Target Pos: ",end="")
        print(frame.final)
        print("Pos Actual: ",end="")
        print(frame.actual)
        print("Ang Actual: ",end="")
        print(frame.current_angle)
        print("Ang Vel Actual: ",end="")
        print(mov.obj.angular.z)
        print("Lin X Vel Actual: ",end="")
        print(mov.obj.linear.x)
        print("Lin Y Vel Actual: ",end="")
        print(mov.obj.linear.y)
        print("")
        
        if cntrl.ControlMode == 1:
            cntrl.change_SP(angle)
        
        coord_1 = frame.actual
        coord_2 = frame.final
        distance = sqrt(sum((coord_2 - coord_1) ** 2))
        if distance > 0.1 and cntrl.ControlMode == 1 and cntrl.output == 0:
            mov.move_base(0.3,0,0)
            if abs(frame.target_angle - frame.current_angle) > 0.5 and distance > sqrt(2):
                mov.move_base(0,0,0);
                cntrl.start_control(1, angle)
                print("Initiating Correction")
            elif distance <= sqrt(2):
                mov.move_base(0,0,0);
                cntrl.start_control(2, frame.final)
        
        """
        print("Pos Final: ",end="")
        print(frame.final)
        print("Ang Final: ",end="")
        print(frame.final_angle)
        
        print("Ang Objetivo: ",end="")
        print(frame.target_angle)
        
        print("")
        
        if laser.meas_state:
        	print(min(laser.lectura))
        	print(max(laser.lectura))
        
        if state == 1:
        	print("Not inverted")
        	mov.move_base_vel(0.1,0,0)
        elif state == -1:
        	print("Inverted")
        	mov.move_base_vel(-0.1,0,0)
        
        count = count + 1
        if count == 10:
        	state = state * -1
        	count = 0
        """
        rate.sleep()
   
   
