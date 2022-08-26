#!/usr/bin/env python3
#
# CONCURSO IBEROAMERICANO DE ROBOTICA ESPACIAL, PEU-UNAM, 2022
# ETAPA 02 - ROS Y EL ROBOT HSR
# EJERCICIO 01
#
# Instrucciones:
# Complete el programa para mover el robot hacia el frente
# hasta que el laser detecte un obstaculo al frente.
# Los publicadores y suscriptores requeridos ya se encuentran
# declarados e inicializados
#

import rospy
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist

EQUIPO = "PUCMM Space Robot"

def callback_scan(msg):
    global obstacle_detected
    #
    # EJERCICIO:
    # Haga algo para detectar si hay un obstaculo enfrente del robot.
    # Asigne True o False a la variable 'obstacle_detected'.
    #
    
    n = int((msg.angle_max-msg.angle_min)/msg.angle_increment/2)
    osbtable_Detected = msg.ranges[n] < 1.0
    
    return

def main():
    print("EJERCICIO 01 - " + EQUIPO)
    rospy.init_node("ejercicio01")
    rospy.Subscriber("/hsrb/base_scan", LaserScan, callback_scan)
    pub_cmd_vel = rospy.Publisher("/hsrb/command_velocity", Twist, queue_size=10)
    loop = rospy.Rate(10)
    
    global obstacle_detected
    global msg_cmd_vel
    obstacle_detected = False
    while not rospy.is_shutdown():
        #
        # EJERCICIO:
        # Declare un mensaje de tipo Twist y asigne las componentes de velocidad apropiadas:
        # Velocidad positiva en x si no hay obstaculo.
        # Velocidad cero si hay obstaculo.
        # Use la variable 'obstacle_detected' para revisar si hay o no obstaculo al frente.
        # Publique el mensaje de tipo Twist usando el publicador 'pub_cmd_vel'.
        #
        
        msg_cmd_vel = Twist()
        msg_cmd_vel.linear.x=0 if obstacle_detected else 0.3
        pub_cmd_vel.publish(msg_cmd_vel)
        
        loop.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
