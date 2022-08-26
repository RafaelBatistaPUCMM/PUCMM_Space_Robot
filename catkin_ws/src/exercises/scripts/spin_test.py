#!/usr/bin/env python3

import rospy
import numpy as np
import ros_numpy
from sensor_msgs.msg   import Imu
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class Orientation:
	def __init__(self):
		orientation_node = rospy.Subscriber('/hsrb/base_imu/data', Imu, self.imu_cb)
	
	def imu_cb(self, msg):
		orientation_q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
		(roll, pitch, yaw) = euler_from_quaternion(orientation_q)
		print(yaw)		
		

def main():
	rospy.init_node('etapa02_node')
	#angle = Orientation()
	rate = rospy.Rate(2)
	while not rospy.is_shutdown():
		tim = rospy.get_time()
		minute = tim // 60
		seconds = tim - minute*60
		print("%d minutos, %.3f segundos" % (minute, seconds))
		print("")
		rate.sleep()
    		
       
if __name__ == '__main__':
    main()
    
