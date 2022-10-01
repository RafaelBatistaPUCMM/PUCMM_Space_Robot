#!/usr/bin/env python3
import rospy, tf, random
import sys
import os
from gazebo_msgs.srv import DeleteModel, SpawnModel
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Pose

rospy.init_node('spawn_node',log_level=rospy.INFO)

roll = 0
pitch = 0
yaw = 0

x = 1
y = -1
z = 0.71

#orientation
q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

# object pose
initial_pose = Pose()
initial_pose.position.x = x
initial_pose.position.y = y
initial_pose.position.z = z
initial_pose.orientation.x = q[0]
initial_pose.orientation.y = q[1]
initial_pose.orientation.z = q[2]
initial_pose.orientation.w = q[3]

# Finding the model file to be spawned
#file = os.path.expanduser('~/CIRE2022/models/ycb_077_rubiks_cube/model-1_4.sdf')
#file = os.path.expanduser('/opt/ros/noetic/share/tmc_gazebo_worlds/models/cat/model-1_4.sdf')
#file = os.path.expanduser('~/CIRE2022/models/ycb_042_adjustable_wrench/model-1_4.sdf')

#file = os.path.expanduser('/home/cire2022/gazebo-models/new_models/monkey_wrench/model-1_4.sdf')

#file = os.path.expanduser('/home/cire2022/gazebo-models/new_models/rubiks_cube/rubiks_cube.sdf')
#file = os.path.expanduser('/home/cire2022/gazebo-models/new_models/rubiks_cube/model.sdf')
file = os.path.expanduser('/home/cire2022/gazebo-models/new_models/cordless_drill/model-1_4.sdf')
#file = os.path.expanduser('/home/cire2022/.gazebo/models/cordless_drill/model-1_4.sdf')

f = open(file, 'r')
sdff = f.read()
f.close()

rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

i = 3
spawn_model_prox("drill_"+str(i), sdff, "", initial_pose, "world")

try:
	print("PROCESS STARTED")
	rospy.spin()
except:
	None






