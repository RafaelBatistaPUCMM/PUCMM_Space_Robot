#!/usr/bin/env python3
import roslaunch
import rospy

launch = roslaunch.scriptapi.ROSLaunch()

def main():
	rospy.init_node("periodic_mapping")
	loop = rospy.Rate(0.3)
	delay = rospy.Rate(2)
	save = roslaunch.core.Node("map_server", "map_saver", args='-f current_map')
	server = roslaunch.core.Node("map_server", "map_server", args='current_map.yaml')
	inflation = roslaunch.core.Node("etapa04", "map_inflation.py")
	cost = roslaunch.core.Node("etapa04", "cost_map.py")

	launch.start()
	
	while not rospy.is_shutdown():
		script_save = launch.launch(save)
		script_server = launch.launch(server)
		script_inflation = launch.launch(inflation)
		script_cost = launch.launch(cost)
		
		loop.sleep()
		
		script_save.stop()
		script_server.stop()
		script_inflation.stop()
		script_cost = launch.launch(cost)
		print("Executed \n")
		if rospy.get_param('/map_request', False):
			rospy.set_param('/map_response',True)
		else:
			rospy.set_param('/map_response',False)
		
	
	launch.stop()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		launch.stop()
		pass
