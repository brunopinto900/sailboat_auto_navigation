#!/usr/bin/python
from std_msgs.msg import Float32, Int16, String
from nav_msgs.msg import Odometry 
import rospy
from sailboat.msg import Wind
from Navigation.Navigator import Navigator as Navigator_class
import math, mpmath

class planner():

    def __init__(self):

	rospy.init_node("tasks_planner", anonymous=True)
	self.sample_time = rospy.get_param("simulation/sample_time") # rate at which this node runs
        self.node = rospy.Rate(self.sample_time) # wait function to use inside threads

	self.Navigator = Navigator_class()

	rospy.Subscriber('position', Odometry, self.get_position)
	self.position = Odometry()

	rospy.Subscriber('heading', Float32, self.get_heading)
	self.heading = 0

	rospy.Subscriber('apparent_wind', Wind, self.get_wind)
	self.apparent_wind = Wind()
	self.true_wind_angle = 0
	
	self.set_goal_heading = rospy.Publisher('goal_heading', Float32, queue_size = 10)
	self.set_task = rospy.Publisher('task', String, queue_size = 10)

	self.target_radius = rospy.get_param("gps/goal_radius")

	self.publish_goal_task()

    

    def get_heading(self,data):
	self.heading = data.data

    def get_position(self,data):
	self.position = data

    def get_wind(self,data):
	
	self.apparent_wind.speed = data.speed
	self.apparent_wind.angle = data.angle
	self.true_wind_angle = self.Navigator.convert_apparent_to_true(self.heading, self.apparent_wind.angle)
	

    def publish_goal_task(self):
	_list = rospy.get_param("gps/list")
	_coords = rospy.get_param("gps/coordinates")
        gps_coordinates = self.Navigator.load_gps_coordinates(_list,_coords)
	n_coords = len(gps_coordinates)
	first_wp = -1
	second_wp = 0
	next_pos = gps_coordinates[first_wp+1]
	current_pos = [0,0]
	self.task ="go_to_waypoint"
	self.set_task.publish(self.task)

	while not rospy.is_shutdown():
	
	    current_pos[0] = self.position.pose.pose.position.y
	    current_pos[1] = self.position.pose.pose.position.x

	    goal = self.Navigator.calculate_goal_heading(current_pos,next_pos)
	    self.set_goal_heading.publish(goal)

            if( self.Navigator.check_end_condition(next_pos, current_pos,self.target_radius) ):
                first_wp = first_wp + 1
		second_wp = second_wp + 1
	        if ( (not second_wp == n_coords) and (not first_wp == n_coords) ):

	            next_pos = gps_coordinates[second_wp]	  

	    if ( second_wp == n_coords or first_wp == n_coords ): # just keeps position
	        self.task ="keep_position"
	        self.set_task.publish(self.task)
	    
	        second_wp = n_coords-1
	        first_wp = n_coords-1
                rospy.logwarn("Position Keeping")
	    
	    self.node.sleep()

if __name__ == '__main__':
    try:
        planner()
    except rospy.ROSInterruptException:
        pass

