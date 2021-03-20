#!/usr/bin/python

import rospy
from std_msgs.msg import Float32, Int16, String
from Control.state_controllers import rudder_controller as rudder_class
from Control.state_controllers import sail_controller as sail_class
from Navigation.Navigator import Navigator as Navigator_class
from sailboat.msg import Wind
import numpy as np
from sailboat.cfg import pidConfig
from dynamic_reconfigure.server import Server

import time

class sailing():

    def __init__(self):
	
	rospy.init_node("tasks_runner", anonymous=True)	
	self.sample_time = rospy.get_param("simulation/sample_time") # rate at which this node runs
        self.node = rospy.Rate(self.sample_time) # wait function to use inside threads
	
	self.Navigator = Navigator_class()

	rospy.Subscriber('task', String, self.get_task)
	self.task = 'go_to_waypoint'

	rospy.Subscriber('apparent_wind',Wind, self.get_wind_vector)
	self.apparent_wind = Wind()
	self.true_wind_angle = 0

	#rospy.Subscriber('estim_heading', Float32, self.get_estim_heading)
	#self.estim_heading = rospy.get_param('scenario/initial_state/heading')

	rospy.Subscriber('heading', Float32, self.get_heading)
	self.heading = rospy.get_param('scenario/initial_state/heading')	

	rospy.Subscriber('goal_heading',Float32, self.get_goal_heading)	
	self.goal_heading = 0.0
	self.goal = 0.0
	self.last_goal = goal = 0
	
	self.set_rudder_angle = rospy.Publisher('rudder_angle',Int16, queue_size = 10)
	self.set_sail_angle = rospy.Publisher('sail_angle',Int16, queue_size = 10)
	self.state_dbg = rospy.Publisher('dbg_state', String, queue_size = 10)

	self.beating_angle = rospy.get_param('navigation/wind_beating_angle')
	self.action_ongoing = 0
	self.max_sail_angle = rospy.get_param("sail/max_angle")
	self.min_sail_angle = rospy.get_param("sail/min_angle")

	
	self.init_controllers() # initialize and set up both rudder and sail controllers
	
	
	srv = Server(pidConfig, self.pid_callback)
	self.sail() # state machine responsible to pick the proper sailing technique and set rudder and sail commands accordingly

    def pid_callback(self,config, level):
     
        Kp = config.Kp
        Ki = config.Ki
        Kd = config.Kd
        self.rudder_controller.PID.setGains(Kp,Ki,Kd)

        return config

    def get_goal_beating(self, data):
	self.goal_beating = data.data

    def get_task(self, data):
	self.task = data.data

    def get_sailing_state(self, data):
	self.sailing_state = data.data

    def get_heading(self, data):
	self.heading = data.data

    #def get_estim_heading(self, data):
	#self.heading = data.data

    def get_goal_heading(self,data):
	self.goal_heading = data.data

    def get_wind_vector(self,data):
	
	self.apparent_wind = data
	self.true_wind_angle = self.Navigator.convert_apparent_to_true(self.heading, self.apparent_wind.angle)
	
	if self.apparent_wind.angle > 180:
            self.apparent_wind.angle -= 360


    def init_controllers(self):

	self.kp = rospy.get_param('controller/kp')
	self.ki = rospy.get_param('controller/ki')
	self.kd = rospy.get_param('controller/kd')
	self.maxOut = rospy.get_param('rudder/max_angle')
	self.maxSpeed = rospy.get_param('model/maxAngularSpeed')
	self.cut_freq = rospy.get_param('controller/cuttof_frequency')
	self.direction = rospy.get_param('controller/direction')
	
	self.rudder_controller = rudder_class(self.kp, self.ki, self.kd, self.maxOut, -self.maxOut, self.maxOut,-self.maxOut,self.direction, 
			self.maxSpeed,-self.maxSpeed, self.cut_freq)

	self.sail_controller = sail_class(self.max_sail_angle,self.min_sail_angle)

    def calculate_sailing_maneuver(self, goal_heading,current_heading, true_wind_angle):

	goal_against_wind, maneuever = self.Navigator.angle_is_upwind(goal_heading,true_wind_angle )
	heading_against_wind, maneuever_tack = self.Navigator.angle_is_upwind(current_heading,true_wind_angle )
	dummy, maneuever_jibe = self.Navigator.angle_is_downwind(current_heading,true_wind_angle )
	self.switch = 1
	
	if(goal_against_wind and heading_against_wind): # beating

	    if(self.last_state != "beating_wind"):
		self.switch = -self.switch
	        

	    if(maneuever == "tacking_to_starboard"):
		angle_to_beat = -self.beating_angle*self.switch
		offset = -10*self.switch
	    else:
		angle_to_beat = self.beating_angle*self.switch
		offset = 10*self.switch

	    

	    self.last_state = "beating_wind"
	    goal = self.Navigator.heading_opposite_to_wind(angle_to_beat + true_wind_angle + offset, true_wind_angle)
	    self.last_goal = goal
	  
	    return "beating_wind", goal
	
	
	if(goal_against_wind and (not heading_against_wind) ): # jibe

	    self.last_state = maneuever_jibe
	    return maneuever_jibe, self.last_goal

	
	if( (not goal_against_wind) and heading_against_wind ): # tack
	    self.last_state = maneuever_tack
	    return maneuever_tack, self.last_goal

	
	if((not goal_against_wind) and (not heading_against_wind) ): #normal
	    self.last_state = "normal"
	    self.last_goal = goal_heading
	    return "normal", goal_heading # no limitations, sails in a straight line towards the goal


    def send_rudder_cmd(self, rudder_angle):

	self.set_rudder_angle.publish(rudder_angle)

    def send_sail_cmd(self, sail_angle):

	self.set_sail_angle.publish(sail_angle)


    def beat_the_wind(self,beating_angle, goal_heading, current_heading, true_wind):

	error = self.Navigator.angle_subtract(beating_angle,current_heading)
	rudder_angle = -self.rudder_controller.PID.calculate(error,current_heading,1)
	sail_angle = self.min_sail_angle	   
	ongoing = 1

	check, maneuver = self.Navigator.angle_is_upwind(goal_heading,true_wind)
	if( not (check) ):
	    ongoing = 0

	return rudder_angle, sail_angle, ongoing

    def run(self):
	self.straight_line()

    def tack(self,switch_side,curr_heading, apparent_wind, true_wind):

	if(switch_side == "to_port"):
	    rudder_angle = -40 #turn right

	elif(switch_side == "to_starboard"):
	    rudder_angle = 40 #turn left

	sail_angle = self.sail_controller.calculate_sail_angle(apparent_wind)

	ongoing = 1
	check, maneuver = self.Navigator.angle_is_upwind(curr_heading,true_wind)
	if( not (check) ):
	    ongoing = 0

	return rudder_angle, sail_angle, ongoing

    def jibe(self,switch_side,curr_heading, apparent_wind, true_wind):
	
	if(switch_side == "to_port"):
	    rudder_angle = -40 #turn right

	elif(switch_side == "to_starboard"):
	    rudder_angle = 40 #turn left
	  
	sail_angle = self.sail_controller.calculate_sail_angle(apparent_wind)
	
	ongoing = 1
	
	check, maneuver = self.Navigator.angle_is_downwind(curr_heading,true_wind)
	if( not (check) ):
	    ongoing = 0

	return rudder_angle, sail_angle, ongoing


    def straight_line(self, goal_heading, current_heading):

	error = self.Navigator.angle_subtract(goal_heading,current_heading)
	rudder_angle = -self.rudder_controller.PID.calculate(error,current_heading,1)
	sail_angle = self.sail_controller.calculate_sail_angle(self.apparent_wind.angle)
	

	return rudder_angle, sail_angle, 0
	

    def sail_in_circles(self, goal_heading, current_heading):

	error = self.Navigator.angle_subtract(goal_heading,current_heading)
	rudder_angle = -self.rudder_controller.PID.calculate(error,current_heading,1)
	sail_angle = self.sail_controller.calculate_sail_angle(self.apparent_wind.angle)
	
	return rudder_angle, sail_angle
	
    def keep_position(self):

	self.sail_in_circles(self.goal, self.heading)

    #def stop(self):

	#self.sail_in_circles(self.goal, self.heading)
	
    def go_to_waypoint(self):
		

	if(self.action_ongoing == 1):
	    self.state = self.last_state
	    self.goal = self.last_goal
	else:
	    self.state, self.goal = self.calculate_sailing_maneuver(self.goal_heading,self.heading,self.true_wind_angle)
	
        #rospy.logwarn(state)
	self.state_dbg.publish(self.state)


	if( self.state == "beating_wind" ):

	    self.rudder_angle, self.sail_angle, self.action_ongoing = self.beat_the_wind(self.goal, self.goal_heading, self.heading, self.true_wind_angle)

	    self.last_state = "beating_wind" 

	elif( self.state == "tacking_to_port" ):

	    self.rudder_angle, self.sail_angle, self.action_ongoing = self.tack("to_port", self.heading, self.apparent_wind.angle ,self.true_wind_angle)
	   
            self.last_state = "tacking_to_port"

	elif( self.state == "tacking_to_starboard" ):

	    self.rudder_angle, self.sail_angle, self.action_ongoing = self.tack("to_starboard", self.heading, self.apparent_wind.angle ,self.true_wind_angle)

	    self.last_state = "tacking_to_starboard"

	elif( self.state == "jibing_to_port" ):

	    self.rudder_angle, self.sail_angle, self.action_ongoing = self.jibe("to_port",self.heading, self.apparent_wind.angle ,self.true_wind_angle)

	    self.last_state = "jibing_to_port"

	elif( self.state == "jibing_to_starboard" ):

	    self.rudder_angle, self.sail_angle, self.action_ongoing = self.jibe("to_starboard",self.heading, self.apparent_wind.angle ,self.true_wind_angle)

	    self.last_state = "jibing_to_starboard"

	else:
	    self.rudder_angle, self.sail_angle, self.action_ongoing = self.straight_line(self.goal, self.heading)
	    self.last_state = "normal"


	self.send_rudder_cmd(self.rudder_angle)
	self.send_sail_cmd(self.sail_angle)
	
	
    


    def sail(self):

	while not rospy.is_shutdown():

	    if (self.task == "go_to_waypoint"): 
	        self.go_to_waypoint()

	    if (self.task == "keep_position"): 
	        self.keep_position()

	    #if (self.task == "stop"): 
	        #self.stop()


	    self.node.sleep()

if __name__ == '__main__':
    try:
        sailing()
    except rospy.ROSInterruptException:
        pass

