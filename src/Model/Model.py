#!/usr/bin/python

import rospy
from sympy import *

class Models(object):
    
    def __init__(self):

	self.CLs = rospy.get_param("model/sail_lift_coefficient")
	self.CLr = rospy.get_param("model/rudder_lift_coefficient")
	self.mass = rospy.get_param("model/mass")
	self.friction = rospy.get_param("model/tangential_friction")
	self.rudder_break = rospy.get_param("model/rudder_break")
	self.d_mast_sail = rospy.get_param('model/dist_mast_to_center_sail')
	self.d_mast_COG = rospy.get_param('model/dist_mast_to_center_of_gravity')
	self.d_rudder_COG = rospy.get_param('model/dist_rudder_to_center_of_gravity')
	self.ang_friction = rospy.get_param('model/angular_friction')
	self.inertia = rospy.get_param('model/moment_of_inertia')


    def buildModel(self):

	x,y,theta, x_dot, y_dot, theta_dot = symbols('x y theta x_dot y_dot theta_dot') # states
	ds, dr, AWA, AWS = symbols('delta_s delta_r AWA AWS') # inputs (rudder, sail angles and apparent wind angle and speed
	states_names = Matrix([x,y,theta, x_dot, y_dot, theta_dot])
	inputs_names = Matrix([ds, dr, AWA, AWS])
	  
	angle_of_attack = (ds - AWA) #here i can use a built in function to be bounded to 360 degrees
	fs = self.CLs*AWS*sin(angle_of_attack)  
	fs_vector = Matrix ( [fs*sin(ds) , fs*cos(ds) ] )

	tang_frict_force_x = -self.friction*(x_dot**2) #Vx
	tang_frict_force_y = -self.friction*(y_dot**2)
	fr_x = self.CLr*(x_dot**2)*sin(dr)  
	fr_y = self.CLr*(y_dot**2)*cos(dr) 

	
 	k = self.ang_friction*theta_dot*x_dot
	   

	model = Matrix( [ 			
						x_dot*sin(theta) - y_dot*cos(theta), 
			       			x_dot*cos(theta) + y_dot*sin(theta), 
			       				 theta_dot, 
			       (fs_vector[0] - fr_x*sin(dr)*self.rudder_break + tang_frict_force_x ) / self.mass, 
	                       (fs_vector[1] - fr_y*cos(dr)*self.rudder_break + tang_frict_force_y ) / self.mass , 
			  ( (self.d_mast_sail - self.d_mast_COG*cos(ds) ) *fs - self.d_rudder_COG*fr_x*cos(dr) - k)*(1/self.inertia) 	
																	] )
	return model, states_names, inputs_names 

if __name__ == '__main__':
    try:
        Models()
    except rospy.ROSInterruptException:
        pass
