#!/usr/bin/python
# Simulates the forces acting on the boat

import rospy
import time,math, mpmath
from sailboat.msg import Vector3, Vector2


class Forces_simu(object):
    def __init__(self):
	
        self.sail_force = Vector2()
	self.SAIL_LIFT_COEF = rospy.get_param("model/sail_lift_coefficient")

	self.rudder_force = Vector2()
	self.RUDDER_LIFT_COEF = rospy.get_param("model/rudder_lift_coefficient")

	

	
    def get_sail_force(self,sail_angle, apparent_wind_speed, apparent_wind_angle): # aerodynamic force in the fixed body frame
            
	angle_of_attack = (sail_angle - apparent_wind_angle) #here i can use a built in function to be bounded to 360 degrees
	sail_force_vector = self.SAIL_LIFT_COEF*apparent_wind_speed*math.sin(angle_of_attack)  
	
	self.sail_force.x = sail_force_vector*math.sin(sail_angle)
	self.sail_force.y = sail_force_vector*math.cos(sail_angle)

	
	
	return self.sail_force

	
    def get_rudder_force(self,rudder_angle,velocity): # hydrodynamic force in the fixed body frame
         
	self.rudder_force.x = self.RUDDER_LIFT_COEF*(mpmath.power(velocity.x,2))*math.sin(rudder_angle)  
	self.rudder_force.y = self.RUDDER_LIFT_COEF*(mpmath.power(velocity.y,2))*math.cos(rudder_angle)

	return self.rudder_force



if __name__ == '__main__':
    try:
        Forces_simu()
    except rospy.ROSInterruptException:
        pass
