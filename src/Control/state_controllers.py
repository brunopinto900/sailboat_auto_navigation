import time
import math
import numpy as np
import rospy
from Control.PID import PID as pid_class

class rudder_controller(object):

    def __init__(self, Kp, Ki, Kd, maxI, minI, maxOut,minOut,Direction, maxSpeed,minSpeed, frequency):
    
        self.PID = pid_class(Kp, Ki, Kd, maxI, minI, maxOut,minOut,Direction, maxSpeed,minSpeed, frequency)


class sail_controller(object):


    def __init__(self,max_angle,min_angle):
	self.max_sail_angle = max_angle
	self.min_sail_angle = min_angle


    def calculate_sail_angle(self,wind_angle):

        return self.max_sail_angle - self.max_sail_angle * abs(wind_angle)/180

