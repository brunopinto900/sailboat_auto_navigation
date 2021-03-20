#!/usr/bin/python

import rospy, time
from std_msgs.msg import Float32, Int16
from sailboat.msg import Vector3, Vector2, Wind
from nav_msgs.msg import Odometry
import math, mpmath
import numpy as np
from sympy import *
from Control.Extended_Kalman_Filter import ExtendedKalmanFilter as ekf
from Model.Model import Models as MODEL
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class state_estimator(object):

    def __init__(self):

	# Initializes the node
	rospy.init_node("state_estimator", anonymous=True)
	self.sample_time = rospy.get_param("simulation/sample_time") # rate at which this node runs
        self.node = rospy.Rate(self.sample_time) # wait function to use inside threads


	# Subscribing to all states and inputs
	rospy.Subscriber('heading', Float32, self.get_heading)
	self.heading = rospy.get_param("scenario/initial_state/heading")
	self.initial_heading = rospy.get_param("scenario/initial_state/heading")

	rospy.Subscriber('angular_velocity', Vector3, self.get_angular_velocity)
	self.heading_rate = rospy.get_param("scenario/initial_state/heading_rate")
	

	rospy.Subscriber('linear_velocity', Vector2, self.get_velocity)
	self.velocity = Vector2()

	rospy.Subscriber('position', Odometry, self.get_position)
	self.position = Odometry()
	# Read init position form the /wp parameters, fazer isto de outro maneira
        try:
            wp_list = rospy.get_param('gps/list')
            wp0 = wp_list[0]
        except KeyError:
            task_list = rospy.get_param('wp/tasks')
            wp0 = task_list[0]['waypoint']
        wp_table = rospy.get_param('gps/coordinates')
        self.initial_position = wp_table[wp0]


	rospy.Subscriber('rudder_angle', Int16, self.get_rudder_angle)
	self.rudder_angle = 0

	rospy.Subscriber('sail_angle', Int16, self.get_sail_angle)
	self.sail_angle = 0

	rospy.Subscriber('apparent_wind', Wind, self.get_wind)
	self.AW = Wind()

	self.set_estim_heading = rospy.Publisher('estim_heading', Float32, queue_size = 10)
	self.set_estim_vel = rospy.Publisher('estim_vel', Vector2, queue_size = 10)
	self.set_estim_heading_rate = rospy.Publisher('estim_heading_rate', Float32, queue_size = 10)
	self.set_estim_position = rospy.Publisher('estim_position', Odometry, queue_size = 10)

	self.model = MODEL()
	self.sailboatModel, self.states_names, self.inputs_names = self.model.buildModel()

	self.fig = plt.figure()
	self.ax1 = self.fig.add_subplot(1,1,1)
	self.predicted_heading = []
	self.true_heading = []
	self.meas_heading = []
	self.t = []
	self.ts = 0
	
	self.pos_covar = rospy.get_param("model/noise/position_covariance")


 	self.proc_covar = np.array ( [       [self.pos_covar, 0, 0, 0, 0, 0],  # covariance on x
                       		  [ 0, self.pos_covar, 0, 0, 0, 0],  # covariance on y
                        	  [0, 0, self.pos_covar, 0, 0, 0],  # covariance on theta
                        	  [0, 0, 0, 0.1, 0, 0],  # covariance on x_dot
                        	  [0, 0, 0, 0, 0.1, 0],  # covariance on y_dot
                        	  [0, 0, 0, 0, 0, 0.1]  # covariance on theta_dot
			  ] )

	self.meas_noise = np.array( [ [0.1], [0.1], [0.1] ] ) # Vector

	
	sensor_equations = np.array ( [ [1,0,0,0,0,0], [0,1,0,0,0,0], [0,0,1,0,0,0] ] )

	initial_state = np.array ( [ [ self.initial_position[0] ], [self.initial_position[1] ], [self.initial_heading], [0] , [0], [0] ] )
	
	initial_prediction = np.eye(6)

	self.EKF = ekf(self.sailboatModel, self.states_names, self.inputs_names, sensor_equations, 6 , 3, 4 ,self.proc_covar, self.meas_noise, initial_state, initial_prediction)

	self.estimate_states()


    def BuildModel(self):
	m = MODEL()
	return m.sailboatModel

    def get_heading(self,data):
	self.heading = data.data

    def get_angular_velocity(self,data):
        self.heading_rate = data.z

    def get_velocity(self,data):
	self.velocity = data

    def get_position(self,data):
	self.position = data

    def get_rudder_angle(self,data):
	self.rudder_angle = data.data

    def get_sail_angle(self,data):
	self.sail_angle = data.data


    def animate(self,i):
	
        self.ax1.clear()
        self.ax1.plot(self.t, self.meas_heading, label = 'Measurements')
	self.ax1.plot(self.t, self.predicted_heading, label = 'Kalman Filter Prediction')

    def get_wind(self,data):
	self.AW = data

    def estimate_states(self):

	while not rospy.is_shutdown():
	    measurements = np.array([ [self.position.pose.pose.position.x], [self.position.pose.pose.position.y] , [self.heading] ])
	    control_input = np.array( [ [self.sail_angle],[self.rudder_angle],[self.AW.angle],[self.AW.speed] ] )

	    estim_states_vector, dummy = self.EKF.estimate( measurements,control_input)

	    self.set_estim_heading.publish(estim_states_vector[2])
            #self.set_estim_position.publish( (estim_states_vector[0],estim_states_vector[1]) )


	    self.predicted_heading.append(estim_states_vector[2])
	    self.meas_heading.append(self.heading)
	    self.ts = self.ts + 0.1
    	    self.t.append(self.ts)

		
	    #ani = animation.FuncAnimation(self.fig, self.animate,interval=1000)

	    #plt.show()
	    #plt.legend()
	    self.node.sleep()
		


if __name__ == '__main__':
    try:
        state_estimator()
    except rospy.ROSInterruptException:
        pass










