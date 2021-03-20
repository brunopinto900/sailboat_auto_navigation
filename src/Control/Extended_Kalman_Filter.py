
from sympy import *
import numpy as np
import rospy
from Control.Kalman_Filter import KalmanFilter as kf


class ExtendedKalmanFilter(object):

    def __init__(self, state_space_equations, states_names, input_names, sensor_equations, n_states,n_measurements, n_inputs, Q, R, initial_state, initial_prediction):

	self.state_model = state_space_equations
	self.sensor_model = sensor_equations
	self.states_names = states_names
	self.input_names = input_names
	self.F = np.eye(n_states)
	self.H = np.zeros( (n_measurements,n_states) )
	self.B = np.zeros( (n_states,n_inputs) )
	self.Kalman_Filter = kf(self.F, self.B, self.H, Q, R, initial_state, initial_prediction)
	
    

    def linearaize(self,function, states, inputs, states_vector, control_input):
	
	jacobian = function.jacobian(states)
	states_vector = np.array(states_vector)
	control_input = np.array(control_input)
	
	tmp = jacobian.subs( zip(states,states_vector*1.0) )
	res = tmp.subs( zip(inputs,control_input*1.0) )
	
	return tmp.subs( zip(inputs,control_input*1.0) )


    def estimate(self, measurements, control_input):
	
	# Linearize f(x,u)	
	self.F = self.linearaize(self.state_model, self.states_names, self.input_names, self.Kalman_Filter.states_vector, control_input)
	self.Kalman_Filter.set_state_transition_matrix(self.F)
	
	# Predict
        predicted_state, predicted_covar = self.Kalman_Filter.predict(control_input)

	# Linearize h(xkp)
	#self.H = self.linearaize(self.sensor_model,self.states_names, predicted_state, 0)
	self.H = self.sensor_model
	self.Kalman_Filter.set_observation_model_matrix(self.H)

	# Update
	estimated_state, estimated_covar = self.Kalman_Filter.update(predicted_state, predicted_covar, measurements)

	return estimated_state, estimated_covar

