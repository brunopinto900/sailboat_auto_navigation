

import numpy as np
import rospy


class KalmanFilter(object):


    # F - state transition model (sometimes is written as 'A' )
    # B - control input	
    # H - observation or sensor model;
    # Q - covariance of the process noise;
    # R - covariance of the observation or sensor noise;

    def __init__(self, F, B, H, Q, R, initial_state, initial_prediction):

        # Set all matrices (F, H, Q, R, B, x, y)
	self.set_state_transition_matrix(F)
	self.set_observation_model_matrix(H)
	self.set_process_covariance_matrix(Q)
	self.set_observation_covariance_matrix(R) 
	self.B = B

	self.states_vector = np.zeros( (self.n_states,1) ) # states vector X
	self.covariance_matrix = np.zeros( (self.n_states,self.n_states) ) # covariance vector P
        self.observations_matrix = np.zeros( (self.n_measurements, 1) ) # measurements vector Y
	
        self.initialize(initial_state,initial_prediction)
	

    def initialize(self,initial_state, initial_prediction):

	self.covariance_matrix = initial_prediction
	self.states_vector = initial_state



    def predict(self, _input): # input u at the time step k 
	
	predicted_state = np.dot(self.F, self.states_vector) + np.dot(self.B, _input) # Xkp = Fk * Xk-1 + Bk * uk
	tmp = np.dot(self.covariance_matrix, self.F.T) #  Pk-1 * Fk^T
	predicted_covariance = np.dot( self.F, tmp) + self.Q # Pkp = Fk * Pk-1 * Fk^T + Q

	return predicted_state, predicted_covariance	



    def update(self,state_predicted, covariance_predicted, measurements): 

	
	self.observations_matrix = measurements - np.dot(self.H, state_predicted) # measurement innovation = Yk - H * Xkp
	
	Pkp_Ht = np.dot(covariance_predicted, np.transpose(self.H) )  # Pkp * Hk^T
        Sk = np.matrix(self.R + np.dot(self.H, Pkp_Ht),dtype='float') # Sk = R + H * Pkp * Hk^T
	

	Sk_inverse = np.linalg.pinv(Sk) # S^-1
        Kalman_gain = np.dot(Pkp_Ht, Sk_inverse) # K

        state_estimate = state_predicted + np.dot(Kalman_gain, self.observations_matrix) # estimated X
	
	
        I = np.eye(self.n_states) # identity matrix

	
	I_minus_KH = I - np.dot(Kalman_gain, self.H)  # I - K*H
	
        covariance_estimate = np.dot(I_minus_KH, covariance_predicted) # estimated P

	return state_estimate, covariance_estimate



    def next_step(self, state_estimate, covariance_estimate):

	self.states_vector = state_estimate
	self.covariance_matrix = covariance_estimate



    def estimate(self, measurements, control_input):

	predicted_state, predicted_covariance = self.predict(control_input)
	
	state_estimate, covariance_estimate = self.update(predicted_state, predicted_covariance, measurements)
	
	self.next_step(state_estimate, covariance_estimate)

	return state_estimate, covariance_estimate


    def set_state_transition_matrix(self, F): #to use with extended kalman filter

	self.F = F
	
	self.n_states = self.F.shape[0]
	
	
	#dummy = len(F[0])
	#if (dummy != self.n_states):
	 #   raise ValueError("Wrong state transition matrix ' F ' dimensions. Should be a n states by n states matrix")


    def set_observation_model_matrix(self, H): #to use with extended kalman filter

	self.H = H
	self.n_measurements = self.H.shape[0]

	#dummy = len(H[0])
	#if (dummy != self.n_states):
	 #   raise ValueError("Wrong observation matrix ' H ' dimensions. Should be a m measurements by n states matrix")


    def set_process_covariance_matrix(self, Q): #to use with extended kalman filter

	self.Q = Q
	#dummy1 = len(Q)
	#dummy2 = len(Q[0])
	#if (dummy1 != dummy2 or dummy1 != self.n_states or dummy2 != self.n_states):
	 #   raise ValueError("Wrong process noise covariance matrix ' Q ' dimensions. Should be a n states by n states matrix")

	

    def set_observation_covariance_matrix(self, R): #to use with extended kalman filter

	self.R = R
	#dummy = len(R[0])
	#if (dummy != self.n_measurements):
	 #   raise ValueError("Wrong sensor noise covariance matrix ' R ' dimensions. Should be a m measurements by 1 matrix")
        	
