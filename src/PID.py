import time
import math
import numpy as np
import rospy

class PID(object):

        def __init__(self, Kp, Ki, Kd, maxI, minI, maxOut,minOut,Direction, maxSpeed,minSpeed, frequency):

	    self.Direction = Direction
            self.setGains(Kp,Ki,Kd)
	    self.SetPoint = 0
	    self.Auto = True
	    self.limit_iTerm(maxI,minI)
	    self.limitOutput(maxOut,minOut)
	    self.limitActuatorSpeed(maxSpeed,minSpeed)	
	    self.reset()
	    #self.control_disturbance = rospy.get_param("simulation/rudder/disturbance")
	    self.previous_out = 0
	    self.current_out = 0
	    self.frequency = rospy.get_param("controller/cuttof_frequency")
	
        def LowPassFilter(self, signal, delta_time, frequency):

   	    a = (2*math.pi*delta_time*frequency) / (2*math.pi*delta_time*frequency + 1)
	    self.current_out = a*signal + (1-a)*self.previous_out
	    self.previous_out = self.current_out 
	    return self.current_out
	
        def reset(self):
      
            self.pTerm = 0.0
            self.dTerm = 0.0
            self.iTerm = 0.0 
            self.Output = 0.0 
            self.lastTime = 0.0
	    self.lastError = 0.0
	    self.lastInput = 0.0
	    self.lastOutput = 0.0 

        def checkSaturation(self,signal,maxV,minV):

            if signal > maxV:
                return maxV
            elif signal < minV:
                return minV
            else:
                return signal


        def Reinitialize(self):

	    self.lastInput = self.Input
	    self.ITerm = self.Output
	    self.ITerm = self.checkSaturation(self.Iterm, maxIntegral, minIntegral)
	 
    
	def SetOperationMode(self,Mode):

	    if(Mode):
	        if(self.Auto == False):
	            Reinitialize() # manual to auto	

            self.Auto = Mode


        def setGains(self, Kp, Ki, Kd):

            self.Kp = Kp
            self.Ki = Ki
            self.Kd = Kd

	    if(self.Direction < 0):
	        self.Kp = -Kp
	        self.Ki = -Ki
 	        self.Kd = -Kd	
       

        def limit_iTerm(self, maxI, minI):
	
	    self.maxI = maxI
            self.minI = minI


        def limitOutput(self,maxOut,minOut):

	    self.OutputMax = maxOut
	    self.OutputMin = minOut
   
        def limitActuatorSpeed(self,maxOut,minOut):

	    self.maxSpeed = maxOut
	    self.minSpeed = minOut

    
        def calculate(self,SetPoint,Input):

            if( self.Auto == False): return 0 # Manual Mode

	    currTime = time.time()
            dt = currTime - self.lastTime
	    self.lastTime = currTime

	    #Calculate error
	    self.SetPoint = SetPoint
	    error = self.SetPoint - Input
            
            # Calculate the proportional term
            self.pTerm = self.Kp*error

        # Calculate the integral term
            self.iTerm += self.Ki*error*dt
	    self.iTerm = self.checkSaturation(self.iTerm,self.maxI,self.minI)

        # Calculate the derivative term
	    self.dInput = self.LowPassFilter(Input, dt, self.frequency)
            self.dTerm = self.Kd * ( (self.dInput - self.lastInput) / dt )

            self.lastError = error
	    self.lastInput = self.dInput
	

            self.Output = self.pTerm + self.iTerm - self.dTerm
	    self.Output = self.checkSaturation(self.Output,self.OutputMax, self.OutputMin)

            return self.Output


        def calculate(self,error,Input, errorSet): #if errorSet > 0, then the error is provided by the user

	    if( self.Auto == False): return 0 # Manual Mode

	    currTime = time.time()
            dt = currTime - self.lastTime
	    self.lastTime = currTime

            # Calculate the proportional term
            self.pTerm = self.Kp*error

            # Calculate the integral term
            self.iTerm += error*dt
	    self.iTerm = self.checkSaturation(self.iTerm,self.maxI,self.minI)

            # Calculate the derivative term
	    self.dInput = self.LowPassFilter(Input, dt, self.frequency)
            self.dTerm = self.Kd * ( (self.dInput - self.lastInput) / dt )

            self.lastError = error
	    self.lastInput = self.dInput

            self.Output = self.pTerm + self.Ki*self.iTerm - self.dTerm
	    self.Output = self.checkSaturation(self.Output,self.OutputMax, self.OutputMin)

            return self.Output

