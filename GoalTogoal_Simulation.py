"""
Created on Fri May 06 09:50:10 2016

@author: JOSE
"""

import vrep
import sys
import time
import numpy as np
import Clases_GoToGoal as gtg

PI = np.pi

vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP

if clientID!=-1:
    print ('Connected to remote API server')
    
else:
    print ('Failing connecting remote API')
    sys.exit('Could not connect')

#Handle creation for using the V-REP simulator
errorCode,left_motor_handle=vrep.simxGetObjectHandle(clientID,'motor_rueda_der',vrep.simx_opmode_oneshot_wait)
errorCode,right_motor_handle=vrep.simxGetObjectHandle(clientID,'motor_rueda_izq',vrep.simx_opmode_oneshot_wait)

#GoToGoal algorithm simulation
#-------------------------------------------------------------------------#
#Initialize the robot class
movil = gtg.Robot()
#Initialize a start position
movil.setPose(0,0,0)
#Initialize the desired position
d_position = [1,0]
error = [0,0]
#time.sleep(3)

while np.sqrt((movil.x - d_position[0])**2 + (movil.y - d_position[1])**2) > 0.05:

#    theta_g = np.arctan2((d_position[1] - movil.y),(d_position[0] - movil.x))    
##    
#    if ((np.sqrt((d_position[0] - movil.x)^2 + (d_position[1] - movil.y)^2) < 0.35) and (np.abs(theta_g - movil.theta)>np.pi)):
#        vel = 0        
#        y = gtg.GoToGoal(movil,d_position,error,vel)
   
    vel = 0.1
    y = gtg.GoToGoal(movil,d_position,error,vel)
        
    #The output v contains y = [v w EI Eant], with this values we calculate the velocity of each wheel
    vr = (2*y[0] + y[1]*movil.longitud)/(2*movil.radio)
    vl = (2*y[0] - y[1]*movil.longitud)/(2*movil.radio)
    
    #Update error
    error = [y[2],y[3]]
    #Calculating the new position of the robot
    dR = vr*movil.radio
    dL = vl*movil.radio
    
    dC = (dR + dL)/2
    phi = (dR - dL)/movil.longitud;
    
    vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,vl, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,vr, vrep.simx_opmode_streaming)
    
    x_new = movil.x + dC*np.cos(movil.theta)
    y_new = movil.y + dC*np.sin(movil.theta)
    theta_new = movil.theta + phi
    movil.setPose(x_new,y_new,theta_new)
    print vl, vr
    time.sleep(0.2)
    
vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)  
 