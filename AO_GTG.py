# -*- coding: utf-8 -*-
"""
Created on Wed May 25 18:21:20 2016

@author: JOSE
"""

import vrep
import sys
import time
import numpy as np
import Clases_GoToGoal as gtg


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
#Creacion de handles para los ultrazonidos y primera lectura del valor
errorCode,ult_handle1=vrep.simxGetObjectHandle(clientID,'ultrasonido1',vrep.simx_opmode_oneshot_wait)
errorCode,detectionState,detectedPoint1,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,ult_handle1,vrep.simx_opmode_streaming)   
u1_val = np.linalg.norm(detectedPoint1)

errorCode,ult_handle2=vrep.simxGetObjectHandle(clientID,'ultrasonido2',vrep.simx_opmode_oneshot_wait)
errorCode,detectionState,detectedPoint2,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,ult_handle2,vrep.simx_opmode_streaming)
u2_val = np.linalg.norm(detectedPoint2)

errorCode,ult_handle3=vrep.simxGetObjectHandle(clientID,'ultrasonido3',vrep.simx_opmode_oneshot_wait)
errorCode,detectionState,detectedPoint3,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,ult_handle3,vrep.simx_opmode_streaming)
u3_val = np.linalg.norm(detectedPoint3)

#Avoid Obstacle algorithm simulation
#-------------------------------------------------------------------------#
#Initialize the robot class
movil = gtg.Robot()
#Initialize the sensor class
sensor = gtg.Sensor()
sensor.u1 = u1_val
sensor.u2 = u3_val
sensor.u3 = u2_val
#Initialize a start position
movil.setPose(0,0,0)
#Initialize the desired position
d_position = [1,1]
error = [0,0]
old_state = 1
current_state = 1
#time.sleep(3)

#Se asigna una vleocidad inicial
vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)  

while np.sqrt((movil.x - d_position[0])**2 + (movil.y - d_position[1])**2) > 0.1:
    
    errorCode,detectionState,detectedPoint1,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,ult_handle1,vrep.simx_opmode_buffer)  
    sensor.u1 = 1.1*np.linalg.norm(detectedPoint1)
    errorCode,detectionState,detectedPoint2,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,ult_handle2,vrep.simx_opmode_buffer)
    sensor.u2 = 1.1*np.linalg.norm(detectedPoint2)
    errorCode,detectionState,detectedPoint3,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,ult_handle3,vrep.simx_opmode_buffer)
    sensor.u3 = 1.1*np.linalg.norm(detectedPoint3)
    
    #Se crea la condicion de cambio de estado, 
    if ((sensor.u1 <= 0.05) or (sensor.u2 <= 0.05) or (sensor.u3 <= 0.05)):      
        old_state = current_state
        current_state = 2
        print 'Cambio estado evasion'
    else:
        old_state = current_state
        currrent_state = 1
        print 'Cambio estado go'
    #Se verifica si elestado anterior es distinto al actual, si esto pasa se reiniccializa el error   
    if current_state != old_state:
        error = [0,0]
    
    #Se realiza el cambio de estado 
    if current_state == 1:
        y = gtg.GoToGoal(movil, d_position, error)
    elif current_state == 2:
        y = gtg.AvoidObstacle(movil,sensor ,error)
        
    vr = (2*y[0] + y[1]*movil.longitud)/(2*movil.radio)
    vl = (2*y[0] - y[1]*movil.longitud)/(2*movil.radio) 
        
    #Update error
    error = [y[2],y[3]]
    #Calculating the new position of the robot
    dR = vr*movil.radio
    dL = vl*movil.radio
    
    dC = (dR + dL)/2
    phi = (dR - dL)/movil.longitud
    
    vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,vl, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,vr, vrep.simx_opmode_streaming)
    
    x_new = movil.x + dC*np.cos(movil.theta)
    y_new = movil.y + dC*np.sin(movil.theta)
    theta_new = movil.theta + phi
    
    if current_state == 1:
        movil.setPose(x_new,y_new,theta_new)
    elif current_state == 2:
        movil.setPose(x_new[0,0],y_new[0,0],theta_new[0,0])
        
    time.sleep(0.2)
    
vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)      
    