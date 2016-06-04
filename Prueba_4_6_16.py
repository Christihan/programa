# -*- coding: utf-8 -*-
"""
Created on Wed Jun 01 09:03:29 2016

@author: JOSE
"""

#Programa de prueba de funcionamiento integrado numero 1
import time
import serial
import numpy as np
import Clases_GoToGoal as gtg

#Inicializar el serial de la beagle

ser = serial.Serial('/dev/ttyACM1',9600)

PI = np.pi
#Se establecen condiciones de velocidad
V_max_RPM = 500
V_max_rad = V_max_RPM*2*PI/60

#Inicializa la clase robot
movil = gtg.Robot()
#Inicializa la posicion del robot
movil.setPose(0,0,0)
#Inicializa el valor de los sensores
sensor = gtg.Sensor()
#Se inicializa los valores delsensor
sensor.setSensor(0,0,0,0,0)
#Se inicializa los errores del PID
error = [0,0]

#######################################################
#####  PASO1 : DETECCION DE MAQUETA CON LIDAR #########
#######################################################

pos = [1,1]   #Función de Emi
d_position = pos

#######################################################
#####  PASO2 : MOVIMIENTO HACIA LA POSICION   #########
#######################################################
while (np.sqrt((movil.x - d_position[0])**2 + (movil.y - d_position[1])**2) > 0.05):
    
    ###################################################
    #####  PASO3 : RECEPCION DE DATOS DEL ARDUINO #####
    ###################################################
    
    #Funcion para recibir los datos de los encoders
    lect_sensores = gtg.leer_sensores(ser)
    sensor.uD = lect_sensores[1] #salida de la funcion
    sensor.uL = lect_sensores[0] #Salida de la función
    
    gtg.Odometria(movil,sensor)
    vel = 0.25
    y = gtg.GoToGoal(movil,d_position,error,vel)
        
    #The output v contains y = [v w EI Eant], with this values we calculate the velocity of each wheel
    vr = (2*y[0] + y[1]*movil.longitud)/(2*movil.radio)
    vl = (2*y[0] - y[1]*movil.longitud)/(2*movil.radio)
    #Update error
    error = [y[2],y[3]]
    #Velocidades a enviar por el arduino    
    vel_ard = gtg.Calc_Potencia(V_max_rad,vr,vl)
    print vr,vl
    print '-------------'
    print vel_ard
    time.sleep(0.2)

    ###################################################
    #######  PASO4 : ENVIO DE DATOS AL ARDUINO  #######
    ###################################################

    gtg.Enviar_Datos(ser,vel_ard[0],vel_ard[1],vel_ard[2])

#Funcion de christihan con velocidades 0
gtg.Enviar_Datos(ser,'S',,)
