# -*- coding: utf-8 -*-
"""
Editor de Spyder

Este es un archivo temporal
"""
import numpy as np
PI = np.pi

def obtener_matriz_rotacion(x, y, theta):
    #Se calcula la matriz R
    R = np.matrix([[np.cos(theta), -np.sin(theta), x],[np.sin(theta), np.cos(theta), y],[0,0,1]])
    return R

R_u1 = obtener_matriz_rotacion(0.335/2, 0.15/2, PI/4)
R_u2 = obtener_matriz_rotacion(0.335/2, 0, 0)
R_u3 = obtener_matriz_rotacion(0.335/2, -0.15/2, -PI/4)

class Robot():
    
    #Se inicializa los valores geometricos del movil como el radio y distancia 
    #entre ruedas ambas en metros 
    radio = 0.06
    longitud = 0.28
    #Hacemos una matriz donde este ñlas posicion x, y, theta
    loc_sensor = np.matrix([[0.335/2,0.335/2,0.335/2],[0.15/2,0,-0.15/2],[PI/4,0,-PI/4]])
    #Aqui se define la funcion que permite setear la posicion del movil
    def setPose (self, x, y, theta):
        self.x = x
        self.y= y
        self.theta = theta
    #Permite la obtencion de la posicion del movil
    def getPose(self):
        x = self.x
        y = self.y
        theta = self.theta
        
        return [x, y, theta]

class Sensor():
    #Setear los sensores leidos
    def setSensor(self, encoderD, encoderI, ult1, ult2, ult3):
        self.ticksD = encoderD
        self.ticksL = encoderI
        self.u1 = ult1
        self.u2 = ult2
        self.u3 = ult3
        
    def getSensor(self):
        tD = self.ticksD 
        tI = self.ticksI
        u11 = self.u1
        u21 = self.u2
        u31 = self.u3
        
        return [tD, tI, u11, u21, u31]
        
def GoToGoal(robot, pos_des, errores,vel):

    #Leemos los Errores
    E_k = errores[0]
    e_k_1 = errores[1]
    #Inicializamos las constantes del PID
    K_p = 1
    K_i = 0.001
    K_d = 0.001
    dt = 0.2
    v = vel
    #Se leeran las posiciones deseadas alas cuales se quiere dirigir
    x_g = pos_des[0]
    y_g = pos_des[1]
    #Se obtiene la posicion actual del robot
    x = robot.x
    y = robot.y
    theta = robot.theta
    #Se va a analizar el error del go to goal
    u_x = x_g - x
    u_y = y_g - y
    #Se calcula el angulo al cual el angulo se debe dirigir
    theta_g = np.arctan2(u_y,u_x)
    #Se calcula el error
    e_k = theta_g - theta
    e_k = np.arctan2(np.sin(e_k),np.cos(e_k))
    #Se calculan los errores 
    e_P = e_k
    e_I = E_k + e_k
    e_D = (e_k - e_k_1)/dt
    #calculamos los PID
    w = K_p*e_P + K_i*e_I + K_d*e_D
    
    return [v, w, e_I, e_k]
    
def AvoidObstacle(robot, sensor, errores):
    #Leemos los Errores
    E_k = errores[0]
    e_k_1 = errores[1]
    #Inicializamos las constantes del PID
    K_p = 0.035
    K_i = 0.01
    K_d = 0
    dt = 1
    v = 0.1
    #Se guradan los datos del robot en variables
    pos_r = np.matrix([[robot.x],[robot.y]])
    theta_r = robot.theta    
    #Se crea una matriz con la distancia sensada en los ultrasonidos
    u_pos = np.matrix([[sensor.u1,sensor.u2,sensor.u3],[0,0,0],[1,1,1]])
    #se crean 4 matrices de rotación, 3 para los sensores respecto al robot y
    # 1 para la posicion del robot
    R_m = obtener_matriz_rotacion(robot.x,robot.y,robot.theta)
    #se pasa a hacer las transformaciones y hallar los vectores desde tierra de 
    #las posiciones detectadas por los ultrasonidos
    V_u1 = R_m*R_u1*u_pos[:,0]
    V_u2 = R_m*R_u2*u_pos[:,1]
    V_u3 = R_m*R_u3*u_pos[:,2]
    #Se hallan la resta de vectores vector sensor con la posicion del robot y se suman
    vect_1 = -(V_u1[0:2,:] - pos_r)
    vect_2 = -(V_u2[0:2,:] - pos_r)
    vect_3 = -(V_u3[0:2,:] - pos_r)

    #Hallo la suma de vectores sin poner peso a cada uno
    vect_t = vect_1 + vect_2 + vect_3
    
    theta_g = np.arctan2(vect_t[1:2,:], vect_t[0:1,:])
    #theta_g = 
    e_k = theta_g - theta_r
    e_k = np.arctan2(np.sin(e_k),np.cos(e_k))
    #Se calculan los errores 
    e_P = e_k
    e_I = E_k + e_k
    e_D = (e_k - e_k_1)/dt
    #calculamos los PID
    w = K_p*e_P + K_i*e_I + K_d*e_D
    return [v, w, e_I, e_k]
    
def Odometria (robot, sensor, prev_ticks):
    
    R_ticks = 260       #Resolucion del encoder    
    
    prev_tD = prev_ticks[0]
    prev_tI = prev_ticks[1]    
    
    tD =  sensor.ticksD
    tI =  sensor.ticksL
    
    dR = 2*PI*robot.radio*(prev_tD - tD)/R_ticks
    dL = 2*PI*robot.radio*(prev_tI- tI)/R_ticks
    
    dC = (dR + dL)/2
    phi = (dR - dL)/robot.longitud
    
    x_new = robot.x + dC*np.cos(robot.theta)
    y_new = robot.y + dC*np.sin(robot.theta)
    theta_new = robot.theta + phi
    
    robot.setPose(x_new,y_new,theta_new)
    
    return [tD, tI]
    
def Calc_Potencia(v_max, u_r, u_l):
    #Se ve el sentido de giro del movil
    if (u_r > 0) and (u_l > 0):
        sentido = 'F'
    elif (u_r < 0) and (u_l < 0):
        sentido = 'B'
    elif (u_r < 0) and (u_l > 0):
        sentido = 'L'
    elif (u_r > 0) and (u_l < 0):
        sentido = 'R'
    #Se calcula la potencia con el valor maximo del 
    ur = np.abs(u_r)
    ul = np.abs(u_l)
    Pot_D = int((ur/v_max)*100)
    Pot_I = int((ul/v_max)*100)
    
    return [sentido, Pot_D, Pot_I]
    
def Enviar_Datos(ser,sentido, pot_r, pot_i):
    ser.write("$P"+str(sentido)+str(pot_r)+str(pot_i)+"#\n")
    
# Inicalizamos la posicion inicial del robot

#r = Robot()
#r.setPose(0,0,0)
#deseado = [2,1]
#errores = [0,0]
#y = GoToGoal(r,deseado,errores)
#errores = [y[2],y[3]]
#
#print errores