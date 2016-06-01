# -*- coding: utf-8 -*-
"""
Created on Wed May 18 16:54:51 2016

@author: JOSE
"""

import numpy as np
import Clases_GoToGoal as gtg

PI = np.pi
u_pos = np.matrix([[0.3,0.4,0.5],[0,0,0],[1,1,1]])
x = 0.1
y = 0.5
theta = 0

R_u1 = gtg.obtener_matriz_rotacion(0.335/2, 0.15/2, PI/4)
R_u2 = gtg.obtener_matriz_rotacion(0.335/2, 0, 0)
R_u3 = gtg.obtener_matriz_rotacion(0.335/2, -0.15/2, -PI/4)
R_m = gtg.obtener_matriz_rotacion(x,y,theta)
    #se pasa a hacer las transformaciones y hallar los vectores desde tierra de 
    #las posiciones detectadas por los ultrasonidos
V_u1 = R_m*R_u1*u_pos[:,0]
V_u2 = R_m*R_u2*u_pos[:,1]
V_u3 = R_m*R_u3*u_pos[:,2]

V_u11 = V_u1[0:2,:]
V_u21 = V_u2[0:2,:]
V_u31 = V_u3[0:2,:]

pos_r = np.matrix([[x],[y]])
u1 = V_u11 - pos_r
u2 = V_u21 - pos_r
u3 = V_u31 - pos_r

U = (u1 + u2 + u3)

ang = np.arctan2(U[1:2,:],U[0:1,:])

print u_pos
print '------------'
print pos_r
print '------------'
print u1
print '------------'
print u2
print '------------'
print u3
print '------------'
print U
print '------------'
print ang
