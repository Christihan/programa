# -*- coding: utf-8 -*-
"""
Created on Tue May 31 11:12:30 2016

@author: JOSE
"""
byte1 = 0
byte2 = 0
MIP = [117, 101, 12, 13, 13, 8,1,3,4,0,10,5,0,10,0,14,0,10]

for i in MIP:
    byte1 += i
    byte2 += byte1
    
print byte1, byte2