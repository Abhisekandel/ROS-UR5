#!/usr/bin/env python
# -*- coding: utf-8 -*-


from sympy import *
import numpy as np
from math import pi as pi
phi,psi,theta =symbols('phi,psi,theta')

def Rotational(phi_v,theta_v,psi_v,xr,yr,zr):
    #conversion from degrees to radians
    theta_v = theta_v*(pi/180)
    phi_v = phi_v*(pi/180)
    psi_v = psi_v*(pi/180)
    
    if xr==1:
    
        Rx =Matrix([[1,0,0],[0,cos(phi),sin(phi)],[0,-sin(phi),cos(phi)]])
    
    else:
        Rx =1
        
    if yr ==1:
        
        Ry = Matrix([[cos(theta),0,-sin(theta)],[0,1,0],[sin(theta),0,cos(theta)]])
    else:
        Ry =1
        
    if zr ==1:
        Rz = Matrix([[cos(psi),sin(psi),0],[-sin(psi),cos(psi),0],[0,0,1]])
    else: 
        Rz = 1
    
    R=Rz*Ry*Rx
    
    R=simplify(R)
    R_sym = R.evalf(subs={theta:theta_v,phi:phi_v,psi:psi_v})
    R_sym = np.array(R_sym).astype(np.float64)
    return R,R_sym

def DH(R,alpha,d):
     mat= np.array([[alpha          ],
                    [-sin(alpha-1)*d],
                    [cos(alpha-1)*d],
                    [1             ]])
     
     T = np.zeros((4,4))
     T[0:3,0:3] = R
     T[:,3] = mat.transpose()
     return T
 
    
 #Ur5 robot parameters
axis_of_rotation =np.array([[0,0,1],
                             [0,1,0],
                             [0,1,0],
                             [0,1,0],
                             [0,0,1],
                             [0,1,0]])
 
 #DH parameters
angles = np.array([[0,0,pi/2],
                    [0,0,0],
                    [0,0,0],
                    [0,pi/2,0],
                    [0,0,-pi/2],
                    [0,0,0]])
                    
                    
a_list=np.array([0,-0.425,-0.39225,0,0,0])
d_list=np.array([0.08920,0,0,0.189,0,0,923,0.823])
#Random values for generating rotation matrix
ran_angles = axis_of_rotation*90
#Rotation matrix from base to the end effector

R0_1,R0_1_Sym=Rotational(ran_angles[0,0],ran_angles[0,1],ran_angles[0,2],axis_of_rotation[0,0],axis_of_rotation[0,1],axis_of_rotation[0,2])
R1_2,R1_2_Sym=Rotational(ran_angles[1,0],ran_angles[1,1],ran_angles[1,2],axis_of_rotation[1,0],axis_of_rotation[1,1],axis_of_rotation[1,2])
R2_3,R2_3_Sym=Rotational(ran_angles[2,0],ran_angles[2,1],ran_angles[2,2],axis_of_rotation[2,0],axis_of_rotation[2,1],axis_of_rotation[2,2])
R3_4,R3_4_Sym=Rotational(ran_angles[3,0],ran_angles[3,1],ran_angles[3,2],axis_of_rotation[3,0],axis_of_rotation[3,1],axis_of_rotation[3,2])
R4_5,R4_5_Sym=Rotational(ran_angles[4,0],ran_angles[4,1],ran_angles[4,2],axis_of_rotation[4,0],axis_of_rotation[4,1],axis_of_rotation[4,2])
R5_6,R5_6_Sym=Rotational(ran_angles[5,0],ran_angles[5,1],ran_angles[5,2],axis_of_rotation[5,0],axis_of_rotation[5,1],axis_of_rotation[5,2])

R = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6
R=simplify(R)
r_sym = R0_1_Sym*R1_2_Sym*R2_3_Sym*R3_4_Sym*R4_5_Sym*R5_6_Sym

#Transforamtion matrix
T0_1 =DH(R0_1_Sym,a_list[0],d_list[0])
T1_2 =DH(R1_2_Sym,a_list[1],d_list[1])
T2_3 =DH(R2_3_Sym,a_list[2],d_list[2])
T3_4 =DH(R3_4_Sym,a_list[3],d_list[3])
T4_5 =DH(R4_5_Sym,a_list[4],d_list[4])
T5_6 =DH(R5_6_Sym,a_list[5],d_list[5])

T0_6 = T0_1*T1_2 *T2_3*T3_4*T4_5*T5_6