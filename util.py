# -*- coding: utf-8 -*-
"""
Created on Wed Jul  7 16:52:43 2021

@author: aviat

General simulation utility functions
"""
import numpy as np

def rotation_matrix(angles):
    ct = np.cos(angles[0])
    cp = np.cos(angles[1])
    cg = np.cos(angles[2])
    st = np.sin(angles[0])
    sp = np.sin(angles[1])
    sg = np.sin(angles[2])
    R_x = np.array([[1,0,0],[0,ct,-st],[0,st,ct]])
    R_y = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    R_z = np.array([[cg,-sg,0],[sg,cg,0],[0,0,1]])
    R = np.dot(R_z, np.dot( R_y, R_x ))
    return R

def wrap_angle(angle):
    return((angle + np.pi) % (2 * np.pi ) - np.pi)

"""
Returns list of matrices based on the given matrix of moment arms (column
 vectors). 
Multiplication of a force vector by any of these matrices results in a 6-vector
 of the force and moment generated at that arm.
"""
# Adapted from Sarah Li, https://github.com/lisarah/geometric/blob/master/slung/slung.py
def force_geometry(moment_arms):
    force_geometry_list = []
    for j in range(moment_arms.shape[1]):
        arm = moment_arms[:,j]
        fm_i = np.zeros((6, 3))
        fm_i[0:3, 0:3] = np.eye(3)
        fm_i[3,1] = -arm[2] # -arm_z
        fm_i[3,2] =  arm[1] #  arm_y
        fm_i[4,0] =  arm[2] #  arm_z
        fm_i[4,2] = -arm[0] # -arm_x
        fm_i[5,0] = -arm[1] # -arm_y
        fm_i[5,1] =  arm[0] #  arm_x
        force_geometry_list.append(fm_i)
    return np.concatenate(force_geometry_list, axis=1)
    