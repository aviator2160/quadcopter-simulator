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

"""
Given an input 3-vector A, returns the 3x3 matrix such that left-multiplying
any other 3-vector B by it is equivalent to taking the cross product A x B.
"""
def cross_matrix(vector):
    cross = np.zeros([3,3])
    cross[0,1] = -vector[2] # -vector_z
    cross[0,2] =  vector[1] #  vector_y
    cross[1,0] =  vector[2] #  vector_z
    cross[1,2] = -vector[0] # -vector_x
    cross[2,0] = -vector[1] # -vector_y
    cross[2,1] =  vector[0] #  vector_x
    return cross

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
        fm_i[0:3,:] = np.eye(3)
        fm_i[3:6,:] = cross_matrix(arm)
        force_geometry_list.append(fm_i)
    return np.concatenate(force_geometry_list, axis=1)
    