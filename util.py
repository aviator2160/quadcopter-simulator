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