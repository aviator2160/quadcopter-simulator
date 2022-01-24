# -*- coding: utf-8 -*-
"""General simulation utility functions.

Created on Wed Jul  7 16:52:43 2021

@author: aviat
"""

import numpy as np

def roll_matrix(theta):
    """Outputs the 3x3 matrix of an x-axis rotation by angle theta (radians)."""
    ct = np.cos(theta)
    st = np.sin(theta)
    return np.array([[1, 0,  0],
                     [0,ct,-st],
                     [0,st, ct]])


def pitch_matrix(phi):
    """Outputs the 3x3 matrix of a y-axis rotation by angle phi (radians)."""
    cp = np.cos(phi)
    sp = np.sin(phi)
    return np.array([[ cp,0,sp],
                     [  0,1, 0],
                     [-sp,0,cp]])


def yaw_matrix(gamma):
    """Outputs the 3x3 matrix of a z-axis rotation by angle gamma (radians)."""
    cg = np.cos(gamma)
    sg = np.sin(gamma)
    return np.array([[cg,-sg,0],
                     [sg, cg,0],
                     [ 0,  0,1]])


def rotation_matrix(angles):
    """Outputs the 3x3 rotation matrix for the given angles.
    
    Outputs the 3x3 rotation matrix for the given list of three
    Tait-Bryan angles: roll (x-axis), pitch (y-axis), yaw (z-axis).

    Args:
        angles: A tuple of the three Tait-Bryan angles defining the rotation:
            0:  Roll (x-axis) angle in radians
            1: Pitch (y-axis) angle in radians
            2:   Yaw (z-axis) angle in radians
    """
    R_x = roll_matrix(angles[0])
    R_y = pitch_matrix(angles[1])
    R_z = yaw_matrix(angles[2])
    return R_z @ R_y @ R_x

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

def body_omega_to_euler_rates_matrix(angles):
    ct = np.cos(angles[0])
    st = np.sin(angles[0])
    cp = np.cos(angles[1])
    tp = np.tan(angles[1])
    return np.array([[1,st * tp,ct * tp],
                     [0,     ct,    -st],
                     [0,st / cp,ct / cp]])

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
    