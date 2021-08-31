# -*- coding: utf-8 -*-
"""
Created on Thu Jul  1 15:12:50 2021

@author: aviat
"""
import util

import numpy as np
import scipy.integrate

POS = slice(0,3)
VEL = slice(3,6)
EUL = slice(6,9)
OMG = slice(9,12)

# State space representation: [x y z x_dot y_dot z_dot theta phi gamma theta_dot phi_dot gamma_dot]
def state_dot(time, state, load):
    state_dot = np.zeros(12)
    # Velocity
    state_dot[POS] = state[VEL]
    # Acceleration
    x_dotdot = np.array([0,0,-load.g]) + load.inertial_force/load.mass
    state_dot[VEL] = x_dotdot
    # Euler rates
    state_dot[EUL] = util.body_omega_to_euler_rates_matrix(state[EUL]) @ state[OMG]
    # Angular acceleration
    omega = state[OMG]
    omega_dot = load.invJ @ (load.body_moment - np.cross(omega, load.J @ omega))
    state_dot[OMG] = omega_dot
    return state_dot

class Payload():
    
    def __init__(self,defs,g=9.81):
        self.g = g
        self.state = np.zeros(12)
        self.state[POS] = defs['position']
        self.state[EUL] = defs['orientation']
        self.x = defs['x']
        self.y = defs['y']
        self.z = defs['z']
        self.mass = defs['mass']
        self.hardpoints = np.array(defs['hardpoints']).transpose()
        self.applied_forces = np.zeros_like(self.hardpoints)
        self.force_geometry = util.force_geometry(self.hardpoints) # concatenate before use as single array
        # From Wikipedia "List of moments of inertia"
        ixx = self.mass*(self.x**2 + self.y**2)/12
        iyy = self.mass*(self.x**2 + self.z**2)/12
        izz = self.mass*(self.y**2 + self.y**2)/12
        self.J = np.array([[ixx,0,0],[0,iyy,0],[0,0,izz]])
        self.invJ = np.linalg.inv(self.J)
        
    def update(self, dt):
        self.R = util.rotation_matrix(self.state[EUL])
        self.invR = self.R.transpose()
        body_forces = np.dot(self.invR, self.applied_forces)
        body_force_moments = np.dot(self.force_geometry, body_forces.flatten(order='F'))
        self.inertial_force = np.dot(self.R, body_force_moments[0:3])
        self.body_moment = body_force_moments[3:6]
        ivp_solution = scipy.integrate.solve_ivp(state_dot,(0,dt),self.state,args=(self,),t_eval=[dt])
        self.state = ivp_solution.y[:,0]
        self.state[EUL] = util.wrap_angle(self.state[EUL])

    def get_position(self):
        return self.state[POS]

    def get_linear_rate(self):
        return self.state[VEL]

    def get_orientation(self):
        return self.state[EUL]

    def get_angular_rate(self):
        return self.state[OMG]

    def get_state(self):
        return self.state

    def set_position(self,position):
        self.state[POS] = position

    def set_orientation(self,orientation):
        self.state[EUL] = orientation
        
    def set_force_at(self,hardpoint_num,force):
        self.applied_forces[:,hardpoint_num] = force
    
    def get_hardpoint(self,hardpoint_num):
        R = util.rotation_matrix(self.state[EUL])
        return self.state[POS] + np.dot(R,self.hardpoints[:,hardpoint_num])