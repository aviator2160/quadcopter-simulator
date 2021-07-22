# -*- coding: utf-8 -*-
"""
Created on Thu Jul  1 15:12:50 2021

@author: aviat
"""
import util

import numpy as np
import scipy.integrate

# State space representation: [x y z x_dot y_dot z_dot theta phi gamma theta_dot phi_dot gamma_dot]
def state_dot(time, state, load):
    state_t = load.state
    state_dot = np.zeros(12)
    # The velocities(t+1 x_dots equal the t x_dots)
    state_dot[0] = state_t[3]
    state_dot[1] = state_t[4]
    state_dot[2] = state_t[5]
    # The acceleration
    x_dotdot = np.array([0,0,-load.g]) + load.inertial_force/load.mass
    state_dot[3] = x_dotdot[0]
    state_dot[4] = x_dotdot[1]
    state_dot[5] = x_dotdot[2]
    # The angular rates(t+1 theta_dots equal the t theta_dots)
    state_dot[6] = state_t[9]
    state_dot[7] = state_t[10]
    state_dot[8] = state_t[11]
    # The angular accelerations
    # Based on Wikipedia "Moment of inertia"
    #body_omega = np.dot(load.invR, state_t[9:12])
    #omega_dot = load.R @ load.invJ @ (load.body_moment - np.cross(body_omega, load.R @ load.J @ body_omega))
    omega = state_t[9:12]
    omega_dot = np.dot(load.invJ, (load.body_moment - np.cross(omega, np.dot(load.J, omega))))
    state_dot[9] = omega_dot[0]
    state_dot[10] = omega_dot[1]
    state_dot[11] = omega_dot[2]
    return state_dot

class Payload():
    ode = scipy.integrate.ode(state_dot).set_integrator('vode',nsteps=500,method='bdf')
    
    def __init__(self,defs,g=9.81):
        self.g = g
        self.state = np.zeros(12)
        self.state[0:3] = defs['position']
        self.state[6:9] = defs['orientation']
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
        self.R = util.rotation_matrix(self.state[6:9])
        self.invR = self.R.transpose()
        body_forces = np.dot(self.invR, self.applied_forces)
        body_force_moments = np.dot(self.force_geometry, body_forces.flatten(order='F'))
        self.inertial_force = np.dot(self.R, body_force_moments[0:3])
        self.body_moment = body_force_moments[3:6]
        Payload.ode.set_initial_value(self.state,0).set_f_params(self)
        self.state = Payload.ode.integrate(Payload.ode.t + dt)
        self.state[6:9] = util.wrap_angle(self.state[6:9])

    def get_position(self):
        return self.state[0:3]

    def get_linear_rate(self):
        return self.state[3:6]

    def get_orientation(self):
        return self.state[6:9]

    def get_angular_rate(self):
        return self.state[9:12]

    def get_state(self):
        return self.state

    def set_position(self,position):
        self.state[0:3] = position

    def set_orientation(self,orientation):
        self.state[6:9] = orientation
        
    def set_force_at(self,hardpoint_num,force):
        self.applied_forces[:,hardpoint_num] = force
    
    def get_hardpoint(self,hardpoint_num):
        R = util.rotation_matrix(self.state[6:9])
        return self.state[0:3] + np.dot(R,self.hardpoints[:,hardpoint_num])