# -*- coding: utf-8 -*-
"""
Created on Mon Sep 13 20:17:12 2021

@author: aviat
"""

import util
import disturbance_decouple as dd
import subspace

import numpy as np
import scipy.linalg as sla

POS = slice(0,3)
VEL = slice(3,6)
EUL = slice(6,9)
OMG = slice(9,12)

def Planner(quads, loads, cables, params):
    Specific_Planner = planner_defs.get(params['type'])
    if Specific_Planner != None:
        return Specific_Planner(quads, loads, cables, params)
    raise PlannerTypeNotFoundError(str(params['type']) + " is not a recognized type of planner!")

class Controller_LQR_Team():
    
    def __init__(self, quads, loads, cables, params):
        self.quads = quads
        self.loads = loads
        self.objs = quads + loads
        self.cables = cables
        self.x_slice = {obj.id : slice(12 * i,12 * (i + 1)) for i,obj in enumerate(self.objs)}
        self.u_slice = {quad.id : slice(4 * i,4 * (i + 1)) for i,quad in enumerate(self.quads)}
        self.goal = params['goal']
        quad_Q = params['quad_Q']
        load_Q = params.get('load_Q')
        block_quad_Q = np.kron(quad_Q,np.eye(len(self.quads)))
        if len(self.loads) > 0:
            block_load_Q = np.kron(load_Q,np.eye(len(self.loads)))
            self.Q = sla.block_diag(block_quad_Q,block_load_Q) # e.g. for 4 quads and 1 load, a 60x60 matrix
        else:
            self.Q = block_quad_Q
        R = params['R']
        self.R = np.kron(R,np.eye(len(self.quads))) # e.g. for 4 quads and 1 load, a 16x16 matrix
        self.state_dim = self.Q.shape[0]
        self.ctrl_dim = self.R.shape[0]
        self.K = np.zeros([self.ctrl_dim, self.state_dim]) # e.g. for 4 quads and 1 load, a 16x60 matrix
        self.offset_gravity = params['offset_gravity']
        for quad in self.quads:
            quad.set_thrusts(quad.mass * quad.g / 4 * np.ones(4))
            quad.thrust_control_matrix = np.array([[1/4,             0,  1/(2*quad.L),  1/(4*quad.b)],
                                                   [1/4,  1/(2*quad.L),             0, -1/(4*quad.b)],
                                                   [1/4,             0, -1/(2*quad.L),  1/(4*quad.b)],
                                                   [1/4, -1/(2*quad.L),             0, -1/(4*quad.b)]])
        self.Q = self.Q[0:48,0:48]
        self.K = self.K[:,0:48]
    
    def get_update(self, dt):
        state = np.zeros(self.state_dim)
        state = np.concatenate([obj.state for obj in self.objs])
        # state[self.x_slice['p1']][POS] -= self.goal
        state[self.x_slice['q1']][POS] -= np.array([2, 1,5])
        state[self.x_slice['q2']][POS] -= np.array([0, 1,5])
        # state[self.x_slice['q3']][POS] -= np.array([0,-1,5])
        # state[self.x_slice['q4']][POS] -= np.array([2,-1,5])
        A,B = self.get_ltv_system()
        # print(A.round(2))
        # print(B.round(2))
        try:
            P = sla.solve_continuous_are(A,B,self.Q,self.R)
        except (np.linalg.LinAlgError, ValueError) as err:
            # puts the error where it's actually visible
            print('Nav LQR algebraic Ricatti equation: ' + str(err))
        else:
            self.K = -sla.inv(self.R).dot(B.T).dot(P)
        U = np.dot(self.K, state)
        for quad in self.quads:
            U[self.u_slice[quad.id]][0] += quad.mass * quad.g * self.offset_gravity
            u = quad.thrust_control_matrix @ U[self.u_slice[quad.id]]
            quad.set_thrusts(u)
        E,V = sla.eig(A + B @ self.K)
        # print(np.around(V,2))
    
    def get_ltv_system(self):
        A_list = [None] * len(self.objs)
        B_list = [None] * len(self.quads)
        for i,quad in enumerate(self.quads):
            A,B = quad.get_ltv_system()
            A_list[i] = A
            B_list[i] = B
        for i,load in enumerate(self.loads):
            A = load.get_ltv_system()
            A_list[i + len(self.quads)] = A
        A = sla.block_diag(*A_list)
        B = sla.block_diag(*B_list)
        B = np.pad(B,[[0,self.state_dim - B.shape[0]],[0,0]])
        return A,B
        
class Controller_DDLQR_Team():
    pass

planner_defs = {
    'lqr_team': Controller_LQR_Team,
    'dd_team': Controller_DDLQR_Team,
    }

class PlannerTypeNotFoundError(Exception):
    pass