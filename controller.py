import util
import disturbance_decouple as dd
import subspace

from abc import ABC, abstractmethod
import numpy as np
import scipy.linalg as sla

POS = slice(0,3)
VEL = slice(3,6)
EUL = slice(6,9)
OMG = slice(9,12)

def Controller(get_time, quad, params):
    Specific_Controller = controller_defs.get(params['type'])
    if Specific_Controller != None:
        return Specific_Controller(get_time, quad, params)
    raise ControllerTypeNotFoundError(str(params['type']) + " is not a recognized type of controller!")

class Controller_P2P(ABC):
    
    def __init__(self, get_time, quad, params):
        self.get_time = get_time
        self.quad = quad
        self.goals = params['goals']
        self.curr_goal = {'time': 0, 'position': (0,0,0), 'yaw': 0}
        self.initialize(params)
    
    def get_update(self, dt):
        if (len(self.goals) > 0) and (self.get_time() > self.goals[0]['time']):
            new_goal = self.goals.pop(0)
            # new_goal might not contain all possible goals
            for key in new_goal:
                self.curr_goal[key] = new_goal[key]
            print(self.quad.id + " goal: " + str(self.curr_goal))
        self.update(dt)
    
    @abstractmethod
    def initialize(self, params):
        pass
    
    @abstractmethod
    def update(self, dt):
        pass

class Controller_PID_P2P(Controller_P2P):
    
    def initialize(self, params):
        self.offset_gravity = params['offset_gravity']
        self.quad.set_thrusts(self.quad.mass * self.quad.g / 4 * self.offset_gravity * np.ones(4))
        self.LINEAR_P = params['Linear_PID']['P']
        self.LINEAR_I = params['Linear_PID']['I']
        self.LINEAR_D = params['Linear_PID']['D']
        self.LINEAR_TO_ANGULAR_SCALER = params['Linear_To_Angular_Scaler']
        self.ANGULAR_P = params['Angular_PID']['P']
        self.ANGULAR_I = params['Angular_PID']['I']
        self.ANGULAR_D = params['Angular_PID']['D']
        self.linear_i_term = np.zeros(3)
        self.angular_i_term = np.zeros(3)
    
    def update(self, dt):
        state = self.quad.get_state()
        linear_error = self.curr_goal['position'] - state[POS]
        self.linear_i_term += self.LINEAR_I * linear_error * dt
        cmd_thrust = self.LINEAR_P * linear_error - self.LINEAR_D * state[VEL] + self.linear_i_term
        throttle = cmd_thrust[2] + self.quad.mass * self.quad.g * self.offset_gravity / 4
        gamma = state[EUL][2]
        dest_theta = self.LINEAR_TO_ANGULAR_SCALER[0] * (cmd_thrust[0] * np.sin(gamma) - cmd_thrust[1] * np.cos(gamma))
        dest_phi   = self.LINEAR_TO_ANGULAR_SCALER[1] * (cmd_thrust[0] * np.cos(gamma) + cmd_thrust[1] * np.sin(gamma))
        dest_gamma = self.curr_goal['yaw']
        dest_eulers = np.array([dest_theta,dest_phi,dest_gamma])
        angular_error = util.wrap_angle(dest_eulers - state[EUL])
        self.angular_i_term += self.ANGULAR_I * angular_error * dt
        cmd_torque = self.ANGULAR_P * angular_error - self.ANGULAR_D * state[OMG] + self.angular_i_term
        m1 = throttle + cmd_torque[1] + cmd_torque[2]
        m2 = throttle + cmd_torque[0] - cmd_torque[2]
        m3 = throttle - cmd_torque[1] + cmd_torque[2]
        m4 = throttle - cmd_torque[0] - cmd_torque[2]
        self.quad.set_thrusts([m1,m2,m3,m4])

"""
Based on https://github.com/lisarah/geometric/blob/master/lti.py

Linear time-varying LQR controller
"""

class Controller_LQR_P2P(Controller_P2P):
    
    def initialize(self, params):
        self.thrust_control_matrix = np.array([[1/4,                  0,  1/(2*self.quad.L),  1/(4*self.quad.b)],
                                               [1/4,  1/(2*self.quad.L),                  0, -1/(4*self.quad.b)],
                                               [1/4,                  0, -1/(2*self.quad.L),  1/(4*self.quad.b)],
                                               [1/4, -1/(2*self.quad.L),                  0, -1/(4*self.quad.b)]])
        self.Q = params['Q']
        self.R = params['R']
        self.K = np.zeros([4, 12])
        self.offset_gravity = params['offset_gravity']
        self.quad.set_thrusts(self.quad.mass * self.quad.g / 4 * np.ones(4))
    
    def update(self, dt):
        state = np.zeros(12)
        state = self.quad.get_state().copy()
        state[POS] -= self.curr_goal['position']
        state[EUL][2] -= self.curr_goal['yaw']
        state[EUL][2] = util.wrap_angle(state[EUL][2])
        # print(state[6:8])
        A,B = self.quad.get_ltv_system()
        try:
            P = sla.solve_continuous_are(A,B,self.Q,self.R)
        except (np.linalg.LinAlgError, ValueError) as err:
            # puts the error where it's actually visible
            print(self.quad.id + ' LQR algebraic Ricatti equation: ' + str(err))
        else:
            self.K = -sla.inv(self.R).dot(B.T).dot(P)
        E,V = sla.eig(A + B @ self.K)
        # print(*sorted(E.round(2), key=lambda z: z.real))
        U = np.dot(self.K, state)
        U[0] += self.quad.mass * self.quad.g * self.offset_gravity
        u = self.thrust_control_matrix @ U
        self.quad.set_thrusts(u)
        # print(np.around(U,2))
        # print(np.around(B,2))

class Controller_DDLQR_P2P(Controller_P2P):
    
    def initialize(self, params):
        self.thrust_control_matrix = np.array([[1/4,                  0,  1/(2*self.quad.L),  1/(4*self.quad.b)],
                                               [1/4,  1/(2*self.quad.L),                  0, -1/(4*self.quad.b)],
                                               [1/4,                  0, -1/(2*self.quad.L),  1/(4*self.quad.b)],
                                               [1/4, -1/(2*self.quad.L),                  0, -1/(4*self.quad.b)]])
        self.Q = params['Q']
        self.R = params['R']
        self.E = np.zeros((12,2))
        self.E[VEL,:] = np.eye(3,2)
        self.H = np.zeros((2,12))
        self.H[:,EUL] = np.eye(2,3)
        self.K = np.zeros((12,4))
        self.F = np.zeros((12,4))
        self.offset_gravity = params['offset_gravity']
        self.quad.set_thrusts(self.quad.mass * self.quad.g / 4 * np.ones(4))
    
    def update(self, dt):
        state = np.zeros(12)
        state = self.quad.get_state().copy()
        state[POS] -= self.curr_goal['position']
        state[EUL][2] -= self.curr_goal['yaw']
        state[EUL][2] = util.wrap_angle(state[EUL][2])
        # print(state[6:8])
        # Generate LQR feedback controller
        A,B = self.quad.get_ltv_system()
        try:
            P = sla.solve_continuous_are(A,B,self.Q,self.R)
        except (np.linalg.LinAlgError, ValueError) as err:
            # puts the error where it's actually visible
            print(self.quad.id + ' LQR algebraic Ricatti equation: ' + str(err))
        else:
            self.K = -sla.inv(self.R).dot(B.T).dot(P)
        # Generate disturbance decoupling feedback controller
        A_star = A + B @ self.K
        V_0,self.F = dd.disturbance_decoupling(self.H,A_star,B,verbose=False)
        if not subspace.contained(subspace.image(self.E), V_0):
            print('Disturbance matrix intersects the decoupling space.')
        # Apply overall controller
        E,V = sla.eig(A + B @ (self.K + self.F))
        # print(*sorted(E.round(2), key=lambda z: z.real))
        # print(self.K.round(1))
        U = np.dot(self.K + self.F, state)
        U[0] += self.quad.mass * self.quad.g * self.offset_gravity
        u = self.thrust_control_matrix @ U
        self.quad.set_thrusts(u)

controller_defs = {
    'pid_p2p': Controller_PID_P2P,
    'lqr_p2p': Controller_LQR_P2P,
    'dd_p2p': Controller_DDLQR_P2P,
    }

class ControllerTypeNotFoundError(Exception):
    pass