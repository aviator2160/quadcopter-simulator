import util

from abc import ABC, abstractmethod
import numpy as np
import scipy.linalg as sla

POS = slice(0,3)
VEL = slice(3,6)
EUL = slice(6,9)
OMG = slice(9,12)

def Controller(get_time, quad, params):
    Specific_Controller = controller_defs.get(params['Type'])
    if Specific_Controller != None:
        return Specific_Controller(get_time, quad, params)
    raise ControllerTypeNotFoundError(str(params['Type']) + " is not a recognized type of controller!")

class Controller_P2P(ABC):
    
    def __init__(self, get_time, quad, params):
        self.get_time = get_time
        self.quad = quad
        self.goals = params['Goals']
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
        self.THRUST_LIMITS = self.quad.thrust_limits
        self.offset_gravity = params['offset_gravity']
        self.quad.set_thrusts(self.quad.mass * self.quad.g / 4 * self.offset_gravity * np.ones(4))
        self.LINEAR_P = params['Linear_PID']['P']
        self.LINEAR_I = params['Linear_PID']['I']
        self.LINEAR_D = params['Linear_PID']['D']
        self.LINEAR_TO_ANGULAR_SCALER = params['Linear_To_Angular_Scaler']
        self.ANGULAR_P = params['Angular_PID']['P']
        self.ANGULAR_I = params['Angular_PID']['I']
        self.ANGULAR_D = params['Angular_PID']['D']
        self.xi_term = 0
        self.yi_term = 0
        self.zi_term = 0
        self.thetai_term = 0
        self.phii_term = 0
        self.gammai_term = 0
    
    def update(self, dt):
        if (len(self.goals) > 0) and (self.get_time() > self.goals[0]['time']):
            self.update_goal(self.goals.pop(0))
        [dest_x,dest_y,dest_z] = self.curr_goal['position']
        [x,y,z,x_dot,y_dot,z_dot,theta,phi,gamma,theta_dot,phi_dot,gamma_dot] = self.quad.get_state()
        x_error = dest_x-x
        y_error = dest_y-y
        z_error = dest_z-z
        self.xi_term += self.LINEAR_I[0]*x_error*dt
        self.yi_term += self.LINEAR_I[1]*y_error*dt
        self.zi_term += self.LINEAR_I[2]*z_error*dt
        cmd_x_dot = self.LINEAR_P[0]*(x_error) + self.LINEAR_D[0]*(-x_dot) + self.xi_term
        cmd_y_dot = self.LINEAR_P[1]*(y_error) + self.LINEAR_D[1]*(-y_dot) + self.yi_term
        cmd_z_dot = self.LINEAR_P[2]*(z_error) + self.LINEAR_D[2]*(-z_dot) + self.zi_term
        throttle = cmd_z_dot + self.quad.mass * self.quad.g * self.offset_gravity / 4
        dest_theta = self.LINEAR_TO_ANGULAR_SCALER[0]*(cmd_x_dot*np.sin(gamma)-cmd_y_dot*np.cos(gamma))
        dest_phi = self.LINEAR_TO_ANGULAR_SCALER[1]*(cmd_x_dot*np.cos(gamma)+cmd_y_dot*np.sin(gamma))
        dest_gamma = self.curr_goal['yaw']
        theta_error = dest_theta-theta
        phi_error = dest_phi-phi
        gamma_error = util.wrap_angle(dest_gamma - gamma)# - gamma_dot
        self.thetai_term += self.ANGULAR_I[0]*theta_error
        self.phii_term += self.ANGULAR_I[1]*phi_error
        self.gammai_term += self.ANGULAR_I[2]*gamma_error
        x_val = self.ANGULAR_P[0]*(theta_error) + self.ANGULAR_D[0]*(-theta_dot) + self.thetai_term
        y_val = self.ANGULAR_P[1]*(phi_error) + self.ANGULAR_D[1]*(-phi_dot) + self.phii_term
        z_val = self.ANGULAR_P[2]*(gamma_error) + self.ANGULAR_D[2]*(-gamma_dot) + self.gammai_term
        m1 = throttle + y_val + z_val
        m2 = throttle + x_val - z_val
        m3 = throttle - y_val + z_val
        m4 = throttle - x_val - z_val
        M = np.clip([m1,m2,m3,m4],self.THRUST_LIMITS[0],self.THRUST_LIMITS[1])
        self.quad.set_thrusts(M)

"""
Based on https://github.com/lisarah/geometric/blob/master/lti.py

Linear time-varying LQR controller
"""

class Controller_LQR_P2P(Controller_P2P):
    
    def initialize(self, params):
        self.THRUST_LIMITS = self.quad.thrust_limits
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
        U = np.dot(self.K, state)
        U[0] += self.quad.mass * self.quad.g * self.offset_gravity
        u = np.clip(self.thrust_control_matrix @ U, self.THRUST_LIMITS[0], self.THRUST_LIMITS[1])
        self.quad.set_thrusts(u)


controller_defs = {
    'pid_p2p': Controller_PID_P2P,
    'lqr_p2p': Controller_LQR_P2P,
    'dd_p2p': Controller_DDLQR_P2P,
    }

class ControllerTypeNotFoundError(Exception):
    pass