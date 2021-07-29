import util

import numpy as np
import cvxpy as cvx
import scipy.linalg as sla

POS = slice(0,3)
VEL = slice(3,6)
EUL = slice(6,9)
OMG = slice(9,12)

def new_controller(get_time, quad, params, identifier):
    # identifier is currently used ONLY for printing goals
    Controller = controllers.get(params['Type'])
    if Controller != None:
        return Controller(get_time, quad, params, identifier)
    raise ControllerTypeNotFoundError(str(params['Type']) + " is not a recognized type of controller!")

class Controller_PID_Point2Point():
    def __init__(self, get_time, quad, params, identifier):
        self.get_time = get_time
        self.quad = quad
        self.id = identifier
        self.goals = params['Goals']
        self.curr_goal = {'time': 0, 'position': (0,0,0), 'yaw': 0}
        self.MOTOR_LIMITS = params['Motor_limits']
        self.TILT_LIMITS = [(params['Tilt_limits'][0]/180.0)*3.14,(params['Tilt_limits'][1]/180.0)*3.14]
        self.YAW_CONTROL_LIMITS = params['Yaw_Control_Limits']
        self.Z_LIMITS = [self.MOTOR_LIMITS[0]+params['Z_XY_offset'],self.MOTOR_LIMITS[1]-params['Z_XY_offset']]
        self.LINEAR_P = params['Linear_PID']['P']
        self.LINEAR_I = params['Linear_PID']['I']
        self.LINEAR_D = params['Linear_PID']['D']
        self.LINEAR_TO_ANGULAR_SCALER = params['Linear_To_Angular_Scaler']
        self.YAW_RATE_SCALER = params['Yaw_Rate_Scaler']
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
        dest_x_dot = self.LINEAR_P[0]*(x_error) + self.LINEAR_D[0]*(-x_dot) + self.xi_term
        dest_y_dot = self.LINEAR_P[1]*(y_error) + self.LINEAR_D[1]*(-y_dot) + self.yi_term
        dest_z_dot = self.LINEAR_P[2]*(z_error) + self.LINEAR_D[2]*(-z_dot) + self.zi_term
        throttle = np.clip(dest_z_dot,self.Z_LIMITS[0],self.Z_LIMITS[1])
        dest_theta = self.LINEAR_TO_ANGULAR_SCALER[0]*(dest_x_dot*np.sin(gamma)-dest_y_dot*np.cos(gamma))
        dest_phi = self.LINEAR_TO_ANGULAR_SCALER[1]*(dest_x_dot*np.cos(gamma)+dest_y_dot*np.sin(gamma))
        dest_gamma = self.curr_goal['yaw']
        dest_theta,dest_phi = np.clip(dest_theta,self.TILT_LIMITS[0],self.TILT_LIMITS[1]),np.clip(dest_phi,self.TILT_LIMITS[0],self.TILT_LIMITS[1])
        theta_error = dest_theta-theta
        phi_error = dest_phi-phi
        gamma_dot_error = (self.YAW_RATE_SCALER*util.wrap_angle(dest_gamma-gamma)) - gamma_dot
        self.thetai_term += self.ANGULAR_I[0]*theta_error
        self.phii_term += self.ANGULAR_I[1]*phi_error
        self.gammai_term += self.ANGULAR_I[2]*gamma_dot_error
        x_val = self.ANGULAR_P[0]*(theta_error) + self.ANGULAR_D[0]*(-theta_dot) + self.thetai_term
        y_val = self.ANGULAR_P[1]*(phi_error) + self.ANGULAR_D[1]*(-phi_dot) + self.phii_term
        z_val = self.ANGULAR_P[2]*(gamma_dot_error) + self.gammai_term
        z_val = np.clip(z_val,self.YAW_CONTROL_LIMITS[0],self.YAW_CONTROL_LIMITS[1])
        m1 = throttle + x_val + z_val
        m2 = throttle + y_val - z_val
        m3 = throttle - x_val + z_val
        m4 = throttle - y_val - z_val
        M = np.clip([m1,m2,m3,m4],self.MOTOR_LIMITS[0],self.MOTOR_LIMITS[1])
        self.quad.actuate(M)

    def update_goal(self,new_goal):
        # new_goal might not contain all possible goals
        for key in new_goal:
            self.curr_goal[key] = new_goal[key]
        print(self.id + " goal: " + str(self.curr_goal))

    def update_yaw_target(self,target):
        self.yaw_target = util.wrap_angle(target)

"""
Based on https://github.com/lisarah/geometric/blob/master/lti.py

Linear time-varying LQR controller
"""
class Controller_LQR_Point2Point():
    def __init__(self, get_time, quad, params, identifier):
        self.get_time = get_time
        self.quad = quad
        self.id = identifier
        self.goals = params['Goals']
        self.curr_goal = {'time': 0, 'position': (0,0,0), 'yaw': 0}
        self.MOTOR_LIMITS = params['Motor_limits']
        self.TILT_LIMITS = [(params['Tilt_limits'][0]/180.0)*3.14,(params['Tilt_limits'][1]/180.0)*3.14]
        self.YAW_CONTROL_LIMITS = params['Yaw_Control_Limits']
        self.Q = params['Q']
        self.R = params['R']
        self.quad.get_ltv_system()
    
    def update(self, dt):
        if (len(self.goals) > 0) and (self.get_time() > self.goals[0]['time']):
            self.update_goal(self.goals.pop(0))
        state = np.zeros(12)
        state = self.quad.get_state().copy()
        state[POS] -= self.curr_goal['position']
        state[EUL][2] -= self.curr_goal['yaw']
        A,B = self.quad.get_ltv_system()
        P = sla.solve_continuous_are(A,B,self.Q,self.R)
        K = -sla.inv(self.R).dot(B.T).dot(P)
        print(K)
        u = np.dot(K, state)
        u = np.clip(u,self.MOTOR_LIMITS[0],self.MOTOR_LIMITS[1])
        self.quad.actuate(u)

    def update_goal(self,new_goal):
        # new_goal might not contain all possible goals
        for key in new_goal:
            self.curr_goal[key] = new_goal[key]
        print(self.id + " goal: " + str(self.curr_goal))

controllers = {
    'pid_p2p': Controller_PID_Point2Point,
    'lqr_p2p': Controller_LQR_Point2Point,
    }

class ControllerTypeNotFoundError(Exception):
    pass