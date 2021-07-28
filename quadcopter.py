import util

import numpy as np
import scipy.integrate

POS = slice(0,3)
VEL = slice(3,6)
EUL = slice(6,9)
OMG = slice(9,12)

# State space representation: [x y z x_dot y_dot z_dot theta phi gamma theta_dot phi_dot gamma_dot]
# From Quadcopter Dynamics, Simulation, and Control by Andrew Gibiansky
def state_dot(time, state, quad):
    state_dot = np.zeros(12)
    # The velocities(t+1 x_dots equal the t x_dots)
    state_dot[POS] = state[VEL]
    # The acceleration
    x_dotdot = np.array([0,0,-quad.g]) + quad.external_force/quad.mass + np.dot(util.rotation_matrix(state[EUL]),np.array([0,0,(quad.m1.thrust + quad.m2.thrust + quad.m3.thrust + quad.m4.thrust)]))/quad.mass
    state_dot[VEL] = x_dotdot
    # The angular rates(t+1 theta_dots equal the t theta_dots)
    state_dot[EUL] = state[OMG]
    # The angular accelerations
    omega = state[OMG]
    tau = np.array([quad.L * (quad.m1.thrust - quad.m3.thrust), quad.L * (quad.m2.thrust - quad.m4.thrust), quad.prop_torque_coeff * (quad.m1.thrust - quad.m2.thrust + quad.m3.thrust - quad.m4.thrust)])
    omega_dot = np.dot(quad.invJ, (tau - np.cross(omega, np.dot(quad.J, omega))))
    state_dot[OMG] = omega_dot
    return state_dot

class Propeller():
    def __init__(self, prop_dia, prop_pitch, thrust_unit='N'):
        self.dia = prop_dia
        self.pitch = prop_pitch
        self.thrust_unit = thrust_unit
        self.speed = 0 # RPM
        self.thrust = 0

    def set_speed(self,speed):
        self.speed = speed
        # From http://www.electricrcaircraftguy.com/2013/09/propeller-static-dynamic-thrust-equation.html
        self.thrust = 4.392e-8 * self.speed * np.power(self.dia,3.5)/(np.sqrt(self.pitch))
        self.thrust = self.thrust*(4.23e-4 * self.speed * self.pitch)
        if self.thrust_unit == 'Kg':
            self.thrust = self.thrust*0.101972

class Quadcopter():
    
    def __init__(self,defs,g=9.81):
        self.g = g
        self.state = np.zeros(12)
        self.state[POS] = defs['position']
        self.state[EUL] = defs['orientation']
        self.L = defs['L']
        self.r = defs['r']
        self.mass = defs['mass']
        self.prop_torque_coeff = defs['prop_torque_coeff']
        self.m1 = Propeller(defs['prop_size'][0],defs['prop_size'][1])
        self.m2 = Propeller(defs['prop_size'][0],defs['prop_size'][1])
        self.m3 = Propeller(defs['prop_size'][0],defs['prop_size'][1])
        self.m4 = Propeller(defs['prop_size'][0],defs['prop_size'][1])
        self.external_force = np.zeros(3)
        # From Quadrotor Dynamics and Control by Randal Beard
        ixx = ((2*self.mass*self.r**2)/5)+(2*self.mass*self.L**2)
        iyy = ixx
        izz = ((2*self.mass*self.r**2)/5)+(4*self.mass*self.L**2)
        self.J = np.array([[ixx,0,0],[0,iyy,0],[0,0,izz]])
        self.invJ = np.linalg.inv(self.J)
    
    def update(self,dt):
        ivp_solution = scipy.integrate.solve_ivp(state_dot,(0,dt),self.state,args=(self,),t_eval=[dt])
        self.state = ivp_solution.y[:,0]
        self.state[EUL] = util.wrap_angle(self.state[EUL])

    def set_motor_speeds(self,speeds):
        self.m1.set_speed(speeds[0])
        self.m2.set_speed(speeds[1])
        self.m3.set_speed(speeds[2])
        self.m4.set_speed(speeds[3])

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
    
    def add_external_force(self,force):
        self.external_force += force
    
    def set_external_force(self,force):
        self.external_force = force