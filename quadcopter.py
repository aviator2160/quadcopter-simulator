import util

import numpy as np
import scipy.integrate

# State space representation: [x y z x_dot y_dot z_dot theta phi gamma theta_dot phi_dot gamma_dot]
# From Quadcopter Dynamics, Simulation, and Control by Andrew Gibiansky
def state_dot(time, state, quad):
    state_t = quad.state
    state_dot = np.zeros(12)
    # The velocities(t+1 x_dots equal the t x_dots)
    state_dot[0] = state_t[3]
    state_dot[1] = state_t[4]
    state_dot[2] = state_t[5]
    # The acceleration
    x_dotdot = np.array([0,0,-quad.g]) + quad.external_force/quad.mass + np.dot(util.rotation_matrix(state_t[6:9]),np.array([0,0,(quad.m1.thrust + quad.m2.thrust + quad.m3.thrust + quad.m4.thrust)]))/quad.mass
    state_dot[3] = x_dotdot[0]
    state_dot[4] = x_dotdot[1]
    state_dot[5] = x_dotdot[2]
    # The angular rates(t+1 theta_dots equal the t theta_dots)
    state_dot[6] = state_t[9]
    state_dot[7] = state_t[10]
    state_dot[8] = state_t[11]
    # The angular accelerations
    omega = state_t[9:12]
    tau = np.array([quad.L * (quad.m1.thrust - quad.m3.thrust), quad.L * (quad.m2.thrust - quad.m4.thrust), quad.b * (quad.m1.thrust - quad.m2.thrust + quad.m3.thrust - quad.m4.thrust)])
    omega_dot = np.dot(quad.invJ, (tau - np.cross(omega, np.dot(quad.J, omega))))
    state_dot[9] = omega_dot[0]
    state_dot[10] = omega_dot[1]
    state_dot[11] = omega_dot[2]
    return state_dot

class Propeller():
    def __init__(self, prop_dia, prop_pitch, thrust_unit='N'):
        self.dia = prop_dia
        self.pitch = prop_pitch
        self.thrust_unit = thrust_unit
        self.speed = 0 #RPM
        self.thrust = 0

    def set_speed(self,speed):
        self.speed = speed
        # From http://www.electricrcaircraftguy.com/2013/09/propeller-static-dynamic-thrust-equation.html
        self.thrust = 4.392e-8 * self.speed * np.power(self.dia,3.5)/(np.sqrt(self.pitch))
        self.thrust = self.thrust*(4.23e-4 * self.speed * self.pitch)
        if self.thrust_unit == 'Kg':
            self.thrust = self.thrust*0.101972

class Quadcopter():
    ode = scipy.integrate.ode(state_dot).set_integrator('vode',nsteps=500,method='bdf')
    run = True
    num_updates = 0
    
    def __init__(self,defs,g=9.81,b=0.0245):
        self.g = g
        self.b = b
        self.state = np.zeros(12)
        self.state[0:3] = defs['position']
        self.state[6:9] = defs['orientation']
        self.L = defs['L']
        self.r = defs['r']
        self.mass = defs['mass']
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
        Quadcopter.ode.set_initial_value(self.state,0).set_f_params(self)
        self.state = Quadcopter.ode.integrate(Quadcopter.ode.t + dt)
        self.state[6:9] = util.wrap_angle(self.state[6:9])

    def set_motor_speeds(self,speeds):
        self.m1.set_speed(speeds[0])
        self.m2.set_speed(speeds[1])
        self.m3.set_speed(speeds[2])
        self.m4.set_speed(speeds[3])

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
    
    def add_external_force(self,force):
        self.external_force += force
    
    def set_external_force(self,force):
        self.external_force = force