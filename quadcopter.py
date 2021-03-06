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
    R = util.rotation_matrix(state[EUL])
    # Velocity
    state_dot[POS] = state[VEL]
    # Acceleration
    x_dotdot = np.array([0,0,-quad.g]) + 1/quad.mass * (quad.external_force + R[:,2] * np.sum(quad.thrusts))
    state_dot[VEL] = x_dotdot
    # Euler rate
    omega = state[OMG]
    state_dot[EUL] = util.body_omega_to_euler_rates_matrix(state[EUL]) @ omega
    # Angular acceleration
    quad.body_moment = np.array([quad.L * (quad.thrusts[1] - quad.thrusts[3]),
                                 quad.L * (quad.thrusts[0] - quad.thrusts[2]),
                                 quad.b * (quad.thrusts[0] - quad.thrusts[1] + quad.thrusts[2] - quad.thrusts[3])])
    quad.body_moment += R.T @ quad.external_torque
    omega_dot = quad.invJ @ (quad.body_moment - np.cross(omega, quad.J @ omega))
    state_dot[OMG] = omega_dot
    return state_dot

class Quadcopter():
    
    def __init__(self,name,params,g=9.81):
        self.id = name
        self.g = g
        self.state = np.zeros(12)
        self.state[POS] = params['position']
        self.state[EUL] = params['orientation']
        self.L = params['L']
        self.r = params['r']
        self.mass = params['mass']
        self.b = params['prop_torque_coeff']
        self.THRUST_LIMITS = params['thrust_limits']
        # From http://www.electricrcaircraftguy.com/2013/09/propeller-static-dynamic-thrust-equation.html
        dia = params['prop_size'][0]
        pitch = params['prop_size'][1]
        self.prop_thrust_const = 4.392e-8 * np.power(dia,3.5) / (np.sqrt(pitch)) * 4.23e-4 * pitch
        self.thrusts = np.zeros(4)
        self.external_force = np.zeros(3)
        self.external_torque = np.zeros(3)
        self.body_moment = np.zeros(3)
        # From Quadrotor Dynamics and Control by Randal Beard
        ixx = ((2*self.mass*self.r**2)/5)+(2*self.mass*self.L**2)
        iyy = ixx
        izz = ((2*self.mass*self.r**2)/5)+(4*self.mass*self.L**2)
        self.J = np.array([[ixx,0,0],[0,iyy,0],[0,0,izz]])
        self.invJ = np.linalg.inv(self.J)
    
    def update(self,dt):
        ivp_solution = scipy.integrate.solve_ivp(state_dot,(0,dt),self.state,args=(self,),t_eval=[dt],)
        self.state = ivp_solution.y[:,0]
        self.state[EUL] = util.wrap_angle(self.state[EUL])

    # @todo: add dependence of thrust on airspeed
    def set_speeds(self,vals):
        self.set_thrusts(self.prop_thrust_const * np.sign(vals) * np.square(vals))
    
    def set_thrusts(self,vals):
        self.thrusts = np.clip(vals,self.THRUST_LIMITS[0],self.THRUST_LIMITS[1])

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
        
    def add_external_torque(self,torque):
        self.external_torque += torque
    
    def set_external_torque(self,torque):
        self.external_torque = torque
    
    def get_ltv_system(self):
        # Linearization of state_dot, returning A and B from x_dot = Ax + B
        n = 12 # number of independent dynamic variables per rotorcraft
        m = 4 # number of independent input variables per rotorcraft
        # A matrix
        A = np.zeros((n,n))
        A[POS,VEL] = np.eye(3) # position <- velocity
        eulers = self.state[EUL]
        omega = self.state[OMG]
        R = util.rotation_matrix(eulers)
        A[VEL,EUL] = -1/self.mass * R @ util.cross_matrix(np.array([0,0,np.sum(self.thrusts)])) # right-crossed with change in angle
        # A[EUL,EUL] = np.array([[omega[2],        0,0],
        #                        [       0,-omega[2],0],
        #                        [       0, omega[1],0]])
        A[EUL,OMG] = np.eye(3) # util.body_omega_to_euler_rates_matrix(eulers)
        A[OMG,OMG] = self.invJ @ (util.cross_matrix(self.J @ omega) - util.cross_matrix(omega) @ self.J)
        # B matrix
        B = np.zeros((n,m))
        B[VEL,0] = 1/self.mass * R[:,2]
        B[OMG,1:4] = self.invJ
        return A,B
    
    def get_jacobian_sparsity(self):
        A = np.zeros((12,12))
        A[POS,VEL] = np.eye(3)              # position rate <- velocity
        A[VEL,EUL] = np.ones((3,3))         # velocity rate <- orientation
        A[VEL,EUL][2,2] = 0                 #   z-accel is independent of yaw
        A[EUL,EUL] = np.ones((3,3))
        # A[EUL,EUL] = np.array([[1, 1, 0],   # Euler rates <- orientation
        #                        [0, 1, 0],   #   all independent of yaw
        #                        [1, 1, 0]])  #   pitch rate is independent of roll
        A[EUL,OMG] = np.ones((3,3))         # Euler rates <- angular velocity
        A[EUL,OMG][1:3,0] = np.zeros((2,)) #   body-x angular rate only affects roll rate
        A[OMG,OMG] = np.ones((3,3))         # angular velocity rate <- angular velocity
        