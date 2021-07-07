# -*- coding: utf-8 -*-
"""
Created on Thu Jul  1 15:12:50 2021

@author: aviat
"""
import numpy as np
import math
import scipy.integrate
import time
import threading

class Payloads():
    def __init__(self, payloads, get_time, gravity=9.81):
        self.loads = payloads
        self.get_time= get_time
        self.g = gravity
        self.thread_object = None
        self.ode =  scipy.integrate.ode(self.state_dot).set_integrator('vode',nsteps=500,method='bdf')
        for key in self.loads:
            self.loads[key]['state'] = np.zeros(12)
            self.loads[key]['state'][0:3] = self.loads[key]['position']
            self.loads[key]['state'][6:9] = self.loads[key]['orientation']
            # From Wikipeda "List of moments of inertia" for a cuboid
            ixx = self.loads[key]['mass']*(self.loads[key]['x']**2 + self.loads[key]['y']**2)/12
            iyy = self.loads[key]['mass']*(self.loads[key]['x']**2 + self.loads[key]['z']**2)/12
            izz = self.loads[key]['mass']*(self.loads[key]['y']**2 + self.loads[key]['y']**2)/12
            self.loads[key]['J'] = np.array([[ixx,0,0],[0,iyy,0],[0,0,izz]])
            self.loads[key]['invJ'] = np.linalg.inv(self.loads[key]['J'])
        self.run = True

    def rotation_matrix(self,angles):
        ct = math.cos(angles[0])
        cp = math.cos(angles[1])
        cg = math.cos(angles[2])
        st = math.sin(angles[0])
        sp = math.sin(angles[1])
        sg = math.sin(angles[2])
        R_x = np.array([[1,0,0],[0,ct,-st],[0,st,ct]])
        R_y = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
        R_z = np.array([[cg,-sg,0],[sg,cg,0],[0,0,1]])
        R = np.dot(R_z, np.dot( R_y, R_x ))
        return R

    def wrap_angle(self,val):
        return( ( val + np.pi) % (2 * np.pi ) - np.pi )

    def state_dot(self, time, state, key):
        state_dot = np.zeros(12)
        # The velocities(t+1 x_dots equal the t x_dots)
        state_dot[0] = self.loads[key]['state'][3]
        state_dot[1] = self.loads[key]['state'][4]
        state_dot[2] = self.loads[key]['state'][5]
        # The acceleration
        x_dotdot = np.array([0,0,-self.loads[key]['mass']*self.g]) + np.dot(self.rotation_matrix(self.loads[key]['state'][6:9]),np.array([0,0,(self.loads[key]['m1'].thrust + self.loads[key]['m2'].thrust + self.loads[key]['m3'].thrust + self.loads[key]['m4'].thrust)]))/self.loads[key]['mass']
        state_dot[3] = x_dotdot[0]
        state_dot[4] = x_dotdot[1]
        state_dot[5] = x_dotdot[2]
        # The angular rates(t+1 theta_dots equal the t theta_dots)
        state_dot[6] = self.loads[key]['state'][9]
        state_dot[7] = self.loads[key]['state'][10]
        state_dot[8] = self.loads[key]['state'][11]
        # The angular accelerations
        omega = self.loads[key]['state'][9:12]
        tau = np.array([self.loads[key]['L']*(self.loads[key]['m1'].thrust-self.loads[key]['m3'].thrust), self.loads[key]['L']*(self.loads[key]['m2'].thrust-self.loads[key]['m4'].thrust), self.b*(self.loads[key]['m1'].thrust-self.loads[key]['m2'].thrust+self.loads[key]['m3'].thrust-self.loads[key]['m4'].thrust)])
        omega_dot = np.dot(self.loads[key]['invJ'], (tau - np.cross(omega, np.dot(self.loads[key]['J'],omega))))
        state_dot[9] = omega_dot[0]
        state_dot[10] = omega_dot[1]
        state_dot[11] = omega_dot[2]
        return state_dot

    def update(self):
        self.num_updates += 1
        for key in self.loads:
            self.quads[key]['total_force'] = np.zeros(3)
            self.quads[key]['total_moment'] = np.zeros(3)
            for h,f in zip(self.loads[key]['hardpoints'],self.loads[key]['forces']):
                pass
            self.quads[key]['external_force']/self.quads[key]['mass']
            self.ode.set_initial_value(self.loads[key]['state'],0).set_f_params(key)
            self.loads[key]['state'] = self.ode.integrate(self.ode.t + self.dt)
            self.loads[key]['state'][6:9] = self.wrap_angle(self.loads[key]['state'][6:9])
            self.loads[key]['state'][2] = max(0,self.loads[key]['state'][2])

    def set_motor_speeds(self,load_name,speeds):
        self.loads[load_name]['m1'].set_speed(speeds[0])
        self.loads[load_name]['m2'].set_speed(speeds[1])
        self.loads[load_name]['m3'].set_speed(speeds[2])
        self.loads[load_name]['m4'].set_speed(speeds[3])

    def get_position(self,load_name):
        return self.loads[load_name]['state'][0:3]

    def get_linear_rate(self,load_name):
        return self.loads[load_name]['state'][3:6]

    def get_orientation(self,load_name):
        return self.loads[load_name]['state'][6:9]

    def get_angular_rate(self,load_name):
        return self.loads[load_name]['state'][9:12]

    def get_state(self,load_name):
        return self.loads[load_name]['state']

    def set_position(self,load_name,position):
        self.loads[load_name]['state'][0:3] = position

    def set_orientation(self,load_name,orientation):
        self.loads[load_name]['state'][6:9] = orientation
        
    def add_force_at(self,load_name,hardpoint,force):
        self.loads[load_name]['forces'][hardpoint] += force
    
    def set_external_force(self,load_name,hardpoint,force):
        self.loads[load_name]['forces'][hardpoint] = force

    def thread_run(self):
        update_num = 0
        while(self.run==True):
            time.sleep(0)
            curr_time = self.get_time()
            if curr_time > update_num * self.dt:
                self.update()
                update_num += 1

    def start_thread(self,dt=0.002):
        self.dt = dt
        self.thread_object = threading.Thread(target=self.thread_run)
        self.thread_object.start()

    def stop_thread(self):
        print(self.num_updates)
        self.run = False