# -*- coding: utf-8 -*-
"""
Created on Thu Jul  1 15:00:02 2021

@author: aviat
"""
from quadcopter import Quadcopter
from payload import Payload
from cable import Cable
from controller import Controller

import numpy as np
import datetime
import time
import threading

WINDY = False
WIND_Z = False
WIND_FORCE = (0,0,0)
WIND_NOISE = 10
VORTEX_TORQUE = (0,0,0)
VORTEX_NOISE = 0

class PhysicsManager():
    """
    Handles general physics (e.g. sim time) and interactions between different
    types of physics object.
    """
    
    def __init__(self,QUAD_DEFS,LOAD_DEFS,CABLE_DEFS,CTRL_DEFS):
        self.WAIT_WAKE_RATE = 0.02
        self.TIME_SCALE_EPSILON = 0.01
        self.quads = {}
        for key,defs in QUAD_DEFS.items():
            self.quads[key] = Quadcopter(name=key,params=defs)
        self.loads = {}
        for key,defs in LOAD_DEFS.items():
            self.loads[key] = Payload(name=key,params=defs)
        self.cables = {}
        for key,defs in CABLE_DEFS.items():
            self.cables[key] = Cable(self.quads[defs['quad']],self.loads[defs['load']],params=defs)
        self.ctrls = {}
        for key,defs in CTRL_DEFS.items():
            self.ctrls[key] = Controller(get_time=self.get_time,quad=self.quads[key],params=defs)
        np.random.seed(1234)
    
    def get_time(self, scaled=True):
        if self.pause == False:
            curr_time = (datetime.datetime.now() - self.start).total_seconds() - self.time_paused
        else:
            curr_time = (self.pause_start - self.start).total_seconds() - self.time_paused
        if scaled == True:
            return curr_time / self.time_scale
        else:
            return curr_time
    
    def visual_data(self):
        sim_data = {}
        for key in self.quads:
            sim_data[key] = dict([('position', self.quads[key].get_position()), ('orientation', self.quads[key].get_orientation())])
        for key in self.loads:
            sim_data[key] = dict([('position', self.loads[key].get_position()), ('orientation', self.loads[key].get_orientation())])
        for key,cable in self.cables.items():
            sim_data[key] = zip(cable.quad.get_position(), cable.load.get_hardpoint(cable.hardpoint_num))
        graph_data = {"q1 position z (m)": self.quads['q1'].get_position()[2]}
        # if len(self.loads) > 0:
        #     ori = self.loads['p1'].get_orientation()
        # else:
        #     ori = self.quads['q1'].get_orientation()
        # graph_data['Pitch (deg)'] = np.rad2deg(ori[1])
        # graph_data['Roll (deg)'] = np.rad2deg(ori[0])
        return (sim_data,graph_data)
    
    def check_update(self, curr_time):
        if curr_time > self.update_num * self.dt:
            self.update_num += 1
            for quad in self.quads.values():
                if WINDY:
                    (wind,vortex) = self.get_wind()
                    quad.add_external_force(wind)
                    quad.add_external_torque(vortex)
                quad.update(self.dt)
                quad.set_external_force(np.zeros(3))
            for load in self.loads.values():
                load.update(self.dt)
            for cable in self.cables.values():
                cable.update(self.dt)
    
    def get_wind(self):
        wind = WIND_FORCE + np.random.normal(0,WIND_NOISE,3)
        if not WIND_Z:
            wind[2] = 0
        vortex = VORTEX_TORQUE + np.random.normal(0,VORTEX_NOISE,3)
        if not WIND_Z:
            vortex[2] = 0
        return (wind,vortex)
    
    def start_threads(self, dt, time_scale):
        time.sleep(0.1) # Extra time to let GUI initialize
        self.dt = dt
        self.time_scale = max(time_scale, self.TIME_SCALE_EPSILON)
        self.run = True
        self.pause = False
        self.start = datetime.datetime.now()
        self.pause_start = self.start
        self.time_paused = 0
        self.phys_thread = threading.Thread(target=self.run_thread)
        self.phys_thread.start()
    
    def run_thread(self):
        self.update_num = 0
        while(self.run==True):
            time.sleep(0)
            curr_time = self.get_time()
            self.check_update(curr_time)
            for ctrl in self.ctrls.values():
                ctrl.check_update(curr_time)
        print(self.update_num)
        # print(self.get_time() / self.update_num)
    
    def stop_threads(self):
        self.run = False
    
    def pause_threads(self, pause):
        self.pause = pause
        if pause == True:
            self.pause_start = datetime.datetime.now()
        else:
            self.time_paused += (datetime.datetime.now() - self.pause_start).total_seconds()
    
    def wait_until_time(self, end_time):
        while self.run:
            remaining = (end_time - self.get_time()) * self.time_scale
            if remaining <= 0:
                break
            elif remaining < self.WAIT_WAKE_RATE:
                time.sleep(remaining)
            else:
                time.sleep(self.WAIT_WAKE_RATE)
    
    def on_keyboard_interrupt(self, signal, frame):
        self.stop_threads()
