# -*- coding: utf-8 -*-
"""
Created on Thu Jul  1 15:00:02 2021

@author: aviat
"""
from quadcopter import Quadcopter
from payload import Payload
from cable import Cable
import controller

import numpy as np
import datetime
import time
import threading

class PhysicsManager():
    """
    Handles general physics (e.g. sim time) and interactions between different
    types of physics object.
    """
    
    def __init__(self,QUAD_DEFS,LOAD_DEFS,CABLE_DEFS,CTRL_DEFS):
        self.WAIT_WAKE_RATE = 0.02
        self.time_scaling_EPSILON = 0.01
        self.quads = {}
        for key in QUAD_DEFS:
            self.quads[key] = Quadcopter(QUAD_DEFS[key])
        self.ctrls = {}
        for key in CTRL_DEFS:
            self.ctrls[key] = controller.new_controller(get_time=self.get_time,quad=self.quads[key],params=CTRL_DEFS[key],identifier=key)
        self.loads = {}
        for key in LOAD_DEFS:
            self.loads[key] = Payload(LOAD_DEFS[key])
        self.cables = {}
        for key,defs in CABLE_DEFS.items():
            self.cables[key] = Cable(self.quads[defs['quad']],self.loads[defs['load']],defs['hardpoint'],defs['stiffness'],defs['damping'])
    
    def get_time(self, scaled=True):
        if self.pause == False:
            curr_time = (datetime.datetime.now() - self.start).total_seconds() - self.time_paused
        else:
            curr_time = (self.pause_start - self.start).total_seconds() - self.time_paused
        if scaled == True:
            return curr_time / self.time_scaling
        else:
            return curr_time
    
    def visual_data(self):
        data = {}
        for key in self.quads:
            data[key] = dict([('position', self.quads[key].get_position()), ('orientation', self.quads[key].get_orientation())])
        for key in self.loads:
            data[key] = dict([('position', self.loads[key].get_position()), ('orientation', self.loads[key].get_orientation())])
        for key,cable in self.cables.items():
            data[key] = zip(cable.quad.get_position(), cable.load.get_hardpoint(cable.hardpoint_num))
        return data
    
    def phys_update(self, dt):
        for quad in self.quads.values():
            quad.update(dt)
            quad.set_external_force(np.zeros(3))
        for load in self.loads.values():
            load.update(dt)
        for cable in self.cables.values():
            cable.update(dt)
    
    def start_threads(self, phys_dt, ctrl_dt, time_scaling):
        time.sleep(0.1) # Extra time to let GUI initialize
        self.time_scaling = max(time_scaling, self.time_scaling_EPSILON)
        self.run = True
        self.pause = False
        self.start = datetime.datetime.now()
        self.pause_start = self.start
        self.time_paused = 0
        self.phys_thread = threading.Thread(target=self.run_thread,args=(phys_dt,ctrl_dt))
        self.phys_thread.start()
    
    def run_thread(self, phys_dt, ctrl_dt):
        phys_update_num = 0
        ctrl_update_num = 0
        while(self.run==True):
            time.sleep(0)
            curr_time = self.get_time()
            if curr_time > phys_update_num * phys_dt:
                self.phys_update(phys_dt)
                phys_update_num += 1
            if curr_time > ctrl_update_num * ctrl_dt:
                for ctrl in self.ctrls.values():
                    ctrl.update(ctrl_dt)
                ctrl_update_num += 1
        # print(phys_update_num)
        # print(self.get_time() / phys_update_num)
        # print(ctrl_update_num)
    
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
            remaining = (end_time - self.get_time()) * self.time_scaling
            if remaining <= 0:
                break
            elif remaining < self.WAIT_WAKE_RATE:
                time.sleep(remaining)
            else:
                time.sleep(self.WAIT_WAKE_RATE)
    
    def on_keyboard_interrupt(self, signal, frame):
        self.stop_threads()