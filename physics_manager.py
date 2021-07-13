# -*- coding: utf-8 -*-
"""
Created on Thu Jul  1 15:00:02 2021

@author: aviat
"""
from quadcopter import Quadcopter
from payload import Payload
import controller

import datetime
import time
import threading

class PhysicsManager():
    """
    Handles general physics (e.g. sim time) and interactions between different
    types of physics object.
    """
    
    def __init__(self, QUAD_DEFS, CTRL_DEFS, LOAD_DEFS=None):
        self.WAIT_WAKE_RATE = 0.02
        self.TIME_SCALING_EPSILON = 0.01
        self.quads = {}
        for key in QUAD_DEFS:
            self.quads[key] = Quadcopter(QUAD_DEFS[key])
        self.ctrls = {}
        for key in CTRL_DEFS:
            self.ctrls[key] = controller.new_controller(identifier=key, params=CTRL_DEFS[key], get_state=self.quads[key].get_state, get_time=self.get_time, actuate=self.quads[key].set_motor_speeds)
        self.loads = {}
        for key in LOAD_DEFS:
            self.loads[key] = Payload(LOAD_DEFS[key])
    
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
        return data
    
    def phys_update(self, dt):
        for quad in self.quads.values():
            quad.update(dt)
        for load in self.loads.values():
            load.update(dt)
    
    def start_threads(self, phys_dt, ctrl_dt, time_scaling):
        if time_scaling > self.TIME_SCALING_EPSILON:
            self.time_scaling = time_scaling
        else:
            self.time_scaling = self.TIME_SCALING_EPSILON
        self.run = True
        self.pause = False
        self.start = datetime.datetime.now()
        self.pause_start = self.start
        self.time_paused = 0
        self.phys_thread = threading.Thread(target=self.check_thread_update,args=(phys_dt,self.phys_update,phys_dt))
        self.phys_thread.start()
        self.ctrl_threads = {}
        for key,ctrl in self.ctrls.items():
            self.ctrl_threads[key] = threading.Thread(target=self.check_thread_update,args=(phys_dt,ctrl.update))
            self.ctrl_threads[key].start()
    
    def check_thread_update(self, dt, update, args=None):
        update_num = 0
        while(self.run==True):
            time.sleep(0)
            curr_time = self.get_time()
            if curr_time > update_num * dt:
                update(args)
                update_num += 1
        # super hacky way to check for just physics updates
        if (args != None):
            print(update_num)
    
    def stop_threads(self):
        self.run = False
    
    def pause_threads(self, pause):
        self.pause = pause
        if pause == True:
            self.pause_start = datetime.datetime.now()
        else:
            self.time_paused += (datetime.datetime.now() - self.pause_start).total_seconds()
    
    def wait_until_time(self, end_time, check_quit):
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