# -*- coding: utf-8 -*-
"""
Created on Thu Jul  1 15:00:02 2021

@author: aviat
"""
import quadcopter, controller

import datetime
import time

class DynamicsManager():
    """
    Handles general physics (e.g. sim time) and interactions between different
    types of physics object.
    """
    
    def __init__(self, QUAD_DEFS, CTRL_DEFS):
        self.WAIT_WAKE_RATE = 0.02
        self.quads = quadcopter.Quadcopters(quads=QUAD_DEFS, get_time=self.get_time)
        self.ctrls = {}
        for key in CTRL_DEFS:
            self.ctrls[key] = controller.new_controller(identifier=key, params=CTRL_DEFS[key], get_state=self.quads.get_state, get_time=self.get_time, actuate=self.quads.set_motor_speeds)
        self.TIME_SCALING_EPSILON = 0.01
        self.run = True
    
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
        for key in self.quads.quads:
            data[key] = dict([('position', self.quads.get_position(key)), ('orientation', self.quads.get_orientation(key))])
        return data
    
    def start_threads(self, phys_dt, ctrl_dt, time_scaling):
        if time_scaling > self.TIME_SCALING_EPSILON:
            self.time_scaling = time_scaling
        else:
            self.time_scaling = self.TIME_SCALING_EPSILON
        self.pause = False
        self.start = datetime.datetime.now()
        self.pause_start = self.start
        self.time_paused = 0
        self.quads.start_thread(phys_dt)
        for ctrl in self.ctrls.values():
            ctrl.start_thread(ctrl_dt)
    
    def stop_threads(self):
        self.quads.stop_thread()
        for ctrl in self.ctrls.values():
            ctrl.stop_thread()
    
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
    
    def interrupt_handler(self, signal, frame):
        self.run = False