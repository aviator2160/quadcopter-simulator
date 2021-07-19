# -*- coding: utf-8 -*-
"""
Created on Thu Jul  1 14:18:35 2021

@author: aviat
"""

import numpy as np

class Cable():
    def __init__(self, quad, load, hardpoint_num, spring=100, damping=1):
        self.quad = quad
        self.load = load
        self.hardpoint_num = hardpoint_num
        self.k = spring
        self.c = damping
        load_to_quad = self.quad.get_position() - self.load.get_hardpoint(self.hardpoint_num)
        self.base_length = np.linalg.norm(load_to_quad)
        self.length = self.base_length
    
    def update(self, dt):
        load_to_quad = self.quad.get_position() - self.load.get_hardpoint(self.hardpoint_num)
        prev_length = self.length
        self.length = np.linalg.norm(load_to_quad)
        l_dot = (self.length - prev_length) / dt
        tension = self.k * max(0, self.length - self.base_length) + self.c * l_dot
        direction_ql = load_to_quad / self.length
        force_ql = tension * direction_ql
        self.load.set_force_at(self.hardpoint_num, force_ql)
        self.quad.add_external_force(-force_ql)