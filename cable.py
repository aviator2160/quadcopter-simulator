# -*- coding: utf-8 -*-
"""
Created on Thu Jul  1 14:18:35 2021

@author: aviat
"""

import util

import numpy as np

class Cable():
    def __init__(self, quad, load, params):
        self.quad = quad
        self.load = load
        self.hardpoint_num = params['hardpoint']
        self.joint = params.get('joint', False)
        if self.joint:
            load.mass += quad.mass
            parallel_axis_offset = load.hardpoints[:,self.hardpoint_num]
            quad_J = quad.J - quad.mass * util.cross_matrix(parallel_axis_offset)
            load.J += quad_J
        else:
            self.k = params['stiffness']
            self.c = params['damping']
            load_to_quad = self.quad.get_position() - self.load.get_hardpoint(self.hardpoint_num)
            self.base_length = np.linalg.norm(load_to_quad)
            self.length = self.base_length
    
    def update(self, dt):
        if self.joint:
            R = util.rotation_matrix(self.quad.get_orientation())
            thrust = np.sum(self.quad.thrusts) * R[:,2]
            self.load.set_force_at(self.hardpoint_num, thrust)
            self.quad.set_position(self.load.get_hardpoint(self.hardpoint_num))
        else:
            load_to_quad = self.quad.get_position() - self.load.get_hardpoint(self.hardpoint_num)
            prev_length = self.length
            self.length = np.linalg.norm(load_to_quad)
            l_dot = (self.length - prev_length) / dt
            tension = self.k * max(0, self.length - self.base_length) + self.c * l_dot
            direction_ql = load_to_quad / self.length
            force_ql = tension * direction_ql
            self.load.set_force_at(self.hardpoint_num, force_ql)
            self.quad.add_external_force(-force_ql)