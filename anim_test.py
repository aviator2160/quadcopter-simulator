# -*- coding: utf-8 -*-
"""
Created on Sat Jun 26 14:56:25 2021

@author: aviat
"""

import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import mpl_toolkits.mplot3d.axes3d as Axes3D

fig = plt.figure()
ax = Axes3D.Axes3D(fig)
ax.set_xlim3d([-2.0, 2.0])
ax.set_xlabel('X')
ax.set_ylim3d([-2.0, 2.0])
ax.set_ylabel('Y')
ax.set_zlim3d([0, 5.0])
ax.set_zlabel('Z')
ax.set_title('Quadcopter Simulation')
l1 = None
l2 = None
hub = None

def rotation_matrix(angles):
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

def init_plot():
    global l1, l2, hub
    l1,  = ax.plot([],[],[],color='blue',linewidth=3,antialiased=False)
    l2,  = ax.plot([],[],[],color='red',linewidth=3,antialiased=False)
    hub, = ax.plot([],[],[],marker='o',color='green', markersize=6,antialiased=False)
    return (l1, l2, hub)

def update(i):
    global l1, l2, hub
    position = [math.cos(0.1 * i), math.sin(0.1 * i), 0]
    R = rotation_matrix([0, 0, 0.05 * i])
    L = 0.3
    points = np.array([ [-L,0,0], [L,0,0], [0,-L,0], [0,L,0], [0,0,0], [0,0,0] ]).T
    points = np.dot(R,points)
    points[0,:] += position[0]
    points[1,:] += position[1]
    points[2,:] += position[2]
    l1.set_data(points[0,0:2],points[1,0:2])
    l1.set_3d_properties(points[2,0:2])
    l2.set_data(points[0,2:4],points[1,2:4])
    l2.set_3d_properties(points[2,2:4])
    hub.set_data(points[0,5],points[1,5])
    hub.set_3d_properties(points[2,5])
    plt.show()
    plt.pause(0.000000000000001)
    return (l1, l2, hub)

anim = animation.FuncAnimation(fig, update, init_func=init_plot,
                               frames=120, interval=30, blit=True)