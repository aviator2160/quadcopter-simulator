import util

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as color
import mpl_toolkits.mplot3d.axes3d as Axes3D
import matplotlib.animation as animation
import sys

PAUSE_ON_START = True
SHOW_TRAILS = True
DARK_MODE = True

class Sim_GUI():
    # 'quad_list' is a dictionary of format: quad_list = {'quad_1_name':{'position':quad_1_position,'orientation':quad_1_orientation,'arm_span':quad_1_arm_span}, ...}
    def __init__(self,QUAD_DEFS,LOAD_DEFS,CABLE_DEFS,get_data,get_time):
        self.quads = QUAD_DEFS
        self.loads = LOAD_DEFS
        self.cables = CABLE_DEFS
        self.get_data = get_data
        self.get_time = get_time
        self.fig = plt.figure()
        self.ax = Axes3D.Axes3D(self.fig, auto_add_to_figure=False)
        self.fig.add_axes(self.ax)
        self.ax.set_xlim3d([-2.0, 2.0])
        self.ax.set_xlabel('X')
        self.ax.set_ylim3d([-2.0, 2.0])
        self.ax.set_ylabel('Y')
        self.ax.set_zlim3d([0, 5.0])
        self.ax.set_zlabel('Z')
        self.ax.set_title('Quadcopter Simulation')
        if DARK_MODE:
            self.ax.set_facecolor((0.1,0.15,0.2))
            for pane in [self.ax.w_xaxis, self.ax.w_yaxis, self.ax.w_zaxis]:
                pane.set_pane_color((0.2,0.3,0.4))
            self.ax.grid(color='whitesmoke')
            for axis,name in zip([self.ax.xaxis,self.ax.yaxis,self.ax.zaxis],['x','y','z']):
                axis.label.set_color('whitesmoke')
                self.ax.tick_params(axis=name, colors='whitesmoke')
        # self.init_plot()
        #plt.figtext(0.02,0.95,'PID control (2x speed)',fontsize=20)
        self.fig.canvas.mpl_connect('key_press_event', self.key_press_routine)
        self.fig.canvas.mpl_connect('close_event', self.close_routine)
        
    def init_plot(self):
        blit_artists = []
        for i,quad in enumerate(self.quads.values()):
            quad['color'] = color.hsv_to_rgb(((0.3 + 0.4 * i) % 1, 0.8, 1))
            if (SHOW_TRAILS):
                quad['trail'], = self.ax.plot([],[],[],color=quad['color'],linewidth=0.5,antialiased=False)
                blit_artists.append(quad['trail'])
        for key in self.loads:
            self.loads[key]['lines'] = [None] * 4
            self.loads[key]['lines'][0], = self.ax.plot([],[],[],color='red',linewidth=3,antialiased=False)
            self.loads[key]['lines'][1], = self.ax.plot([],[],[],color='gold',linewidth=3,antialiased=False)
            self.loads[key]['lines'][2], = self.ax.plot([],[],[],color='green',linewidth=3,antialiased=False)
            self.loads[key]['lines'][3], = self.ax.plot([],[],[],color='blue',linewidth=3,antialiased=False)
            for line in self.loads[key]['lines']:
                blit_artists.append(line)
        for key in self.cables:
            self.cables[key]['line'], = self.ax.plot([],[],[],color='lightgray',linewidth=0.5,antialiased=False)
            blit_artists.append(self.cables[key]['line'])
        for key in self.quads:
            self.quads[key]['l1'], = self.ax.plot([],[],[],color='blue',linewidth=3,antialiased=False)
            self.quads[key]['l2'], = self.ax.plot([],[],[],color='red',linewidth=3,antialiased=False)
            self.quads[key]['hub'], = self.ax.plot([],[],[],marker='o',color=self.quads[key]['color'],markersize=6,antialiased=False)
            blit_artists.append(self.quads[key]['l1'])
            blit_artists.append(self.quads[key]['l2'])
        return tuple(blit_artists)
    
    def update(self, i=0):
        blit_artists = []
        data = self.get_data()
        if (SHOW_TRAILS):
            for key in self.quads:
                trail_pts_update = zip(self.quads[key]['trail'].get_data_3d(), data[key]['position'])
                self.quads[key]['trail'].set_data_3d([np.append(curr,new) for curr,new in trail_pts_update])
                blit_artists.append(self.quads[key]['trail'])
        for key in self.loads:
            self.loads[key]['position'] = data[key]['position']
            self.loads[key]['orientation'] = data[key]['orientation']
            R = util.rotation_matrix(self.loads[key]['orientation'])
            points = np.array(self.loads[key]['hardpoints']).T
            points = np.dot(R,points)
            points[0,:] += self.loads[key]['position'][0]
            points[1,:] += self.loads[key]['position'][1]
            points[2,:] += self.loads[key]['position'][2]
            for j,line in enumerate(self.loads[key]['lines']):
                line.set_data_3d(*zip(self.loads[key]['position'], points[:,j]))
                blit_artists.append(line)
        for key,cable in self.cables.items():
            cable['line'].set_data_3d(*data[key])
            blit_artists.append(cable['line'])
        for key in self.quads:
            self.quads[key]['position'] = data[key]['position']
            self.quads[key]['orientation'] = data[key]['orientation']
            R = util.rotation_matrix(self.quads[key]['orientation'])
            L = self.quads[key]['L']
            points = np.array([ [-L,0,0], [L,0,0], [0,-L,0], [0,L,0], [0,0,0], [0,0,0] ]).T
            points = np.dot(R,points)
            points[0,:] += self.quads[key]['position'][0]
            points[1,:] += self.quads[key]['position'][1]
            points[2,:] += self.quads[key]['position'][2]
            self.quads[key]['l1'].set_data(points[0,0:2],points[1,0:2])
            self.quads[key]['l1'].set_3d_properties(points[2,0:2])
            self.quads[key]['l2'].set_data(points[0,2:4],points[1,2:4])
            self.quads[key]['l2'].set_3d_properties(points[2,2:4])
            self.quads[key]['hub'].set_data(points[0,5],points[1,5])
            self.quads[key]['hub'].set_3d_properties(points[2,5])
            blit_artists.append(self.quads[key]['l1'])
            blit_artists.append(self.quads[key]['l2'])
            blit_artists.append(self.quads[key]['hub'])
        return tuple(blit_artists)
    
    def frame_iter(self):
        while self.get_time() < self.sim_duration:
            yield
        self.run = False
    
    def animate(self, duration, pause_sim, frame_rate=30):
        self.run = True
        self.pause = False
        self.pause_sim = pause_sim
        self.sim_duration = duration
        frame_delta = 1/frame_rate
        self.anim = animation.FuncAnimation(self.fig, self.update, init_func=self.init_plot,
                                       frames=self.frame_iter, interval=1000*frame_delta,
                                       blit=True)
        if PAUSE_ON_START:
            self.toggle_pause()
        while self.run == True:
            plt.pause(frame_delta)
        self.init_plot()
        self.update()
    
    def close(self):
        plt.close(self.fig)
    
    def toggle_pause(self):
        self.pause = not self.pause
        self.pause_sim(self.pause)
        if self.pause == True:
            self.anim.pause()
        else:
            self.anim.resume()

    def key_press_routine(self,event):
        sys.stdout.flush()
        if event.key == 'p':
            self.toggle_pause()
        if event.key == 'x':
            y = list(self.ax.get_ylim3d())
            y[0] += 0.2
            y[1] += 0.2
            self.ax.set_ylim3d(y)
        elif event.key == 'w':
            y = list(self.ax.get_ylim3d())
            y[0] -= 0.2
            y[1] -= 0.2
            self.ax.set_ylim3d(y)
        elif event.key == 'd':
            x = list(self.ax.get_xlim3d())
            x[0] += 0.2
            x[1] += 0.2
            self.ax.set_xlim3d(x)
        elif event.key == 'a':
            x = list(self.ax.get_xlim3d())
            x[0] -= 0.2
            x[1] -= 0.2
            self.ax.set_xlim3d(x)
    
    def close_routine(self,event):
        self.run = False