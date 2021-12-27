# -*- coding: utf-8 -*-
"""
Created on Sun Jul  4 16:49:33 2021

@author: aviat
"""

from physics_manager import PhysicsManager
import gui
import sim_defs as sims

import signal
import argparse

# Constants

DEFAULT_SIM = 'single_lqr_p2p'
HEADLESS = False
TIME_SCALING = 1.0 # Any positive number(Smaller is faster). 1.0->Real Time, 0.0->Run as fast as possible
PHYSICAL_DYNAMICS_UPDATE = 0.01 # seconds

def run_sim(params):
    # get object definitions if they exist, else return empty dict
    quads  = params.get('QUADCOPTER_DEFS',{})
    loads  = params.get('PAYLOAD_DEFS',{})
    cables = params.get('CABLE_DEFS',{})
    ctrls  = params.get('CONTROLLER_DEFS',{})
    # Make objects for dynamics (quadcopters/controllers) and gui
    phys = PhysicsManager(quads,loads,cables,ctrls)
    if not HEADLESS:
        sim_view = gui.Sim_GUI(quads,loads,cables,get_data=phys.visual_data,get_time=phys.get_time)
    # Catch Ctrl+C to stop threads
    signal.signal(signal.SIGINT, phys.on_keyboard_interrupt)
    # Start the threads
    phys.start_threads(dt=PHYSICAL_DYNAMICS_UPDATE, time_scale=TIME_SCALING)
    # Update the GUI while switching between destination poitions
    if HEADLESS:
        phys.wait_until_time(params['SIM_DURATION'])
    else:
        sim_view.animate(duration=params['SIM_DURATION'], pause_sim=phys.pause_threads, frame_rate=30)
        sim_view.close()
    # Stop threads once animations are done, and when sim is done
    phys.stop_threads()

def parse_args():
    parser = argparse.ArgumentParser(description="Quadcopter Simulator")
    parser.add_argument("--headless", help='Run without GUI', action='store_true')
    parser.add_argument("--sim", help=', '.join(sims.defs.keys()), default=DEFAULT_SIM)
    parser.add_argument("--time_scale", type=float, default=-1.0, help='Time scaling factor. <1: fast-motion, 1.0: real time, >1: slow-motion, ex: --time_scale 0.5')
    parser.add_argument("--phys_timestep", type=float, default=0.0, help='delta time for dynamics update (seconds), ex: --quad_update_time 0.01')
    # parser.add_argument("--controller_update_time", type=float, default=0.0, help='delta time for controller update(seconds), ex: --controller_update_time 0.005')
    return parser.parse_args()

class SimTypeNotFoundError(Exception):
    pass

if __name__ == "__main__":
    args = parse_args()
    if args.headless: HEADLESS = True
    if args.time_scale >= 0: TIME_SCALING = args.time_scale
    if args.phys_timestep > 0: QUAD_DYNAMICS_UPDATE = args.phys_timestep
    # if args.controller_update_time > 0: CONTROLLER_DYNAMICS_UPDATE = args.controller_update_time
    sim_def = sims.defs.get(args.sim)
    if sim_def == None:
        raise SimTypeNotFoundError(str(args.sim) + " is not a recognized simulation name!")
    run_sim(sim_def)
        