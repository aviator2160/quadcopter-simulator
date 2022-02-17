# -*- coding: utf-8 -*-
"""Runs a nonlinear simulation of quadcopters carrying dangling payloads.

For information about required and optional command-line arguments, run the
the script with the -h or --help flag.

Created on Sun Jul  4 16:49:33 2021

@author: aviat
"""

from physics_manager import PhysicsManager
import gui
import scene_defs as scenes

import signal
import argparse

# Constants

DEFAULT_SCENE = 'multi_slung_lqr_team'
DEFAULT_HEADLESS = False
DEFAULT_TIME_SCALE = 4.0  # Any positive number(Smaller is faster).
                            # 1.0->Real Time, 0.0->Run as fast as possible
DEFAULT_DYNAMICS_TIMESTEP = 0.01  # seconds

def run_sim(params, headless, time_scale, timestep):
    """Runs a simulation with the given parameters.

    This function simulates the nonlinear dynamics of quadcopters carrying
    dangling payloads, with various possible controllers.

    Args:
        params:
            A dict mapping keys  -> collections of definitions of objects of
            some type. Each such collection (for example, the definitions of
            all quadcopters in the scene) is itself a dict mapping scene object
            names -> their definitions.
            
            The keys and their associated collections are:

            'QUADCOPTER_DEFS': Defines all quadcopters in the scene.
            'PAYLOAD_DEFS':    Defines all payloads in the scene.
            'CABLE_DEFS':      Defines all cables linking quadcopters to payloads.
            'CONTROLLER_DEFS:' Defines controllers of individual quadcopters.
            'PLANNER_DEFS:'    Defines team-wide controllers of one or several
                                 quadcopters carrying payloads collectively.
        headless:
            If true, the simulation runs in "headless mode": the GUI is not
            shown and the simulation occurs in the background. Headless mode
            has better performance than with the GUI enabled.
        time_scale:
            The scale factor between simulated time and real time. If less
            than 1, the simulation runs faster than real time. If greater than
            1, it runs slower than real time. For instance, time_scale=1.5
            means that it takes 1.5 seconds of real time for 1 simulated second
            to elapse.
        timestep:
            The length of time between each physics update, in seconds.
            For example, timestep=0.01 means updates occur every 0.01 seconds,
            corresponding to 100 Hz.
    """

    # Get object definitions if they exist, else return empty dict
    quads  = params.get('QUADCOPTER_DEFS', {})
    loads  = params.get('PAYLOAD_DEFS',    {})
    cables = params.get('CABLE_DEFS',      {})
    ctrls  = params.get('CONTROLLER_DEFS', {})
    planners = params.get('PLANNER_DEFS',  {})
    
    phys = PhysicsManager(quads, loads, cables, ctrls, planners)
    if not headless:
        sim_view = gui.Sim_GUI(quads, loads, cables,
                               get_data=phys.visual_data,
                               get_time=phys.get_time)
    signal.signal(signal.SIGINT, phys.on_keyboard_interrupt) # Catch Ctrl+C to stop threads
    phys.start_threads(dt=timestep, time_scale=time_scale)
    # In headless mode, sim runs in background with no GUI
    if headless:
        phys.wait_until_time(params['SIM_DURATION'])
    else:
        sim_view.animate(duration=params['SIM_DURATION'],
                         pause_sim=phys.pause_threads, frame_rate=30)
        sim_view.close()
    # Stop threads once animations are done, and when sim is done
    phys.stop_threads()

if __name__ == '__main__':
    """ Main script to run simulation with command-line arguments.

    Command-line args:
        --scene: Name of the scene to simulate. See scene_defs.py for list
                 of implemented scenes.
        --headless: Runs simulation in headless mode if set. See run_sim()
                    docstring, 'headless' arg for more information.
        --time_scale: Time scale factor. See run_sim() docstring,
            'time_scale' arg for more information.
        --timestep: Physics timestep. See run_sim() docstring, 'timestep' arg
                    for more information.
    
    Raises:
        ValueError: One of the command-line arguments was passed an invalid
                    value.
    """

    parser = argparse.ArgumentParser(description='Quadcopter Simulator')
    parser.add_argument(
        '--scene', choices=scenes.defs.keys(), default=DEFAULT_SCENE,
        help=('Choose one of the following scenes: '
              ', '.join(scenes.defs.keys()))) # Prints all scene names
    parser.add_argument(
        '--headless', action='store_true', default=DEFAULT_HEADLESS,
        help='Run without GUI if set')
    parser.add_argument(
        '--time_scale', type=float, default=DEFAULT_TIME_SCALE,
        help=('Time scale factor. <1: Fast-motion. 1.0: Real-time. '
              '>1: Slow-motion. 0: as fast as possible. '
              'Must be non-negative. Ex: --time_scale 0.5'))
    parser.add_argument(
        '--timestep', type=float, default=DEFAULT_DYNAMICS_TIMESTEP,
        help=('Time delta for physics update (seconds). '
              'Ex: for 100 Hz physics updates, use --quad_update_time 0.01'))
    args = parser.parse_args()
    if args.time_scale < 0:
        raise ValueError('Simulation time scale must be non-negative!')
    if args.timestep < 0:
        raise ValueError('Simulation timestep must be non-negative!')
    scene = scenes.defs.get(args.scene)
    if scene == None:
        raise ValueError(str(args.sim) + \
            ' is not a recognized simulation scene name!')
    run_sim(scene, args.headless, args.time_scale, args.timestep)
