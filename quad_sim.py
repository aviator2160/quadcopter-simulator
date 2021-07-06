import dynamics,gui
import signal
import argparse

# Constants
HEADLESS = False
TIME_SCALING = 1.0 # Any positive number(Smaller is faster). 1.0->Real Time, 0.0->Run as fast as possible
PHYSICAL_DYNAMICS_UPDATE = 0.002 # seconds
CONTROLLER_DYNAMICS_UPDATE = 0.005 # seconds
run = True

def Single_Point2Point():
    # Set goals to go to
    GOALS = [{'time': 0,  'position': ( 1, 1,2), 'yaw': 0},
             {'time': 8,  'position': ( 1,-1,4), 'yaw': 3.14},
             {'time': 16, 'position': (-1,-1,2), 'yaw': -1.54},
             {'time': 24, 'position': (-1, 1,4), 'yaw': 1.54}]
    SIM_DURATION = 32 # simulated seconds
    # Define the quadcopters
    QUADCOPTER_DEFS={'q1':{'position':[1,0,4],'orientation':[0,0,0],'L':0.3,'r':0.1,'prop_size':[10,4.5],'mass':1.2}}
    # Controller parameters
    CONTROLLER_DEFS = {'q1':{
                            'Type':'pid_p2p',
                            'Goals':GOALS,
                            'Motor_limits':[4000,9000],
                            'Tilt_limits':[-10,10],
                            'Yaw_Control_Limits':[-900,900],
                            'Z_XY_offset':500,
                            'Linear_PID':{'P':[300,300,7000],'I':[0.04,0.04,4.5],'D':[450,450,5000]},
                            'Linear_To_Angular_Scaler':[1,1,0],
                            'Yaw_Rate_Scaler':0.18,
                            'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
                            'Time_delta':CONTROLLER_DYNAMICS_UPDATE
                            }
                        }
    
    # Make objects for dynamics (quadcopters/controllers) and gui
    dyn_object = dynamics.DynamicsManager(QUAD_DEFS=QUADCOPTER_DEFS, CTRL_DEFS=CONTROLLER_DEFS)
    if not HEADLESS:
        gui_object = gui.GUI(QUAD_DEFS=QUADCOPTER_DEFS, get_data=dyn_object.visual_data, get_time=dyn_object.get_time)
    # Catch Ctrl+C to stop threads
    signal.signal(signal.SIGINT, dyn_object.interrupt_handler)
    # Start the threads
    dyn_object.start_threads(phys_dt=PHYSICAL_DYNAMICS_UPDATE, ctrl_dt=CONTROLLER_DYNAMICS_UPDATE, time_scaling=TIME_SCALING)
    # Update the GUI while switching between destination poitions
    if not HEADLESS:
        gui_object.animate(duration=SIM_DURATION, pause_sim=dyn_object.pause_threads, frame_rate=30)
        gui_object.close()
    # Stop threads once animations are done, and when sim is done
    dyn_object.wait_until_time(SIM_DURATION, check_quit)
    dyn_object.stop_threads()

def Multi_Point2Point():
    # Set goals to go to
    GOALS_1 = [{'time': 0,  'position': (-1,-1,4)},
               {'time': 8,  'position': ( 1, 1,2)}]
    GOALS_2 = [{'time': 0,  'position': ( 1,-1,2)},
               {'time': 10, 'position': (-1, 1,4)}]
    SIM_DURATION = 16 # simulated seconds
    # Define the quadcopters
    QUADCOPTER_DEFS={'q1':{'position':[1,0,4],'orientation':[0,0,0],'L':0.3,'r':0.1,'prop_size':[10,4.5],'mass':1.2},
        'q2':{'position':[-1,0,4],'orientation':[0,0,0],'L':0.15,'r':0.05,'prop_size':[6,4.5],'mass':0.5}}
    # Controller parameters
    CONTROLLER_DEFS={
        'q1':{
            'Type':'pid_p2p',
            'Goals':GOALS_1,
            'Motor_limits':[4000,9000],
            'Tilt_limits':[-10,10],
            'Yaw_Control_Limits':[-900,900],
            'Z_XY_offset':500,
            'Linear_PID':{'P':[300,300,9000],'I':[0.04,0.04,4.5],'D':[450,450,5000]},
            'Linear_To_Angular_Scaler':[1,1,0],
            'Yaw_Rate_Scaler':0.18,
            'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
        }, 
        'q2':{
            'Type':'pid_p2p',
            'Goals':GOALS_2,
            'Motor_limits':[4000,9000],
            'Tilt_limits':[-10,10],
            'Yaw_Control_Limits':[-900,900],
            'Z_XY_offset':500,
            'Linear_PID':{'P':[300,300,12000],'I':[0.04,0.04,4.5],'D':[450,450,5000]},
            'Linear_To_Angular_Scaler':[1,1,0],
            'Yaw_Rate_Scaler':0.18,
            'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
        }
    }

    # Make objects for dynamics (quadcopters/controllers) and gui
    dyn_object = dynamics.DynamicsManager(QUAD_DEFS=QUADCOPTER_DEFS, CTRL_DEFS=CONTROLLER_DEFS)
    if not HEADLESS:
        gui_object = gui.GUI(QUAD_DEFS=QUADCOPTER_DEFS, get_data=dyn_object.visual_data, get_time=dyn_object.get_time)
    # Catch Ctrl+C to stop threads
    signal.signal(signal.SIGINT, dyn_object.interrupt_handler)
    # Start the threads
    dyn_object.start_threads(phys_dt=PHYSICAL_DYNAMICS_UPDATE, ctrl_dt=CONTROLLER_DYNAMICS_UPDATE, time_scaling=TIME_SCALING)
    # Update the GUI while switching between destination poitions
    if not HEADLESS:
        gui_object.animate(duration=SIM_DURATION, pause_sim=dyn_object.pause_threads, frame_rate=30)
        gui_object.close()
    # Stop threads once animations are done, and when sim is done
    dyn_object.wait_until_time(SIM_DURATION, check_quit)
    dyn_object.stop_threads()

def Single_Velocity():
    # Set goals to go to
    GOALS = [{'time': 0,  'position': ( 0.5,   0,2)},
             {'time': 5,  'position': (   0, 0.5,2)},
             {'time': 10, 'position': (-0.5,   0,2)},
             {'time': 15, 'position': (   0,-0.5,2)}]
    SIM_DURATION = 20 # simulated seconds
    # Define the quadcopters
    QUADCOPTER_DEFS={'q1':{'position':[0,0,0],'orientation':[0,0,0],'L':0.3,'r':0.1,'prop_size':[10,4.5],'mass':1.2}}
    # Controller parameters
    CONTROLLER_DEFS={'q1':{
                            'Type':'pid_velocity',
                            'Goals':GOALS,
                            'Motor_limits':[4000,9000],
                            'Tilt_limits':[-10,10],
                            'Yaw_Control_Limits':[-900,900],
                            'Z_XY_offset':500,
                            'Linear_PID':{'P':[2000,2000,7000],'I':[0.25,0.25,4.5],'D':[50,50,5000]},
                            'Linear_To_Angular_Scaler':[1,1,0],
                            'Yaw_Rate_Scaler':0.18,
                            'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
                            }
                        }

    # Make objects for dynamics (quadcopters/controllers) and gui
    dyn_object = dynamics.DynamicsManager(QUAD_DEFS=QUADCOPTER_DEFS, CTRL_DEFS=CONTROLLER_DEFS)
    gui_object = gui.GUI(QUAD_DEFS=QUADCOPTER_DEFS, get_data=dyn_object.visual_data, get_time=dyn_object.get_time)
    # Catch Ctrl+C to stop threads
    signal.signal(signal.SIGINT, dyn_object.interrupt_handler)
    # Start the threads
    if not HEADLESS:
        dyn_object.start_threads(phys_dt=PHYSICAL_DYNAMICS_UPDATE, ctrl_dt=CONTROLLER_DYNAMICS_UPDATE, time_scaling=TIME_SCALING)
    # Update the GUI while switching between destination poitions
    if not HEADLESS:
        gui_object.animate(duration=SIM_DURATION, pause_sim=dyn_object.pause_threads, frame_rate=30)
        gui_object.close()
    # Stop threads once animations are done, and when sim is done
    dyn_object.wait_until_time(SIM_DURATION, check_quit)
    dyn_object.stop_threads()

def parse_args():
    parser = argparse.ArgumentParser(description="Quadcopter Simulator")
    parser.add_argument("--headless", help='Run without GUI', action='store_true')
    parser.add_argument("--sim", help='single_p2p, multi_p2p or single_velocity', default='multi_p2p')
    parser.add_argument("--time_scale", type=float, default=-1.0, help='Time scaling factor. 0.0:fastest,1.0:realtime,>1:slow, ex: --time_scale 0.1')
    parser.add_argument("--quad_update_time", type=float, default=0.0, help='delta time for quadcopter dynamics update(seconds), ex: --quad_update_time 0.002')
    parser.add_argument("--controller_update_time", type=float, default=0.0, help='delta time for controller update(seconds), ex: --controller_update_time 0.005')
    return parser.parse_args()

def check_quit():
    global run
    return not run

if __name__ == "__main__":
    args = parse_args()
    if args.headless: HEADLESS = True
    if args.time_scale>=0: TIME_SCALING = args.time_scale
    if args.quad_update_time>0: QUAD_DYNAMICS_UPDATE = args.quad_update_time
    if args.controller_update_time>0: CONTROLLER_DYNAMICS_UPDATE = args.controller_update_time
    if args.sim == 'single_p2p':
        Single_Point2Point()
    elif args.sim == 'multi_p2p':
        Multi_Point2Point()
    elif args.sim == 'single_velocity':
        Single_Velocity()
