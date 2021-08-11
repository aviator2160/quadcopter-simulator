"""
Definitions of all simulators

@author: Avi Mittal
"""

import numpy as np

Slung_LQR_P2P = {
    'SIM_DURATION': 32, # simulated seconds
    # Define the quadcopters
    'QUADCOPTER_DEFS':{
        'q1':{
            'position':[-2,0,4],'orientation':[0,0,0],
            'L': 0.3,'r': 0.1,'mass': 1.0,
            'prop_size':[10,4.5],'prop_torque_coeff': 0.0245
            },
        },
    'PAYLOAD_DEFS':{
        'p1':{
            'position':[-2,0,2],'orientation':[0,0,0],'x': 0.4,'y': 0.4,'z': 0.2,'mass': 1.0,
            # 'hardpoints':[[0.2,0.2,0],[-0.2,0.2,0],[-0.2,-0.2,0],[0.2,-0.2,0]]}
            'hardpoints':[[0.3,0,0],[0,0.3,0],[-0.3,0,0],[0,-0.3,0]]
            },
        },
    # Controller parameters
    'CONTROLLER_DEFS':{
        'q1':{
            'Type':'lqr_p2p',
            'Goals':[
                {'time': 0,  'position': ( 1, 1,2), 'yaw': 0},
                {'time': 8,  'position': ( 1,-1,4), 'yaw': 3.14},
                {'time': 16, 'position': (-1,-1,2), 'yaw': -1.54},
                {'time': 24, 'position': (-1, 1,4), 'yaw': 1.54}
                ],
            'Thrust_limits':[0,10],
            'Q': np.diag([1,1,50, 1,1,1, 1,1,0, 1,1,0.1]),
            'R': np.eye(4) * 0.1
            },
        },
    'CABLE_DEFS':{
        'c1':{'quad':'q1','load':'p1','hardpoint': 0,'stiffness': 100,'damping': 1},
        'c2':{'quad':'q1','load':'p1','hardpoint': 1,'stiffness': 100,'damping': 1},
        'c3':{'quad':'q1','load':'p1','hardpoint': 2,'stiffness': 100,'damping': 1},
        'c4':{'quad':'q1','load':'p1','hardpoint': 3,'stiffness': 100,'damping': 1},
        },
    }

Slung_PID_P2P = {
    'SIM_DURATION': 10, # simulated seconds
    # Define the quadcopters
    'QUADCOPTER_DEFS':{
        'q1':{
            'position':[-2,0,4],'orientation':[0,0,0],
            'L': 0.3,'r': 0.1,'mass': 1.0,
            'prop_size':[10,4.5],'prop_torque_coeff': 0.0245
            },
        },
    'PAYLOAD_DEFS':{
        'p1':{
            'position':[-2,0,2],'orientation':[0,0,0],'x': 0.4,'y': 0.4,'z': 0.2,'mass': 1.0,
            # 'hardpoints':[[0.2,0.2,0],[-0.2,0.2,0],[-0.2,-0.2,0],[0.2,-0.2,0]]}
            'hardpoints':[[0.3,0,0],[0,0.3,0],[-0.3,0,0],[0,-0.3,0]]
            },
        },
    # Controller parameters
    'CONTROLLER_DEFS':{
        'q1':{
            'Type':'pid_p2p',
            'Goals':[{'time': 0, 'position':(1,0,5)}],
            'Motor_limits':[3000,9000],
            'Tilt_limits':[-10,10],
            'Yaw_Control_Limits':[-900,900],
            'Z_XY_offset': 500,
            'Linear_PID':{'P':[0.5,0.5,4000],'I':[0.01,0.01,2400],'D':[0.65,0.65,3000]},
            'Linear_To_Angular_Scaler':[0.1,0.1,0],
            'Yaw_Rate_Scaler': 0.18,
            'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
            },
        },
    'CABLE_DEFS':{
        'c1':{'quad':'q1','load':'p1','hardpoint': 0,'stiffness': 100,'damping': 1},
        'c2':{'quad':'q1','load':'p1','hardpoint': 1,'stiffness': 100,'damping': 1},
        'c3':{'quad':'q1','load':'p1','hardpoint': 2,'stiffness': 100,'damping': 1},
        'c4':{'quad':'q1','load':'p1','hardpoint': 3,'stiffness': 100,'damping': 1},
        },
    }

Single_LQR_P2P = {
    'SIM_DURATION': 32, # simulated seconds
    # Define the quadcopters
    'QUADCOPTER_DEFS':{
        'q1':{
            'position':[1,0,4],'orientation':[0,0.5,0],
            'L': 0.3,'r': 0.1,'mass': 1.2,
            'prop_size':[10,4.5],'prop_torque_coeff': 0.0245
            },
        },
    # Controller parameters
    'CONTROLLER_DEFS':{
        'q1':{
            'Type':'lqr_p2p',
            'Goals':[
                {'time': 0,  'position': ( 1, 1,2), 'yaw': 0},
                {'time': 8,  'position': ( 1,-1,4), 'yaw': 3.14},
                {'time': 16, 'position': (-1,-1,2), 'yaw': -1.54},
                {'time': 24, 'position': (-1, 1,4), 'yaw': 1.54}
                ],
            'Thrust_limits':[0,10],
            'Q': np.diag([1,1,20, 1,1,1, 1,1,0, 1,1,0.1]),
            'R': np.eye(4) * 0.1
            },
        },
    }

Single_PID_P2P = {
    'SIM_DURATION': 32, # simulated seconds
    # Define the quadcopters
    'QUADCOPTER_DEFS':{
        'q1':{
            'position':[1,0,4],'orientation':[0,0,0],
            'L': 0.3,'r': 0.1,'mass': 1.2,
            'prop_size':[10,4.5],'prop_torque_coeff': 0.0245
            },
        },
    # Controller parameters
    'CONTROLLER_DEFS':{
        'q1':{
            'Type':'pid_p2p',
            'Goals':[
                {'time': 0,  'position': ( 1, 1,2), 'yaw': 0},
                {'time': 8,  'position': ( 1,-1,4), 'yaw': 0.5},
                {'time': 16, 'position': (-1,-1,2), 'yaw': 1},
                {'time': 24, 'position': (-1, 1,4), 'yaw': 0.5}
                ],
            'Motor_limits':[3000,9000],
            'Tilt_limits':[-10,10],
            'Yaw_Control_Limits':[-900,900],
            'Z_XY_offset': 500,
            'Linear_PID':{'P':[0.5,0.5,20_000],'I':[0.01,0.01,20_000],'D':[0.65,0.65,10_000]},
            'Linear_To_Angular_Scaler':[1,1,0],
            'Yaw_Rate_Scaler': 0.18,
            'Angular_PID':{'P':[22_000,22_000,1500],'I':[0,0,1.2],'D':[12_000,12_000,0]},
            },
        },
    }

Multi_PID_P2P = {
    'SIM_DURATION': 16, # simulated seconds
    # Define the quadcopters
    'QUADCOPTER_DEFS':{
        'q1':{
            'position':[1,0,4],'orientation':[0,0,0],
            'L': 0.3,'r': 0.1,'mass': 1.2,
            'prop_size':[10,4.5],'prop_torque_coeff': 0.0245
            },
        'q2':{
            'position':[-1,0,4],'orientation':[0,0,0],
            'L': 0.15,'r': 0.05,'mass': 0.5,
            'prop_size':[6,4.5],'prop_torque_coeff': 0.0245,
            },
        },
    # Controller parameters
    'CONTROLLER_DEFS':{
        'q1':{
            'Type':'pid_p2p',
            'Goals':[
                {'time': 0, 'position': (-1,-1,4)},
                {'time': 8, 'position': ( 1, 1,2)}
            ],
            'Motor_limits':[3000,9000],
            'Tilt_limits':[-10,10],
            'Yaw_Control_Limits':[-900,900],
            'Z_XY_offset': 500,
            'Linear_PID':{'P':[0.5,0.5,20_000],'I':[0.01,0.000_1,20_000],'D':[0.65,0.65,10_000]},
            'Linear_To_Angular_Scaler':[1,1,0],
            'Yaw_Rate_Scaler': 0.18,
            'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
            }, 
        'q2':{
            'Type':'pid_p2p',
            'Goals':[
                {'time': 0,  'position': ( 1,-1,2)},
                {'time': 10, 'position': (-1, 1,4)}
            ],
            'Motor_limits':[3000,9000],
            'Tilt_limits':[-10,10],
            'Yaw_Control_Limits':[-900,900],
            'Z_XY_offset': 500,
            'Linear_PID':{'P':[0.5,0.5,12_000],'I':[0.01,0.01,12_000],'D':[0.65,0.65,12_000]},
            'Linear_To_Angular_Scaler':[1,1,0],
            'Yaw_Rate_Scaler': 0.18,
            'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
            },
        },
    }

defs = {
    'slung_lqr_p2p':  Slung_LQR_P2P,
    'slung_pid_p2p':  Slung_PID_P2P,
    'single_lqr_p2p': Single_LQR_P2P,
    'single_pid_p2p': Single_PID_P2P,
    'multi_pid_p2p':  Multi_PID_P2P,
    }