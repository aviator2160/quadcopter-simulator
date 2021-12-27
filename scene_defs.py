"""
Definitions of all simulator scenes

@author: Avi Mittal
"""

import numpy as np

Multi_Slung_LQR_P2P = {
    'SIM_DURATION': 8, # simulated seconds
    # Define the quadcopters
    'QUADCOPTER_DEFS':{
        'q1':{'position':[-1, 1,4],},
        'q2':{'position':[-3, 1,4],},
        'q3':{'position':[-3,-1,4],},
        'q4':{'position':[-1,-1,4],},
        },
    'PAYLOAD_DEFS':{
        'p1':{
            'position':[-2,0,2],'orientation':[0,0,0],'x': 0.4,'y': 0.4,'z': 0.2,'mass': 4.0,
            'hardpoints':[[0.2,0.2,0],[-0.2,0.2,0],[-0.2,-0.2,0],[0.2,-0.2,0]]
            },
        },
    # Controller parameters
    'CONTROLLER_DEFS':{
        'q1':{'type':'lqr_p2p','goals':[{'time': 0,'position':(3, 2,5)}],},
        'q2':{'type':'lqr_p2p','goals':[{'time': 0,'position':(-1, 2,5)}],},
        'q3':{'type':'lqr_p2p','goals':[{'time': 0,'position':(-1,-2,5)}],},
        'q4':{'type':'lqr_p2p','goals':[{'time': 0,'position':(3,-2,5)}],},
        },
    'CABLE_DEFS':{
        'c1':{'quad':'q1','load':'p1','hardpoint': 0,'stiffness': 100,'damping': 1},
        'c2':{'quad':'q2','load':'p1','hardpoint': 1,'stiffness': 100,'damping': 1},
        'c3':{'quad':'q3','load':'p1','hardpoint': 2,'stiffness': 100,'damping': 1},
        'c4':{'quad':'q4','load':'p1','hardpoint': 3,'stiffness': 100,'damping': 1},
        },
    }

Multi_Slung_PID_P2P = {
    'SIM_DURATION': 8, # simulated seconds
    # Define the quadcopters
    'QUADCOPTER_DEFS':{
        'q1':{'position':[-1, 1,4],},
        'q2':{'position':[-3, 1,4],},
        'q3':{'position':[-3,-1,4],},
        'q4':{'position':[-1,-1,4],},
        },
    'PAYLOAD_DEFS':{
        'p1':{
            'position':[-2,0,2],'orientation':[0,0,0],'x': 0.4,'y': 0.4,'z': 0.2,'mass': 4.0,
            'hardpoints':[[0.2,0.2,0],[-0.2,0.2,0],[-0.2,-0.2,0],[0.2,-0.2,0]]
            },
        },
    # Controller parameters
    'CONTROLLER_DEFS':{
        'q1':{'type':'pid_p2p','goals':[{'time': 0,'position':(2, 1,5)}],},
        'q2':{'type':'pid_p2p','goals':[{'time': 0,'position':(0, 1,5)}],},
        'q3':{'type':'pid_p2p','goals':[{'time': 0,'position':(0,-1,5)}],},
        'q4':{'type':'lqr_p2p','goals':[{'time': 0,'position':(2,-1,5)}],},
        },
    'CABLE_DEFS':{
        'c1':{'quad':'q1','load':'p1','hardpoint': 0,'stiffness': 100,'damping': 1},
        'c2':{'quad':'q2','load':'p1','hardpoint': 1,'stiffness': 100,'damping': 1},
        'c3':{'quad':'q3','load':'p1','hardpoint': 2,'stiffness': 100,'damping': 1},
        'c4':{'quad':'q4','load':'p1','hardpoint': 3,'stiffness': 100,'damping': 1},
        },
    }

Slung_Both_P2P = {
    'SIM_DURATION': 32, # simulated seconds
    # Define the quadcopters
    'QUADCOPTER_DEFS':{
        'q1':{'position':[0, 0.5,4],},
        'q2':{'position':[0,-0.5,4],},
        },
    'PAYLOAD_DEFS':{
        'p1':{
            'position':[0,0.5,2],'orientation':[0,0,0],'x': 0.4,'y': 0.4,'z': 0.2,'mass': 1.0,
            'hardpoints':[[0.3,0,0],[0,0.3,0],[-0.3,0,0],[0,-0.3,0]]
            },
        'p2':{
            'position':[0,-0.5,2],'orientation':[0,0,0],'x': 0.4,'y': 0.4,'z': 0.2,'mass': 1.0,
            'hardpoints':[[0.3,0,0],[0,0.3,0],[-0.3,0,0],[0,-0.3,0]]
            },
        },
    # Controller parameters
    'CONTROLLER_DEFS':{
        'q1':{
            'type':'lqr_p2p',
            'goals':[
                {'time': 0,  'position': ( 1, 1.5,2), 'yaw': 0},
                {'time': 8,  'position': ( 1,-0.5,4), 'yaw': 3.14},
                {'time': 16, 'position': (-1,-0.5,2), 'yaw': -1.54},
                {'time': 24, 'position': (-1, 1.5,4), 'yaw': 1.54}
                ],
            'Q': np.diag([2,2,250, 5,5,20, 0.1,0.1,0.1, 1,1,0.1]),
            },
        'q2':{
            'type':'pid_p2p',
            'goals':[
                {'time': 0,  'position': ( 1, 0.5,2), 'yaw': 0},
                {'time': 8,  'position': ( 1,-1.5,4), 'yaw': 3.14},
                {'time': 16, 'position': (-1,-1.5,2), 'yaw': -1.54},
                {'time': 24, 'position': (-1, 0.5,4), 'yaw': 1.54}
                ],
            'Linear_PID':{'P':[1,1,5],'I':[0.01,0.01,0.3],'D':[2.5,2.5,2.5]},
            'Angular_PID':{'P':[15,15,8],'I':[0.01,0.1,0.1],'D':[4,4,15]},
            },
        },
    'CABLE_DEFS':{
        'c11':{'quad':'q1','load':'p1','hardpoint': 0,'stiffness': 100,'damping': 1},
        'c12':{'quad':'q1','load':'p1','hardpoint': 1,'stiffness': 100,'damping': 1},
        'c13':{'quad':'q1','load':'p1','hardpoint': 2,'stiffness': 100,'damping': 1},
        'c14':{'quad':'q1','load':'p1','hardpoint': 3,'stiffness': 100,'damping': 1},
        'c21':{'quad':'q2','load':'p2','hardpoint': 0,'stiffness': 100,'damping': 1},
        'c22':{'quad':'q2','load':'p2','hardpoint': 1,'stiffness': 100,'damping': 1},
        'c23':{'quad':'q2','load':'p2','hardpoint': 2,'stiffness': 100,'damping': 1},
        'c24':{'quad':'q2','load':'p2','hardpoint': 3,'stiffness': 100,'damping': 1},
        },
    }


Slung_LQR_P2P = {
    'SIM_DURATION': 32, # simulated seconds
    # Define the quadcopters
    'QUADCOPTER_DEFS':{
        'q1':{'position':[0,0,4],},
        },
    'PAYLOAD_DEFS':{
        'p1':{
            'position':[0,0,2],'orientation':[0,0,0],'x': 0.4,'y': 0.4,'z': 0.2,'mass': 1.0,
            'hardpoints':[[0.3,0,0],[0,0.3,0],[-0.3,0,0],[0,-0.3,0]]
            },
        },
    # Controller parameters
    'CONTROLLER_DEFS':{
        'q1':{
            'type':'lqr_p2p',
            'goals':[
                {'time': 0,  'position': ( 1, 1,2), 'yaw': 0},
                {'time': 8,  'position': ( 1,-1,4), 'yaw': 3.14},
                {'time': 16, 'position': (-1,-1,2), 'yaw': -1.54},
                {'time': 24, 'position': (-1, 1,4), 'yaw': 1.54}
                ],
            'Q': np.diag([2,2,250, 4,4,20, 0.1,0.1,0.1, 1,1,0.1]),
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
    'SIM_DURATION': 32, # simulated seconds
    # Define the quadcopters
    'QUADCOPTER_DEFS':{
        'q1':{'position':[0,0,4],},
        },
    'PAYLOAD_DEFS':{
        'p1':{
            'position':[0,0,2],'orientation':[0,0,0],'x': 0.4,'y': 0.4,'z': 0.2,'mass': 1.0,
            'hardpoints':[[0.3,0,0],[0,0.3,0],[-0.3,0,0],[0,-0.3,0]]
            },
        },
    # Controller parameters
    'CONTROLLER_DEFS':{
        'q1':{
            'type':'pid_p2p',
            'goals':[
                {'time': 0,  'position': ( 1, 1,2), 'yaw': 0},
                {'time': 8,  'position': ( 1,-1,4), 'yaw': 3.14},
                {'time': 16, 'position': (-1,-1,2), 'yaw': -1.54},
                {'time': 24, 'position': (-1, 1,4), 'yaw': 1.54}
                ],
            'Linear_PID':{'P':[1,1,5],'I':[0.01,0.01,0.3],'D':[1.8,1.8,2.0]},
            'Angular_PID':{'P':[15,15,8],'I':[0.1,0.1,0.1],'D':[4,4,15]},
            },
        },
    'CABLE_DEFS':{
        'c1':{'quad':'q1','load':'p1','hardpoint': 0,'stiffness': 100,'damping': 1},
        'c2':{'quad':'q1','load':'p1','hardpoint': 1,'stiffness': 100,'damping': 1},
        'c3':{'quad':'q1','load':'p1','hardpoint': 2,'stiffness': 100,'damping': 1},
        'c4':{'quad':'q1','load':'p1','hardpoint': 3,'stiffness': 100,'damping': 1},
        },
    }

Multi_LQR_P2P = {
    'SIM_DURATION': 16, # simulated seconds
    # Define the quadcopters
    'QUADCOPTER_DEFS':{
        'q1':{
            'position':[1,0,4],'orientation':[0,0,0],
            'L': 0.5,'r': 0.15,'mass': 2.0,
            'prop_size':[13,4.5],
            'thrust_limits':[-20,20],
            },
        'q2':{
            'position':[-1,0,4],'orientation':[0,0,0],
            'L': 0.15,'r': 0.04,'mass': 0.5,
            'prop_size':[6,4.5],
            'thrust_limits':[-4,4],
            },
        },
    # Controller parameters
    'CONTROLLER_DEFS':{
        'q1':{
            'type':'lqr_p2p',
            'goals':[
                {'time': 0, 'position': (-1,-1,4)},
                {'time': 8, 'position': ( 1, 1,2)}
                ],
            'thrust_limits':[-10,10],
            'R': np.diag([1, 1,1,1]),
            'offset_gravity': 1,
            }, 
        'q2':{
            'type':'lqr_p2p',
            'goals':[
                {'time': 0,  'position': ( 1,-1,2)},
                {'time': 10, 'position': (-1, 1,4)}
                ],
            'thrust_limits':[-4,4],
            'R': np.diag([1, 3,3,3]),
            'offset_gravity': 1,
            },
        },
    }

Multi_PID_P2P = {
    'SIM_DURATION': 16, # simulated seconds
    # Define the quadcopters
    'QUADCOPTER_DEFS':{
        'q1':{
            'position':[1,0,4],'orientation':[0,0,0],
            'L': 0.5,'r': 0.15,'mass': 2.0,
            'prop_size':[13,4.5],
            'thrust_limits':[-20,20],
            },
        'q2':{
            'position':[-1,0,4],'orientation':[0,0,0],
            'L': 0.15,'r': 0.04,'mass': 0.5,
            'prop_size':[6,4.5],
            'thrust_limits':[-4,4],
            },
        },
    # Controller parameters
    'CONTROLLER_DEFS':{
        'q1':{
            'type':'pid_p2p',
            'goals':[
                {'time': 0, 'position': (-1,-1,4), 'yaw': 0},
                {'time': 8, 'position': ( 1, 1,2), 'yaw': 3.14}
                ],
            'Linear_PID':{'P':[1,1,5],'I':[0.01,0.01,0.03],'D':[1.8,1.8,2.0]},
            'Angular_PID':{'P':[35,35,8],'I':[0.1,0.1,0.1],'D':[10,10,20]},
            }, 
        'q2':{
            'type':'pid_p2p',
            'goals':[
                {'time': 0,  'position': ( 1,-1,2), 'yaw': 0},
                {'time': 10, 'position': (-1, 1,4), 'yaw': 3.14}
                ],
            'Linear_PID':{'P':[1,1,3],'I':[0.01,0.01,0.03],'D':[1.8,1.8,1.2]},
            'Angular_PID':{'P':[15,15,8],'I':[0.1,0.1,0.1],'D':[2,2,15]},
            },
        },
    }

Both_P2P = {
    'SIM_DURATION': 32, # simulated seconds
    # Define the quadcopters
    'QUADCOPTER_DEFS':{
        'q1':{'position':[0, 0.5,4],},
        'q2':{'position':[0,-0.5,4],},
        },
    # Controller parameters
    'CONTROLLER_DEFS':{
        'q1':{
            'type':'lqr_p2p',
            'goals':[
                {'time': 0,  'position': ( 1, 1.5,2), 'yaw': 0},
                {'time': 8,  'position': ( 1,-0.5,4), 'yaw': 3.14},
                {'time': 16, 'position': (-1,-0.5,2), 'yaw': -1.54},
                {'time': 24, 'position': (-1, 1.5,4), 'yaw': 1.54}
                ],
            'Q': np.diag([2,2,30, 1,1,5, 10,10,0.1, 1,1,0.1]),
            'R': np.diag([0.1, 0.1,0.1,0.2]),
            'offset_gravity': 1, # Fraction of quad weight
            },
        'q2':{
            'type':'pid_p2p',
            'goals':[
                {'time': 0,  'position': ( 1, 0.5,2), 'yaw': 0},
                {'time': 8,  'position': ( 1,-1.5,4), 'yaw': 3.14},
                {'time': 16, 'position': (-1,-1.5,2), 'yaw': -1.54},
                {'time': 24, 'position': (-1, 0.5,4), 'yaw': 1.54}
                ],
            'Linear_PID':{'P':[1,1,3],'I':[0.1,0.1,0.3],'D':[1.8,1.8,1.2]},
            'Angular_PID':{'P':[15,15,8],'I':[1,1,0.1],'D':[2,2,15]},
            'offset_gravity': 1, # Fraction of quad weight
            },
        },
    }

Single_LQR_P2P = {
    'SIM_DURATION': 32, # simulated seconds
    # Define the quadcopters
    'QUADCOPTER_DEFS':{
        'q1':{
            'position':[0,0,4],'orientation':[0,0,0],
            # 'position':[0,0,4],'orientation':[1.5,0,0],
            },
        },
    # Controller parameters
    'CONTROLLER_DEFS':{
        'q1':{
            'type':'lqr_p2p',
            'goals':[
                {'time': 0,  'position': ( 1, 1,2), 'yaw': 0},
                {'time': 8,  'position': ( 1,-1,4), 'yaw': 3.14},
                {'time': 16, 'position': (-1,-1,2), 'yaw': -1.54},
                {'time': 24, 'position': (-1, 1,4), 'yaw': 1.54}
                ],
            'offset_gravity':0, # Fraction of quad weight
            'Q': np.diag([1,1,50, 1,1,5, 10,10,0.1, 1,1,0.1]),
            'R': np.eye(4) * 0.1
            },
        },
    }

Single_PID_P2P = {
    'SIM_DURATION': 32, # simulated seconds
    # Define the quadcopters
    'QUADCOPTER_DEFS':{
        'q1':{
            'position':[0,0,4],'orientation':[0,0,0],
            # 'position':[0,0,4],'orientation':[1.5,0,0],
            },
        },
    # Controller parameters
    'CONTROLLER_DEFS':{
        'q1':{
            'type':'pid_p2p',
            'goals':[
                {'time': 0,  'position': ( 1, 1,2), 'yaw': 0},
                {'time': 8,  'position': ( 1,-1,4), 'yaw': 3.14},
                {'time': 16, 'position': (-1,-1,2), 'yaw': -1.54},
                {'time': 24, 'position': (-1, 1,4), 'yaw': 1.54}
                ],
            },
        },
    }

Test = {
    'SIM_DURATION': 8, # simulated seconds
    # Define the quadcopters
    'QUADCOPTER_DEFS':{
        'q1':{'position':[-1, 1,4],},
        'q2':{'position':[-3, 1,4],},
        },
    # Controller parameters
    'CONTROLLER_DEFS':{
        'q1':{'type':'lqr_p2p','goals':[{'time': 0,'position':(2, 1,5)}],},
        'q2':{'type':'lqr_p2p','goals':[{'time': 0,'position':(0, 1,5)}],},
        },
    
    # 'SIM_DURATION': 15, # simulated seconds
    # # Define the quadcopters
    # 'QUADCOPTER_DEFS':{
    #     'q1':{
    #         'position':[0,0,3],'orientation':[0,0,0],
    #         },
    #     },
    # # Controller parameters
    # 'CONTROLLER_DEFS':{
    #     'q1':{
    #         # 'type':'lqr_p2p',
    #         'type':'dd_p2p',
    #         'goals':[{'time': 0,  'position': ( 1, 0,3), 'yaw': 0},],
    #         'offset_gravity':0, # Fraction of quad weight
    #         'Q': np.diag([1,1,50, 1,1,5, 10,10,0.1, 1,1,0.1]),
    #         # 'Q': np.diag([0,0,50, 0,0,5, 10,10,0.1, 1,1,0.1]),
    #         'R': np.eye(4) * 0.1
    #         },
    #     },
    }

DEFAULT_QUAD = {
    'position':[0,0,0],'orientation':[0,0,0], # np.random.normal(0,0.3,3),
    'L': 0.3,'r': 0.1,'mass': 1.0,
    'prop_size':[10,4.5],'prop_torque_coeff': 0.0245,
    'thrust_limits':[-10,10],
    }

DEFAULT_CONTROLLERS = {
    'pid_p2p':{
        'Motor_limits':[-10,10],
        'Linear_PID':{'P':[1,1,3],'I':[0.1,0.1,0.3],'D':[1.8,1.8,1.2]},
        'Angular_PID':{'P':[15,15,8],'I':[1,1,1],'D':[2,2,15]},
        'Linear_To_Angular_Scaler':[0.1,0.1,0],
        'offset_gravity': 1,
        'Timestep': 0.05,
        },
    'lqr_p2p':{
        'Q': np.diag([1,1,50, 1,1,5, 10,10,0.1, 1,1,0.1]),
        'R': np.diag([1, 1,1,1]),
        'offset_gravity': 1,
        'Timestep': 0.05,
        },
    'dd_p2p':{
        'Q': np.diag([1,1,50, 1,1,5, 10,10,0.1, 1,1,0.1]),
        'R': np.diag([1, 1,1,1]),
        'offset_gravity': 1,
        'Timestep': 0.05,
        },
    }

defs = {
    'multi_slung_lqr_p2p': Multi_Slung_LQR_P2P,
    'multi_slung_pid_p2p': Multi_Slung_PID_P2P,
    'slung_lqr_p2p':  Slung_LQR_P2P,
    'slung_pid_p2p':  Slung_PID_P2P,
    'multi_lqr_p2p':  Multi_LQR_P2P,
    'multi_pid_p2p':  Multi_PID_P2P,
    'single_lqr_p2p': Single_LQR_P2P,
    'single_pid_p2p': Single_PID_P2P,
    'both_p2p': Both_P2P,
    'slung_both_p2p': Slung_Both_P2P,
    'test': Test,
    }

# Add default values to scene definitions for any category that exists but is missing them
for scene in defs.values():
    for quad in scene.get('QUADCOPTER_DEFS',{}).values():
        for key in DEFAULT_QUAD:
            if key not in quad:
                quad[key] = DEFAULT_QUAD[key]
    for ctrl in scene.get('CONTROLLER_DEFS',{}).values():
        DEFAULT = DEFAULT_CONTROLLERS[ctrl['type']]
        for key in DEFAULT:
            if key not in ctrl:
                ctrl[key] = DEFAULT[key]