# Quadcopter and slung load simulator
An environment to simulate quadcopter teams carrying slung payloads using various types of controllers. Eventually to simulate disturbance decoupling. The simulator supports time scaling (including real-time simulations) and headless mode (the simulator runs in background without a GUI update).

This simulator environment is being developed by Avi Mittal, based on one created by Abhijit Majumdar (<https://github.com/abhijitmajumdar/Quadcopter_simulator>).

![Single Quadcopter Simulation](/quad_sim.gif?raw=true "quad_sim")

Single Quadcopter Simulation

![Multi Quadcopter Simulation](/multiquad_sim.gif?raw=true "multiquad_sim")

Multi Quadcopter Simulation

## Dependencies
- Simulation of dynamics:
    - Numpy
    - SciPy (integrate, linalg)
- Controllers
	- Cvxpy (not yet used)
- GUI:
    - Matplotlib (pyplot, animation)
    - Matplotlib Mapping Toolkits
- Threading:
    - Datetime
    - Time
    - Threading
- Input/Output:
	- Argparse
	- Signal
	- Sys

## How to run
1. Clone the repository
2. Move into the directory
3. Run the code
```sh
$ git clone https://github.com/aviator2160/disturbance-decoupling.git
$ cd disturbance_decoupling
$ python simulate.py --sim single_pid_p2p
```

#### List of example simulations:
- Single quad with point-to-point proportional-integral-derivative (PID) control:
```sh
$ python simulate.py --sim single_pid_p2p
```
- Single quad with point-to-point continuous-time linear-quadratic regulator (LQR) control:
```sh
$ python simulate.py --sim single_lqr_p2p
```
- One quad with point-to-point PID control and a second quad with point-to-point LQR control, on parallel trajectories:
```sh
$ python simulate.py --sim both_p2p
```
- Multiple (two) quads with point to point PID control:
```sh
$ python simulate.py --sim multi_pid_p2p
```
- Multiple (two) quads with point to point LQR control:
```sh
$ python simulate.py --sim multi_lqr_p2p
```
- Single quad carrying a slung load with point-to-point PID control:
```sh
$ python simulate.py --sim slung_pid_p2p
```
- Single quad carrying a slung load with point-to-point LQR control:
```sh
$ python simulate.py --sim slung_lqr_p2p
```
- Team of quads (four) carrying a slung load with with point-to-point PID control:
```sh
$ python simulate.py --sim multi_slung_pid_p2p
```
- Team of quads (four) carrying a slung load with with point-to-point LQR control:
```sh
$ python simulate.py --sim multi_slung_lqr_p2p
```

#### Other parameter examples
- Speeding up time scaling (default 1.0) to double-speed (0.5)
```sh
$ python simulate.py --sim single_pid_p2p --time_scale 0.5
```
- Refining physics timestep (default 0.01 s) to 0.005s
```sh
$ python simulate.py --sim single_pid_p2p --phys_timestep 0.005
```
- Enabling headless mode (turns off GUI)
```sh
$ python simulate.py --sim single_pid_p2p --headless
```
These parameters can also be changed by editing the default parameters in the script `simulate.py`. These parameters will still be overridden by command line arguments, if provided.

#### Adding simulations
All simulations are defined in `sim_defs.py`. To add a new simulation, adding a new dictionary containing its definitions to this file. One can define the number of and parameters for quadcopters, controllers, payloads, and cables.

## Implementation (everything past this point is OUTDATED)
The main classes which define the simulator are Quadcopter and GUI. There is also a sample Controller classes, which implements a controller for the quadcopter. The objective was to make a quadcopter dynamic simulator, which allowed us to control each motor individually. The other requirement was the ability to run the simulations in the background, hence possibly expediting the computations, commonly referred to as the headless mode. Once the simulator thread is started, the GUI may or may not be updated at the developers will. There is also a time scaling factor, which can allow the simulation to run as fast as the processor supports, and as slow as one can fall asleep doing so.

##### Propeller class
This class defines the thrust generated by a propeller of a specified size at a given speed of rotation. This was based on the equation provided on http://www.electricrcaircraftguy.com/2013/09/propeller-static-dynamic-thrust-equation.html. This was made into a separate class to enable the implementation of other multi-rotors as well.

##### Quadcopter class
This class performs the simulations of the dynamics based on the state space solution of a quadcopter. It uses 4 objects of the Propeller class to implement the quad configuration of a quadcopter. The state space representation of a quadcopter model have been adapted from Quadcopter Dynamics, Simulation, and Control by Andrew Gibiansky and Quadrotor Dynamics and Control by Randal Beard. The class is initialized using the quadcopter parameters like length of an arm, the weight of the quadcopter, radius of a sphere representing the center blob of the quadcopter, etc. It is defined in a dictionary which can be modified.

The state space is defined as: *X = [x,y,z,x_dot,y_dot,z_dot,theta,phi,gamma,theta_dot,phi_dot, gamma_dot]*. The update to the state is performed by an ODE solver from the current state to a new state over a period of *dt* time(defined by user). It uses the *vode* ODE solver available from the *SciPy* library. It has an update method to update the state, which is run on a thread at intervals defined by the time scaling factor. The thread can be started by the *start_thread* method.

It has methods to set_motor_speeds(), get_orientation(), get_position(), get_angular_rate(), get_linear_rate(), set_position() and set_orientation(), which can be used by the controller.

##### Graphical User Interface (GUI) class
The GUI initialization takes in the same quadcopter parameter directory. It is optional to pass in the get methods for position and orientation of the quadcopter to the GUI initialization. It uses a *Matplotlib function animation* to plot a representation of the quadcopter, which updates only the parts of the plot that change. These updates occur at a fixed rate which is significantly slower than the quadcopter class update. It can also be updated using the method update(), with the position and orientation of the quadcopter if the get methods are not defined while initializing the object.

##### Controller class
A demo to implement a controller class. It is initialized using the quadcopter object and a controller parameter dictionary. The quadcopter object is used to update the global time as well as the quadcopter state and also to set the motor speeds on the quadcopter. An example  parameter dictionary is provided which defines the different constants used by the controller. The update() method updates the motor speeds based on the control algorithm. The start_thread() method initializes the thread to keep updating the controller every *update_rate*(specified by the user).

Two example implementation of controller class are provided. One is to implement a point to point controller which controls to move the quadcopter to a desired (x,y,z) location. The other is a velocity controller, which controls to set the (x,y) velocity of the quadopter as desired, while using the z to set the quadcopter altitude. The velocity controller class is inherited from the point-to-point class, since the only change is in the update method, and can be used as an example to implement other type of controllers.

## Parameters
- TIME_SCALING: Used to define how fast the simulator runs. Value of 1.0 corresponds to real-time simulations. Any smaller number makes the simulation faster and vice versa. A value of 0 runs the simulation run as fast as possible on the current hardware.
- QUAD_DYNAMICS_UPDATE: The delta time over which the dynamics of the quadcopter are updated
- CONTROLLER_DYNAMICS_UPDATE: The delta time over which the controller updates the motors (Note: Changing this value would also cause the default controller parameters to behave differently)
- QUADCOPTER(S): The parameters which define the quadcopter: initial position and orientation,length of arm, center radius, propeller size and weight.
- CONTROLLER(N)PARAMETERS: The parameters which define the controller behavior: Motor limits, Tilt limits, Yaw_Control_Limits, Throttle offset, Linear PID, Linear to Angular Scaler, Yaw_Rate_Scaler and Angular PID
- GOAL(S): The goals to loop over

## Changes
- Added sim showcasing LQR control with different sizes of drones
- Added sims showcasing PID and LQR control for teams of quads carrying slung loads

## To-do
- Add disturbance decoupling LQR controller
- Tune PIDs more finely?
