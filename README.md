# RL-ROBOT
This repository provides a Reinforcement Learning framework in Python from the Machine Perception and Intelligent Robotics research group [(MAPIR)](http://mapir.isa.uma.es).

### Requirements
* Python 3
* numpy
* matplotlib
* tkinter   `sudo apt-get install python-tk`


### V-REP settings: 
(Tested on V-REP_PRO_EDU_V3_3_2 64_bits Linux)

1. Use default values of `remoteApiConnections.txt`
    ~~~
    portIndex1_port 		= 19997
    portIndex1_debug 		= false
    portIndex1_syncSimTrigger 	= true
    ~~~

2. Activate threaded rendering (recommended):
    `system/usrset.txt -> threadedRenderingDuringSimulation = 1` 

Recommended simulation settings for scenes in RL-ROS (already set in the provide ones):

* Simulation step time: 50 ms  (default) 
* Real-Time Simulation: Enabled
* Multiplication factor: 3.00 (required CPU >= i3 3110m)

 **Execute V-REP** 
 (`./vrep.sh on linux`). `File -> Open Scene -> (open any scene for RL-ROS)` 

 
 