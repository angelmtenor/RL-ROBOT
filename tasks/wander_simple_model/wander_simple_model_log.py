""" EXPERIMENT LOG """

# PARAMETERS
import numpy as np

NAME = 'Wander_Maze2x2_4s4a'
DESCRIPTION = 'Wander avoiding obstacles. 2 Distance Ranges with 2 Lasers'
ALGORITHM_NAME = 'LE_TOSL'
EXPLORATION_STRATEGY = 'SOFT'
ROBOT = 'Pioneer 3dx with 8-point laser'
LEARN_FROM_MODEL = 'False'
ENVIRONMENT = 'VREP_SIM: square 2x2'
ENVIRONMENT_DETAIL = 'SpeedX3. Threaded Render. Sens:Buffer, Act:Streaming'
AGENT_ELEMENTS = ['MOBILE_BASE', 'DISTANCE_SENSOR']
ENV_ELEMENTS = []
N_STATES = 4
N_ACTIONS = 4
GAMMA = 0.9
INITIAL_ALPHA = 0.1
STEP_TIME = 0.333333333333
INITIAL_POLICY = 0

steps = 15106

# REWARDS = [-10.    -2.    -0.02  10.  ]

MOTOR_POWER = 1

# RESULTS
Average_reward = 0.00
Elapsed_time = 0.00
actual_step_time = 0.00
policy = np.array([2.00, 2.00, 1.00, 3.00])

V = np.array([54.82, 62.28, 28.97, 80.24])

Q = np.array([[-2.39, 0.55, 54.82, 4.66],
              [2.10, -3.55, 62.28, 12.71],
              [16.16, 28.97, 0.60, 25.85],
              [11.06, 8.32, 7.87, 80.24]])

Q_count = np.array([[22, 32, 300, 27],
                    [11, 13, 2850, 13],
                    [18, 34, 17, 29],
                    [10, 10, 8, 11711]])
