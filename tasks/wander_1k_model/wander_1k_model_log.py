""" EXPERIMENT LOG """

# PARAMETERS
NAME = 'Wander_Maze6x6_1024s9a'
DESCRIPTION = 'Wander avoiding obstacles. 4 Distance Ranges with 5 Lasers'
ALGORITHM_NAME = 'LE_TOSL'
EXPLORATION_STRATEGY = 'SOFT'
ROBOT = 'Pioneer 3dx with 8-point laser'
ENVIRONMENT = 'VREP_SIM: maze 6x6'
ENVIRONMENT_DETAIL = 'SpeedX3. Threaded Render. Sens:Buffer, Act:Streaming'
AGENT_ELEMENTS = ['MOBILE_BASE', 'DISTANCE_SENSOR']
ENV_ELEMENTS = []
N_STATES = 1024
N_ACTIONS = 9
GAMMA = 0.9
INITIAL_ALPHA = 0.1
STEP_TIME = 0.333333333333
INITIAL_POLICY = 0

steps = 18000
N_REPETITIONS = 1
N_EPISODES = 1

LEARN_FROM_MODEL = False
FILE_MODEL = ''

s0 = 1023

# REWARDS = [-10.    -2.    -0.02  10.  ]

MOTOR_POWER = 1

# RESULTS

Average_reward = 3.37

Elapsed_time = 6025.17
actual_step_time = 0.33
