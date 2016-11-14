""" EXPERIMENT LOG """

# PARAMETERS
NAME = 'Arm_cube_table_1024s9a'
DESCRIPTION = 'Arm reaching a Cube on table grid 4x8 '
ALGORITHM_NAME = 'LE_TOSL'
EXPLORATION_STRATEGY = 'SOFT'
ROBOT = 'Widowx Arm'
LEARN_FROM_MODEL = 'False'
ENVIRONMENT = 'VREP_SIM: Table-Cube'
ENVIRONMENT_DETAIL = 'SpeedX3. Threaded Render. Sens:Buffer, Act:Streaming'
AGENT_ELEMENTS = ['ARM']
ENVIRONMENT_ELEMENTS = ['GOAL_OBJECT']
N_STATES = 1024
N_ACTIONS = 9
GAMMA = 0.9
INITIAL_ALPHA = 0.1
STEP_TIME = 0.333333333333
INITIAL_POLICY = 0

steps = 120
N_REPETITIONS = 1
N_EPISODES = 50

FILE_MODEL = 'Arm_cube_table_1024s9a__LE_TOSL__SOFT__T3'

s0 = 190

# REWARDS = [-10.    -2.    -1.    -0.02   1.     2.     3.     4.     5.    10.  ]

MOTOR_POWER = 1

# RESULTS

Mean_Average_reward = 9.87

Median_Average_reward = 9.87

Elapsed_time = 40.14
actual_step_time = 0.33
