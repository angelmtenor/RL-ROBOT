""" EXPERIMENT LOG """

# PARAMETERS
NAME = "Nav_Maze4x4_obstacles_1024s9a"
DESCRIPTION = "Navigation to a Goal. 16 pos / 8 orient / 8 lasers states"
ALGORITHM_NAME = "LE_TOSL"
EXPLORATION_STRATEGY = "SOFT"
ROBOT = "Pioneer 3dx with 8-point laser"
LEARN_FROM_MODEL = "False"
ENVIRONMENT = "VREP_SIM: maze4X4_goal_with_obstacles"
ENVIRONMENT_DETAIL = "SpeedX3. Threaded Render. Sens:Buffer, Act:Streaming"
AGENT_ELEMENTS = ["MOBILE_BASE", "DISTANCE_SENSOR"]
ENVIRONMENT_ELEMENTS = ["GOAL_OBJECT"]
N_STATES = 1024
N_ACTIONS = 9
GAMMA = 0.9
INITIAL_ALPHA = 0.1
STEP_TIME = 0.333333333333
INITIAL_POLICY = 0

steps = 3600
N_REPETITIONS = 1
N_EPISODES = 50

FILE_MODEL = "Nav_Maze4x4_obstacles_1024s9a__LE_TOSL__SOFT__MODEL_T2"

s0 = 944

# REWARDS = [-10.   -2.   -1.   -0.1   1.    2.    3.    4.    5.   10. ]

MOTOR_POWER = 1

# RESULTS

Average_reward = 9.48

Elapsed_time = 69.95
actual_step_time = 0.02
