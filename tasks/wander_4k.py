#   +-----------------------------------------------+
#   | RL-ROBOT. Reinforcement Learning for Robotics |
#   | Angel Martinez-Tenor                          |
#   | MAPIR. University of Malaga. 2016             |
#   +-----------------------------------------------+
""" Task template v1.1 """

import numpy as np

import agent
import robot

# Task Parameters:
NAME = "Wander_4096s_25a"
DESCRIPTION = "Wander avoiding obstacles. 6 lasers. 4 ranges each. 25 actions "
ROBOT = "Pioneer 3dx with 8-point laser"
ENVIRONMENT = "VREP_SIM: square 6x6m with obstacles"
ENVIRONMENT_DETAIL = "SpeedX3. Threaded Render. Sens:Buffer, Act:Streaming"
AGENT_ELEMENTS = ["MOBILE_BASE", "DISTANCE_SENSOR"]
ENV_ELEMENTS = []
# AGENT_ELEMENTS: "MOBILE_BASE","DISTANCE_SENSOR","GOAL","ARM"
# ENV_ELEMENTS: "GOAL_OBJECT"

# Physical Parameters:
STEP_TIME = 1  # s
MOTOR_SPEED = 1  # rad/s (pioneer 3dx: 1 rad/s: ~ 0.1m/s)
RANGE_OBSTACLES = 0.80  # m
RANGE_DISPLACEMENT = 0.07  # m
RANGE_DAMAGE = 0.05  # m

# STATES. Input variables:
# Dictionary -> "variable name": (discrete intervals). Example:
# "X": (0, 5)       2 ranges: s(X)=0 if X<=5, =1 if X>5.
# "X": (0, 2, 4)    3 ranges: s(X)=0 if X<=2, =1 if X<4, =2 if X>4
# For multiple linear intervals we suggest np.linspace
INPUT_VARIABLES = {
    "laser_front": np.linspace(0, RANGE_OBSTACLES, 4),
    "laser_front_left": np.linspace(0, RANGE_OBSTACLES, 4),
    "laser_left": np.linspace(0, RANGE_OBSTACLES, 4),
    "laser_right": np.linspace(0, RANGE_OBSTACLES, 4),
    "laser_front_right": np.linspace(0, RANGE_OBSTACLES, 4),
    "laser_rear": np.linspace(0, RANGE_OBSTACLES, 4),
}
OUTPUT_VARIABLES = {
    "left_wheel": np.linspace(-MOTOR_SPEED, MOTOR_SPEED, 5),
    "right_wheel": np.linspace(-MOTOR_SPEED, MOTOR_SPEED, 5),
}
INITIAL_STATE = 0  # (usually overwritten by the fist observation)
INITIAL_POLICY = 0


def execute_action(actuator):
    """Send the commands to the actuators of the robot.
    input: vector of actuator values: e.g. [2.0,-2.0] rad/s"""
    assert len(actuator) == len(out_data), " Check output variables"
    v_left, v_right = actuator[0], actuator[1]

    # Two changes were made to improve the learning process:

    #   1- backward movement replaced by forward movement at double speed
    if v_left < 0 and v_right < 0:
        v_left = v_right = MOTOR_SPEED * 2

    #   2- One wheel stopped and the other moving backward replaced by no motion
    elif (v_left == 0 and v_right < 0) or (v_left < 0 and v_right == 0):
        v_left = v_right = 0

    robot.move_wheels(v_left, v_right)  # left wheel, right_wheel speeds (rad/s)


# TASK DEFINITION: REWARDS ----------------------------------------------------
REWARDS = np.array([-10.0, -2.0, -0.02, 10.0])


def get_reward():  # abstract s,a,sp pointless here
    """Return the reward from s,a,sp or the environment (recommended)"""
    # Sensors values already updated in robot.py when the state was observed
    distance_fl = robot.sensor["laser_front_left"]
    distance_fr = robot.sensor["laser_front_right"]
    distance_f = robot.sensor["laser_front"]
    displacement = robot.mobilebase_displacement2d

    n_collisions = (
        int(distance_fl < RANGE_DAMAGE)
        + int(distance_fr < RANGE_DAMAGE)
        + int(distance_f < RANGE_DAMAGE)
    )

    r = REWARDS[2]
    if n_collisions > 1:  # big penalty
        r = min(REWARDS)
    elif n_collisions == 1:  # small penalty
        r = REWARDS[1]
    elif displacement > RANGE_DISPLACEMENT:
        r = max(REWARDS)
    return r


# ------------------------------------------------------------------------------
def get_input_data():  # -- no modification needed --
    """Ask for sensory data to the robot and returns a vector with the values.
    Relate Input Variables with robot sensors"""
    global in_data
    for idx, item in enumerate(in_names):
        in_data[idx] = robot.sensor[item]
    return in_data


def setup():
    """task module setup is performed in agent"""
    agent.setup_task()


n_inputs = int
in_values = [None]
in_names = [None]
in_sizes = [int]
n_states = int
in_data = [None]

n_outputs = int
out_values = [None]
out_names = [None]
out_sizes = [int]
n_actions = int
out_data = [None]
