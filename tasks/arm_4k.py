# -*- coding: utf-8 -*-
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
NAME = "Arm_cube_table_4096s27a"
DESCRIPTION = "Arm reaching a Cube on table (x,y,z) with 3 joints"
ROBOT = "Widowx arm"
ENVIRONMENT = "VREP_SIM: table with goal object"
ENVIRONMENT_DETAIL = "SpeedX3. Threaded Render. Sens:Buffer, Act:Streaming"
AGENT_ELEMENTS = ["ARM"]
ENV_ELEMENTS = ["GOAL_OBJECT"]
# AGENT_ELEMENTS: "MOBILE_BASE","DISTANCE_SENSOR", "ARM"
# ENV_ELEMENTS: "GOAL_OBJECT"

# Physical Parameters:  (tuned for Widowx arm at  V-REP scenario)
STEP_TIME = 1  # s
MOTOR_SPEED = 1
RANGE_OBSTACLES = -1  # m  (not used)
RANGE_DISPLACEMENT = 0.02  # m
RANGE_DAMAGE = -1  # m  (not used)

# STATES. Input variables:
# Dictionary -> "variable name": (discrete intervals). Example:
# "X": (0, 5)       2 ranges: s(X)=0 if X<=5, =1 if X>5.
# "X": (0, 2, 4)    3 ranges: s(X)=0 if X<=2, =1 if X<4, =2 if X>4
# For multiple linear intervals we suggest np.linspace
INPUT_VARIABLES = {
    "gripper_x": np.linspace(5.2, 5.5, 4),  # Data from V-REP scenario
    "gripper_y": np.linspace(4.8, 5.2, 8),  # (smaller partition)
    "gripper_z": np.linspace(0.2, 0.4, 4),
    "goal_x": np.linspace(5.2, 5.5, 4),
    "goal_y": np.linspace(4.8, 5.2, 8),
}
OUTPUT_VARIABLES = {
    "arm": np.linspace(-MOTOR_SPEED, MOTOR_SPEED, 3),
    "biceps": np.linspace(-MOTOR_SPEED, MOTOR_SPEED, 3),
    "forearm": np.linspace(-MOTOR_SPEED, MOTOR_SPEED, 3)
}
INITIAL_STATE = 0  # (usually overwritten by the fist observation)
INITIAL_POLICY = 0


def execute_action(actuator):
    """ Send the commands to the actuators of the robot.
    input: vector of actuator values: e.g. [2.0,-2.0] rad/s """
    assert len(actuator) == len(out_data), " Check output variables"
    v_arm, v_biceps, v_forearm = actuator
    # (modify joints if necessary)
    robot.move_full_arm(v_arm, v_biceps, v_forearm)


# TASK DEFINITION: REWARDS ----------------------------------------------------
REWARDS = np.array([-1.0, -0.02, 1.0, 2.0, 3.0, 4.0, 5.0, 10.0])
goal_reached = False


def get_reward():  # abstract s,a,sp pointless here
    """ Return the reward from s,a,sp or the environment (recommended)"""
    # Sensors values already updated in robot.py when the state was observed
    distance_gripper_goal = robot.distance_gripper_goal
    displacement_gripper_goal = robot.gripper_goal_displacement3d
    # if negative: Gripper is getting closer to the goal

    r = REWARDS[1]  # tiny step penalty

    if distance_gripper_goal < 0.05:  # robot reaches the goal
        r = max(REWARDS)
        # LE.step_time = 0 # here, if the robot reaches the static goal
        # we can speed up the rest of the experiment

    elif displacement_gripper_goal > RANGE_DISPLACEMENT:  # moving away
        r = min(REWARDS)

    elif displacement_gripper_goal < -RANGE_DISPLACEMENT:  # approaching
        r = round(REWARDS[2] + 4 / distance_gripper_goal)
        r = r if r < 5 else 5

    return r


# ------------------------------------------------------------------------------
def get_input_data():  # -- no modification needed --
    """ Ask for sensory data to the robot and returns a vector with the values.
    Relate Input Variables with robot sensors """
    global in_data
    for idx, item in enumerate(in_names):
        in_data[idx] = robot.sensor[item]
    return in_data


def setup():
    """ task module setup is performed in agent """
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
