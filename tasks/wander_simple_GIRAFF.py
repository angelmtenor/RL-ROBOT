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
NAME = "Wander_4s_4a"
DESCRIPTION = "Wander avoiding obstacles. 2 laser. 2 ranges each"
ROBOT = "Giraff"
ENVIRONMENT = "ROS"
ENVIRONMENT_DETAIL = "Trapezoid 1.5x1.0m"
AGENT_ELEMENTS = ["MOBILE_BASE", "DISTANCE_SENSOR"]
ENV_ELEMENTS = []
# AGENT_ELEMENTS: "MOBILE_BASE","DISTANCE_SENSOR","GOAL","ARM"
# ENV_ELEMENTS: "GOAL_OBJECT"

# Physical Parameters: (tuned for Giraff robot)
STEP_TIME = 1  # s
MOTOR_SPEED = 0.5  # rad/s
RANGE_OBSTACLES = 0.20  # m
RANGE_DISPLACEMENT = 0.03  # m
RANGE_DAMAGE = 0.15  # m

# STATES. Input variables:
# Dictionary -> "variable name": (discrete intervals). Example:
# "X": (0, 5)       2 ranges: s(X)=0 if X<=5, =1 if X>5.
# "X": (0, 2, 4)    3 ranges: s(X)=0 if X<=2, =1 if X<4, =2 if X>4
# For multiple linear intervals we suggest np.linspace
INPUT_VARIABLES = {
    "laser_front_left": (0, RANGE_OBSTACLES),
    "laser_front_right": (0, RANGE_OBSTACLES)
}
OUTPUT_VARIABLES = {
    "left_wheel": (-MOTOR_SPEED, MOTOR_SPEED),
    "right_wheel": (-MOTOR_SPEED, MOTOR_SPEED)
}
INITIAL_STATE = 0  # (usually overwritten by the fist observation)
INITIAL_POLICY = 0


def execute_action(actuator):  # two differential drive wheels template
    """ Send the commands to the actuators of the robot.
    input: vector of actuator values: e.g. [2.0,-2.0] rad/s """
    assert len(actuator) == len(out_data), " Check output variables"
    # backward action is replaced by no motion:
    v_left, v_right = actuator[0], actuator[1]
    if v_left < 0 and v_right < 0:
        v_left = 0
        v_right = 0
    robot.move_wheels(v_left, v_right)  # left wheel, right_wheel speeds (rad/s)


# Rewards
REWARDS = np.array([-10.0, -2.0, -0.02, 10.0])
goal_reached = False


def get_reward():  # abstract s,a,sp pointless here
    """ Return the reward from s,a,sp or the environment (recommended)"""
    # Sensors values already updated in robot.py when the state was observed
    distance_fl = robot.sensor["laser_front_left"]
    distance_fr = robot.sensor["laser_front_right"]
    displacement = robot.mobilebase_displacement2d

    r = REWARDS[2]
    if distance_fl < RANGE_DAMAGE and distance_fr < RANGE_DAMAGE:
        r = min(REWARDS)
    elif distance_fl < 0.1 or distance_fr < 0.1:
        r = REWARDS[1]
    elif displacement > RANGE_DISPLACEMENT:
        r = max(REWARDS)
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
