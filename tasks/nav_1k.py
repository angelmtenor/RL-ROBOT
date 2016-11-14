# -*- coding: utf-8 -*-
#   +-----------------------------------------------+
#   | RL-ROBOT. Reinforcement Learning for Robotics |
#   | Angel Martinez-Tenor                          |
#   | MAPIR. University of Malaga. 2016             |
#   +-----------------------------------------------+
""" Task template v1.1 """

import math

import numpy as np

import agent
import robot

# Task Parameters:
NAME = "navigation_2D_1024s_9a"
DESCRIPTION = "Navigation to a Goal. 16 pos / 8 orient / 8 lasers states"
ROBOT = "Pioneer 3dx with 8-point laser"
ENVIRONMENT = "VREP_SIM: square 4x4m with obstacles and a goal"
ENVIRONMENT_DETAIL = "SpeedX3. Threaded Render. Sens:Buffer, Act:Streaming"
AGENT_ELEMENTS = ["MOBILE_BASE", "DISTANCE_SENSOR"]
ENV_ELEMENTS = ["GOAL_OBJECT"]
# AGENT_ELEMENTS: "MOBILE_BASE","DISTANCE_SENSOR", "ARM"
# ENV_ELEMENTS: "GOAL_OBJECT"

# Physical Parameters:
STEP_TIME = 1  # s
MOTOR_SPEED = 1  # rad/s (pioneer 3dx: 1 rad/s: ~ 0.1m/s)
RANGE_OBSTACLES = 0.20  # m
RANGE_DISPLACEMENT = 0.07  # m
RANGE_DAMAGE = 0.08  # m

# STATES. Input variables:
# Dictionary -> "variable name": (discrete intervals). Example:
# "X": (0, 5)       2 ranges: s(X)=0 if X<=5, =1 if X>5.
# "X": (0, 2, 4)    3 ranges: s(X)=0 if X<=2, =1 if X<4, =2 if X>4
# For multiple linear intervals we suggest np.linspace
INPUT_VARIABLES = {
    "mobile_x": np.linspace(3, 7, 4),  # V-REP scenario: (3 to 7)m in X
    "mobile_y": np.linspace(3, 7, 4),  # V-REP scenario: (3 to 7)m in Y
    "mobile_theta": np.linspace(-math.pi, math.pi, 8),
    "laser_front": np.linspace(0, RANGE_OBSTACLES, 2),
    "laser_front_left": np.linspace(0, RANGE_OBSTACLES, 2),
    "laser_front_right": np.linspace(0, RANGE_OBSTACLES, 2)
}
OUTPUT_VARIABLES = {
    "left_wheel": np.linspace(-MOTOR_SPEED, MOTOR_SPEED, 3),
    "right_wheel": np.linspace(-MOTOR_SPEED, MOTOR_SPEED, 3)
}
INITIAL_STATE = 0  # (usually overwritten by the fist observation)
INITIAL_POLICY = 0


def execute_action(actuator):
    """ Send the commands to the actuators of the robot.
    input: vector of actuator values: e.g. [2.0,-2.0] rad/s """
    assert len(actuator) == len(out_data), " Check output variables"
    v_left, v_right = actuator[0], actuator[1]
    # backward action is replaced by double speed
    # 2 actions with only one wheel backwards changed to no motion (duplicated)
    if v_left < 0 and v_right < 0:
        v_left = v_right = MOTOR_SPEED * 2
    elif v_left * v_right < 0:
        v_left = v_right = 0
    robot.move_wheels(v_left, v_right)  # left wheel, right_wheel speeds (rad/s)


# TASK DEFINITION: REWARDS ----------------------------------------------------
REWARDS = np.array([-10.0, -2.0, -1.0, -0.1, 1.0, 2.0, 3.0, 4.0, 5.0, 10.0])
THRES_STEPS_AT_GOAL = 5  # auxiliary: if the robot remains at the goal more than
# this value, the step time will be reduced to accelerate the experiments
steps_at_goal = 0
goal_reached = False


def get_reward():  # abstract s,a,sp pointless here
    """ Return the reward from s,a,sp or the environment (recommended)"""
    # Sensors values already updated in robot.py when the state was observed
    global goal_reached
    global steps_at_goal

    distance_fl = robot.sensor["laser_front_left"]
    distance_fr = robot.sensor["laser_front_right"]
    distance_f = robot.sensor["laser_front"]
    distance_robot_goal = robot.distance_mobilebase_goal
    displacement_robot_goal = robot.mobilebase_goal_displacement2d
    # if negative: MobileBase is getting closer

    n_collisions = (int(distance_fl < RANGE_DAMAGE) +
                    int(distance_fr < RANGE_DAMAGE) +
                    int(distance_f < RANGE_DAMAGE))

    r = REWARDS[3]

    if distance_robot_goal < 0.5:  # robot reaches the goal
        r = max(REWARDS)

    elif n_collisions > 1:
        r = min(REWARDS)  # big penalty

    elif n_collisions == 1:  # small penalty
        r = REWARDS[1]

    elif displacement_robot_goal > RANGE_DISPLACEMENT:  # moving away
        r = REWARDS[2]

    elif displacement_robot_goal < -RANGE_DISPLACEMENT:  # approaching
        r = round(REWARDS[4] + 4 / distance_robot_goal)
        r = r if r < 5 else 5

    goal_reached = False
    if r >= max(REWARDS):
        steps_at_goal += 1
        if steps_at_goal > THRES_STEPS_AT_GOAL:
            # LE.step_time = LE.INITIAL_STEP_TIME/8.0
            goal_reached = True
            robot.stop_motion()
    else:
        steps_at_goal = 0

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
