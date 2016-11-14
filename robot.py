# -*- coding: utf-8 -*-
#   +-----------------------------------------------+
#   | RL-ROBOT. Reinforcement Learning for Robotics |
#   | Angel Martinez-Tenor                          |
#   | MAPIR. University of Malaga. 2016             |
#   +-----------------------------------------------+
""" Generic Robot module """

import math
import sys

import numpy as np

import exp

link_dict = {'ROS': 'rl_ros', 'VREP': 'rl_vrep', 'MODEL': 'rl_vrep'}
try:
    link_module = link_dict[exp.ENVIRONMENT_TYPE]
except KeyError:
    sys.exit("ENVIRONMENT_TYPE " + exp.ENVIRONMENT_TYPE + " undefined \n")
link = __import__(link_module)

# 2D navigation

mobilebase_pose2d = np.full(3, -1, dtype=np.float64)  # x(m),y(m),theta(rad)
last_mobilebase_pose2d = np.full(3, -1, dtype=np.float64)
mobilebase_displacement2d = 0  # between two consecutive steps (m)

dist_obstacle = np.full(link.N_LASERS, -1, dtype=np.float64)

last_distance_mobilebase_goal = -1
distance_mobilebase_goal = -1
mobilebase_goal_displacement2d = -1

# 3D motion
gripper_pose3d = np.full(3, -1, dtype=np.float64)
last_gripper_pose3d = np.full(3, -1, dtype=np.float64)
gripper_displacement3d = -1

goal_pose3d = np.full(3, -1, dtype=np.float64)
last_goal_pose3d = np.full(3, -1, dtype=np.float64)
goal_displacement3d = -1  # Just In case we want to move an object

last_distance_gripper_goal = -1
distance_gripper_goal = -1
gripper_goal_displacement3d = -1

AGENT_ELEMENTS = []
ENV_ELEMENTS = []

initiated = False

sensor = {}


def setup(agent_elem, environment_elem):
    """ initialize robot """
    global AGENT_ELEMENTS, ENV_ELEMENTS
    global mobilebase_pose2d, last_mobilebase_pose2d, mobilebase_displacement2d
    global dist_obstacle
    global gripper_pose3d, last_gripper_pose3d, gripper_displacement3d
    global goal_pose3d, last_goal_pose3d, goal_displacement3d
    global last_distance_mobilebase_goal, distance_mobilebase_goal
    global mobilebase_goal_displacement2d
    global last_distance_gripper_goal, distance_gripper_goal
    global gripper_goal_displacement3d
    global initiated

    global sensor

    AGENT_ELEMENTS = agent_elem
    ENV_ELEMENTS = environment_elem

    mobilebase_pose2d = np.full(3, -1, dtype=np.float64)
    last_mobilebase_pose2d = np.full(3, -1, dtype=np.float64)

    mobilebase_displacement2d = 0

    dist_obstacle = np.full(link.N_LASERS, -1, dtype=np.float64)

    gripper_pose3d = np.full(3, -1, dtype=np.float64)
    last_gripper_pose3d = np.full(3, -1, dtype=np.float64)
    gripper_displacement3d = -1

    goal_pose3d = np.full(3, -1, dtype=np.float64)
    last_goal_pose3d = np.full(3, -1, dtype=np.float64)
    goal_displacement3d = -1

    last_distance_mobilebase_goal = -1
    distance_mobilebase_goal = -1
    mobilebase_goal_displacement2d = -1

    last_distance_gripper_goal = -1
    distance_gripper_goal = -1
    gripper_goal_displacement3d = -1

    update()
    update()  # To obtain the first relative displacement

    return


def update():
    """ update robot & environment state (sensors, locations...) """
    global mobilebase_pose2d, last_mobilebase_pose2d, mobilebase_displacement2d
    global dist_obstacle
    global gripper_pose3d, last_gripper_pose3d, gripper_displacement3d
    global goal_pose3d, last_goal_pose3d, goal_displacement3d
    global last_distance_mobilebase_goal, distance_mobilebase_goal
    global mobilebase_goal_displacement2d
    global last_distance_gripper_goal, distance_gripper_goal
    global gripper_goal_displacement3d

    global sensor

    if "DISTANCE_SENSOR" in AGENT_ELEMENTS:
        dist_obstacle = get_distance_obstacle()

    if "MOBILE_BASE" in AGENT_ELEMENTS:
        last_mobilebase_pose2d = mobilebase_pose2d
        mobilebase_pose2d = get_mobilebase_pose2d()
        mobilebase_displacement2d = distance2d(mobilebase_pose2d,
                                               last_mobilebase_pose2d)

    if "ARM" in AGENT_ELEMENTS:
        last_gripper_pose3d = gripper_pose3d
        gripper_pose3d = link.get_gripper_pose3d()
        gripper_displacement3d = distance3d(gripper_pose3d, last_gripper_pose3d)

    if "GOAL_OBJECT" in ENV_ELEMENTS:
        last_goal_pose3d = link.get_goal_pose_3d()
        goal_pose3d = link.get_goal_pose_3d()
        goal_displacement3d = distance3d(goal_pose3d, last_goal_pose3d)

        if "MOBILE_BASE" in AGENT_ELEMENTS:
            last_distance_mobilebase_goal = distance_mobilebase_goal
            distance_mobilebase_goal = distance2d(goal_pose3d,
                                                  mobilebase_pose2d)
            mobilebase_goal_displacement2d = (distance_mobilebase_goal -
                                              last_distance_mobilebase_goal)
            # Negative: the mobilebase is getting closer

        if "ARM" in AGENT_ELEMENTS:
            last_distance_gripper_goal = distance_gripper_goal
            distance_gripper_goal = distance3d(goal_pose3d, gripper_pose3d)
            gripper_goal_displacement3d = (distance_gripper_goal -
                                           last_distance_gripper_goal)
            # Negative: the arm is getting closer

    sensor["mobile_x"] = mobilebase_pose2d[0]
    sensor["mobile_y"] = mobilebase_pose2d[1]
    sensor["mobile_theta"] = mobilebase_pose2d[2]

    sensor["gripper_x"] = gripper_pose3d[0]
    sensor["gripper_y"] = gripper_pose3d[1]
    sensor["gripper_z"] = gripper_pose3d[2]

    sensor["goal_x"] = goal_pose3d[0]
    sensor["goal_y"] = goal_pose3d[0]

    sensor["laser_front"] = dist_obstacle[0]
    sensor["laser_front_left"] = dist_obstacle[1]
    sensor["laser_left"] = dist_obstacle[2]
    sensor["laser_rear_left"] = dist_obstacle[3]
    sensor["laser_rear"] = dist_obstacle[4]
    sensor["laser_rear_right"] = dist_obstacle[5]
    sensor["laser_right"] = dist_obstacle[6]
    sensor["laser_front_right"] = dist_obstacle[7]


def move_wheels(left_wheel, right_wheel):
    """ move base wheels (inputs: rad/s) """
    link.move_wheels(left_wheel, right_wheel)
    return


def stop_motion():
    """ stop the mobilebase """
    link.stop_motion()
    return


def move_full_arm(arm, biceps, forearm):
    """ move robotic arm """
    link.move_arm(arm)
    link.move_biceps(biceps)
    link.move_forearm(forearm)
    return


def get_distance_obstacle():
    """ return distances to objects measured by laser """
    di = link.get_distance_obstacle()
    return di


def get_mobilebase_pose2d():
    """ return 2d pose of the mobilebase (x,y,theta) """
    po = link.get_mobilebase_pose2d()
    return po


def get_gripper_pose3d():
    """ return the position of the gripper:  [ x(m), y(m), z(m) ] """
    po = link.get_gripper_pose3d()
    return po


def get_goal_pose3d():
    """ return the position of the goal object:  [ x(m), y(m), z(m) ] """
    po = link.get_goal_pose_3d()
    return po


def distance2d(pose_a, pose_b):
    """ get distance 2d (x,y axis) between 2 poses """
    delta_pose2d = abs(pose_a - pose_b)
    displacement2d = math.sqrt(delta_pose2d[0] ** 2 + delta_pose2d[1] ** 2)
    return displacement2d


def distance3d(pose_a, pose_b):
    """ get distance 3d between 2 poses """

    delta_pose3d = abs(pose_a - pose_b)
    displacement3d = math.sqrt(delta_pose3d[0] ** 2 + delta_pose3d[1] ** 2 +
                               delta_pose3d[2] ** 2)
    return displacement3d


def start():
    """ Start robot """
    link.start()
    return


def stop():
    """ stop robot """
    link.stop()
    return


def setup_devices():
    """ setup robot's devices """
    link.setup_devices()
    return


def connect():
    """ connect to the robot """
    link.connect()
    return


def disconnect():
    """ disconnect from the robot """
    link.disconnect()
    return
