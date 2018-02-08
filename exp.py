# -*- coding: utf-8 -*-
#   +-----------------------------------------------+
#   | RL-ROBOT. Reinforcement Learning for Robotics |
#   | Angel Martinez-Tenor                          |
#   | MAPIR. University of Malaga. 2016             |
#   +-----------------------------------------------+
""" Experiment parameters """

import time

import numpy as np

# Basic parameters -------------------------------------------------------------
TASK_ID = "wander_1k"  # task filename from tasks folder
ENVIRONMENT_TYPE = "MODEL"  # available: "MODEL", "VREP", "ROS"
SPEED_RATE = 3.0  # Recommended: REAL ROBOT: 1.0 (x1), VREP: 3.0 (x3)

N_REPETITIONS = 1  # Number of repetitions of the experiment
N_EPISODES = 1  # >1 for episodic experiments: Uses arrays from previous epi
N_STEPS = 60 * 60  # 1 step ~ 1 second (Sets LE.N_STEPS)

FILE_MODEL = TASK_ID + "_model"  # MODEL environment only
N_EPISODES_MODEL = 1  # MODEL environment only
CONTINUE_PREVIOUS_EXP = False
PREVIOUS_EXP_FILE = ""

DISPLAY_STEP = 1800  # Policy will be printed each DISPLAY_STEP

# Learning parameters ----------------------------------------------------------
ALGORITHM = "TOSL"  # "TOSL": true online SARSA lambda, "SL": SARSA lambda,
#  "S": SARSA, Q: "Q-learning
ACTION_STRATEGY = "QBIASSR"  # "QBIASSR"; Q-biased softmax regression,
# "softmax": softmax regression, "eGreedy", "random", "exploit"
ALPHA = 0.1
GAMMA = 0.9
LAMBDA = 0.9
TEMPERATURE = 1
SUFFIX = ""  # str(N_STEPS) + "steps_" + str(N_EPISODES) +"epi"

# Extra -----------------------------------------------------------------------
EXPORT_SASR_step = False  # Needed to create a Markovian model for further
#  simulation  ToDo: rlrobot.py uses large array sasr_step. To Delete

USE_T_SAS_AND_R_SAS = False  # Needed to create a Markovian model to be used
# within the learning process (Model-based RL).  Not used so far. ToDo

# Teaching
LEARN_FROM_TEACHING = False
TEACHING_STEPS = 199
TEACHING_FILE = ""
SKIP_VIEW_TEACHING = True

TEACH_THE_ROBOT = False

# --------------------- Do not modify below ------------------------------------

LEARN_FROM_MODEL = False
TEACHING_PROCESS = False
EPISODIC = False
TAUGHT_SASR = False


def check():
    """ Check experiment parameters """
    global N_STEPS, LEARN_FROM_MODEL, TEACHING_STEPS, TEACHING_PROCESS
    global EPISODIC, DISPLAY_STEP, TAUGHT_SASR

    LEARN_FROM_MODEL = False
    EPISODIC = False

    if N_EPISODES > 1:
        EPISODIC = True

    N_STEPS = int(N_STEPS)

    if ENVIRONMENT_TYPE == "MODEL":
        LEARN_FROM_MODEL = True

    assert (not (TEACH_THE_ROBOT and LEARN_FROM_TEACHING)), (
        "\n Teaching and Learning at the same time \n")

    if LEARN_FROM_TEACHING:
        TEACHING_PROCESS = True
        # NAME = NAME + "_teaching"
        TAUGHT_SASR = np.load(TEACHING_FILE)
        aux = np.size(TAUGHT_SASR[:, 0])
        if TEACHING_STEPS > aux:
            TEACHING_STEPS = aux
            print("\nWarning, teaching steps reduced to " + str(aux))

    if TEACH_THE_ROBOT:
        print("\nTEACHING IN 3 SECONDS (keys W,A,S,D) \n")
        time.sleep(3)
        print("\nGO !!! \n")

    if ENVIRONMENT_TYPE == "VREP" and SPEED_RATE == 1:
        print("\n\n\n\n WARNING: VREP WITH SPEED_RATE = 1 \n\n")
        time.sleep(10)

    if ENVIRONMENT_TYPE == "ROS" and SPEED_RATE > 1:
        print("\n\n\n\n WARNING: ROS WITH SPEED_RATE: ", SPEED_RATE, "\n\n")
        time.sleep(10)

    if not DISPLAY_STEP:
        DISPLAY_STEP = int(1e6)
