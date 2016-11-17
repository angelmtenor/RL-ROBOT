# -*- coding: utf-8 -*-
#   +-----------------------------------------------+
#   | RL-ROBOT. Reinforcement Learning for Robotics |
#   | Angel Martinez-Tenor                          |
#   | MAPIR. University of Malaga. 2016             |
#   +-----------------------------------------------+
""" RL-ROBOT usage example """

import exp
import rlrobot

exp.ENVIRONMENT_TYPE = "VREP"
exp.TASK_ID = "nav_1k"
# exp.FILE_MODEL = exp.TASK_ID + "_model"
# exp.N_EPISODES_MODEL = 1

exp.ALGORITHM = "TOSL"
exp.ACTION_STRATEGY = "QBIASR"

exp.N_REPETITIONS = 1
exp.N_EPISODES = 10
exp.N_STEPS = 1 * 60 * 60

exp.SUFFIX = "Video"
exp.DISPLAY_STEP = 100

rlrobot.run()

# ----------------------

# exp.ACTION_STRATEGY = "softmax"
# rlrobot.run()
