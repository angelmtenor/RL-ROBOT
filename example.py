# -*- coding: utf-8 -*-
#   +-----------------------------------------------+
#   | RL-ROBOT. Reinforcement Learning for Robotics |
#   | Angel Martinez-Tenor                          |
#   | MAPIR. University of Malaga. 2016             |
#   +-----------------------------------------------+
""" RL-ROBOT usage example """

import exp
import rlrobot

exp.ENVIRONMENT_TYPE = "MODEL"
exp.TASK_ID = "wander_1k"
exp.FILE_MODEL = exp.TASK_ID + "_model"
exp.N_EPISODES_MODEL = 1

exp.ALGORITHM = "TOSL"
exp.ACTION_STRATEGY = "QBIASR"

exp.N_REPETITIONS = 100
exp.N_EPISODES = 1
exp.N_STEPS = 1 * 10 * 60

exp.SUFFIX = ""
exp.DISPLAY_STEP = 0

rlrobot.run()

# ----------------------

# exp.ACTION_STRATEGY = "softmax"
# rlrobot.run()
