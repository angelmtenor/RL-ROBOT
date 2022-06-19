import exp
import rlrobot

exp.ENVIRONMENT_TYPE = "MODEL"   # "VREP" for V-REP simulation
exp.TASK_ID = "wander_1k"
exp.FILE_MODEL = exp.TASK_ID + "_model"

exp.ALGORITHM = "TOSL"
exp.ACTION_STRATEGY = "QBIASSR"
 
exp.N_REPETITIONS = 1
exp.N_EPISODES = 1
exp.N_STEPS = 60 * 60

exp.DISPLAY_STEP = 500

rlrobot.run()