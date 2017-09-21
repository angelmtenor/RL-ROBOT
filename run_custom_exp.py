import exp
import rlrobot

exp.ALGORITHM = "TOSL"
exp.ACTION_STRATEGY = "QBIASSR"
rlrobot.run()

exp.ALGORITHM = "TOSL"
exp.ACTION_STRATEGY = "softmax"
rlrobot.run()

exp.ALGORITHM = "Q"
exp.ACTION_STRATEGY = "softmax"
rlrobot.run()
