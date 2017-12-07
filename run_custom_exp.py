# import sys
# sys.modules[__name__].__dict__.clear()
# import exp, rlrobot

# exp.ALGORITHM = "TOSL"
# exp.ACTION_STRATEGY = "QBIASSR"
# rlrobot.run()

# import sys
# sys.modules[__name__].__dict__.clear()
# import exp, rlrobot

# exp.ALGORITHM = "TOSL"
# exp.ACTION_STRATEGY = "softmax"
# rlrobot.run()

import sys
sys.modules[__name__].__dict__.clear()
import exp, rlrobot

exp.ALGORITHM = "Q"
exp.ACTION_STRATEGY = "softmax"
rlrobot.run()