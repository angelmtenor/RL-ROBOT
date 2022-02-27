#   +-----------------------------------------------+
#   | RL-ROBOT. Reinforcement Learning for Robotics |
#   | Angel Martinez-Tenor                          |
#   | MAPIR. University of Malaga. 2016             |
#   +-----------------------------------------------+
""" SARSA algorithm """

import time

import numpy as np

import agent
import exp
import lp

ID = "S"
CAPTION = "SARSA"


def setup():
    """Setup algorithm"""
    pass  # no needed here


def execute():
    """Execute the learning algorithm"""
    s = lp.s
    a = lp.a
    alpha = lp.alpha
    q = lp.q
    v = lp.v
    policy = lp.policy

    agent.execute_action(a)
    time.sleep(lp.step_time)

    sp = agent.observe_state()
    r = agent.obtain_reward(s, a, sp)

    ap = agent.select_action(sp)

    # update Q
    delta = r + exp.GAMMA * q[sp, ap] - q[s, a]  # TD error (SARSA)
    q[s, a] = q[s, a] + alpha * delta  # update rule

    # Update V and Policy
    v[s] = np.max(q[s])
    policy[s] = np.argmax(q[s])

    lp.s = s
    lp.a = a
    lp.sp = sp
    lp.ap = ap
    lp.r = r
    lp.alpha = alpha
    lp.q = q
    lp.v = v
    lp.policy = policy
    lp.delta = delta
