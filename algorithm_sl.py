# -*- coding: utf-8 -*-
#   +-----------------------------------------------+
#   | RL-ROBOT. Reinforcement Learning for Robotics |
#   | Angel Martinez-Tenor                          |
#   | MAPIR. University of Malaga. 2016             |
#   +-----------------------------------------------+
""" SARSA(lambda) algorithm """

import math
import time
from collections import deque

import numpy as np

import agent
import exp
import lp
import task

ID = "SL"
CAPTION = "SARSA Lambda"

eligibility = None
eli_queue = deque([])
initiated = False


def setup():
    """ Initializes global variables """
    global eligibility, eli_queue, initiated

    eligibility = np.zeros((task.n_states, task.n_actions), dtype=np.float64)

    # Reduced ET:
    # size of queue: n < log(threshold) / log(gamma*lambda)
    threshold = 0.01
    eli_size = int(math.log(threshold) / math.log(exp.GAMMA * exp.LAMBDA))
    # a FIFO queue will hold the states to update (eligibility >= 0.01)

    # uncomment for non-reduced ET  (high-computational cost without performance gain)
    # threshold = 0
    # eli_size = task.n_states

    eli_queue = deque([], eli_size)

    initiated = True


def execute():
    """ Execute the learning algorithm """

    global eligibility

    assert initiated, " SL not initiated! setup() must be previously called"

    s = lp.s
    a = lp.a
    alpha = lp.alpha
    q = lp.q
    v = lp.v
    policy = lp.policy

    # Specific Learning Algorithm
    agent.execute_action(a)
    time.sleep(lp.step_time)
    sp = agent.observe_state()
    r = agent.obtain_reward(s, a, sp)

    ap = agent.select_action(sp)  # Exploration strategy

    delta = r + exp.GAMMA * q[sp, ap] - q[s, a]  # TD error

    eligibility[s, a] = 1.0  # replace trace

    if eli_queue.count(s) > 0:
        eli_queue.remove(s)
    assert eli_queue.count(s) == 0, ("duplicated states found in ET: ", str(s))
    eli_queue.appendleft(s)

    # only the states in eli_queue are updated:
    for i in eli_queue:  # no all states updated, just those in eli_queue
        # replace eli_queue by range(task.n_states) for non-reduced ET
        for j in range(task.n_actions):
            if eligibility[i, j] > 0.01:
                q[i, j] = q[i, j] + alpha * delta * eligibility[i, j]
                eligibility[i, j] *= exp.GAMMA * exp.LAMBDA
            else:
                eligibility[i, j] = 0

        # update v and policy
        v[i] = np.max(q[i])
        policy[i] = np.argmax(q[i])

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
