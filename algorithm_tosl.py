# -*- coding: utf-8 -*-
#   +-----------------------------------------------+
#   | RL-ROBOT. Reinforcement Learning for Robotics |
#   | Angel Martinez-Tenor                          |
#   | MAPIR. University of Malaga. 2016             |
#   +-----------------------------------------------+
""" True Online SARSA(lambda) (TOSL) algorithm """

import math
import time
from collections import deque

import numpy as np

import agent
import exp
import lp
import task

ID = "TOSL"
CAPTION = "True Online Sarsa Lambda"

eligibility = None
q_old = 0
eli_queue = deque([])
initiated = False


def setup():
    """ Initializes global variables """
    global eligibility, q_old, eli_queue, initiated

    eligibility = np.zeros((task.n_states, task.n_actions), dtype=np.float64)
    q_old = 0

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
    """ Execute Learning Algorithm """

    global eligibility, q_old

    assert initiated, "TOSL not initiated! setup() must be previously called"

    s = lp.s
    a = lp.a
    alpha = lp.alpha
    q = lp.q
    v = lp.v
    policy = lp.policy

    # Specific Learning Algorithm
    agent.execute_action(a)
    time.sleep(lp.step_time)
    # robot.stop()
    # time.sleep(TASK.STEP_TIME/2)
    sp = agent.observe_state()
    r = agent.obtain_reward(s, a, sp)

    ap = agent.select_action(sp)  # Exploration strategy

    diff_q = q[s, a] - q_old
    q_old = q[sp, ap]

    delta = r + exp.GAMMA * q[sp, ap] - q[s, a]  # TD error

    eligibility[s, a] = (1.0 - alpha) * eligibility[s, a] + 1

    if eli_queue.count(s) > 0:
        eli_queue.remove(s)
    assert eli_queue.count(s) == 0, ("duplicated states found in ET ", str(s))
    eli_queue.appendleft(s)

    for i in eli_queue:  # no all states updated, just those in eli_queue
        # replace eli_queue by range(task.n_states) for non-reduced ET
        for j in range(task.n_actions):
            if eligibility[i, j] > 0.01:
                q[i, j] = q[i, j] + alpha * (delta + diff_q) * eligibility[i, j]
                eligibility[i, j] *= exp.GAMMA * exp.LAMBDA
            else:
                eligibility[i, j] = 0

            if i == s and j == a:
                q[i, j] = q[i, j] - alpha * diff_q

        # update v and policy
        v[i] = np.max(q[i])
        policy[i] = np.argmax(q[i])

    lp.s, lp.a = s, a
    lp.sp, lp.ap = sp, ap
    lp.r = r
    lp.alpha = alpha
    lp.q = q
    lp.v = v
    lp.policy = policy
    lp.delta = delta
