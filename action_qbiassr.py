#   +-----------------------------------------------+
#   | RL-ROBOT. Reinforcement Learning for Robotics |
#   | Angel Martinez-Tenor                          |
#   | MAPIR. University of Malaga. 2016             |
#   +-----------------------------------------------+
""" Q-Biased Softmax Regression (QBIASSR) v0.8.7 optimized """

import math
import random
from functools import reduce
from itertools import combinations

import numpy as np

import agent
import exp
import lp
import task

DEFAULT_TEMPERATURE = exp.TEMPERATURE
temperature = DEFAULT_TEMPERATURE
control_sequence = np.full(0, -1, dtype=np.int32)
rewards_sequence = np.full(0, -1, dtype=np.float32)
mix = np.full(0, -1, dtype=np.int)
comb = np.full(0, -1, dtype=np.int)
initiated = False


def setup():
    """Initializes QBIASSR"""
    global control_sequence, rewards_sequence, comb, mix, initiated
    # size_sequence = size of eli queue: n < log(threshold) / log(gamma*lambda)
    threshold = 0.01
    size_sequence = int(math.log(threshold) / math.log(exp.GAMMA * exp.LAMBDA))

    # size_sequence limits: [4, n_states/4]
    lower_limit = 4
    upper_limit = int(task.n_states / 4)
    if size_sequence > upper_limit:
        size_sequence = upper_limit
    if size_sequence < lower_limit:
        size_sequence = lower_limit
    control_sequence = np.full(size_sequence, -1, dtype=np.int32)
    rewards_sequence = np.full(size_sequence, -1, dtype=np.float32)

    # Create mix[s], index[s], subrow[s]
    n_inputs = task.n_inputs
    n_states = task.n_states
    comb = np.array(list(combinations(range(n_inputs), n_inputs - 1)), dtype=np.int16)
    mix = np.full([n_states, n_inputs, n_states], -1, dtype=np.int)
    try:
        index = np.full(([n_states, n_inputs, n_states]), -1, dtype=np.int)
    except MemoryError:
        mem = (n_states**2) * n_inputs * np.dtype(np.int).itemsize / (2**20)
        print(f"There is Not Enough Memory. Needed {mem:.1f} GB.")
        print("Please, select another task or reduce the number of states.")
        exit()

    for s in range(n_states):
        ss = agent.unwrap_state(s)
        for i in range(ss.size):
            j = ss[i]
            n = agent.cont_VAR[i, j]
            for k in range(n):
                index[s, i, k] = agent.VAR[i, j, k]
        for idx, item in enumerate(comb):
            matches = reduce(np.intersect1d, (index[s, item]))
            mix[s, idx, 0 : len(matches)] = matches
    initiated = True


def custom_softmax(input_array, temp):
    """Softmax Boltzmann action selection given a vector and temperature"""
    selected_action = -1

    # 1: Get the probabilities
    _input_array_size = len(input_array)
    _Pa = np.zeros(_input_array_size)
    for i in range(_input_array_size):
        _Pa[i] = math.exp(input_array[i] / temp)
    _Pa = np.divide(_Pa, sum(_Pa))

    # 2: Select the action
    ran = random.random()
    accum = 0
    for i in range(_input_array_size):
        accum = accum + _Pa[i]
        if ran < accum:
            selected_action = i
            break

    assert selected_action > -1
    return selected_action


def select_biased_action(s):
    """Select an action 'a' given state 's' by QBIASSR"""
    assert initiated, "QBIASSR not initiated! setup() must be called previously"

    n_actions = task.n_actions
    q = lp.q
    q_limit = lp.q_limit
    bias_s = 0
    for c in range(len(comb)):
        s_array = mix[s, c]
        s_array = s_array[s_array >= 0]
        subrow = np.zeros((len(s_array), n_actions))
        for idx, item in enumerate(s_array):
            subrow[idx] = q[item]
        aux = np.average(subrow, 0)
        bias_s += aux / len(comb)

    low_reward_loop_evasion(s)
    q_s_bias = q[s] + bias_s

    # 2016_05_26: Temporal qs_bias row is normalized for softmax regression.
    #  Standard q_limit: 100 (e.g: Rmax=10, GAMMA=0.9)
    q_s_bias *= 100 / q_limit
    selected_action = custom_softmax(tuple(q_s_bias), temperature)
    return selected_action


def low_reward_loop_evasion(s):
    """Increase the temperature if the agent is stuck in a sequence of states
    with negative average reward"""
    global temperature
    global control_sequence
    global rewards_sequence

    size_sequence = control_sequence.size

    # early steps of learning:
    if lp.step < size_sequence:
        temperature = DEFAULT_TEMPERATURE
        return

    control_sequence = lp.sasr_step[lp.step - size_sequence : lp.step, 0]
    # different state reached:
    if s not in control_sequence:
        temperature = DEFAULT_TEMPERATURE
        return

    # not enough repeated states:
    unique_sequence = np.unique(control_sequence)
    loop_rate = control_sequence.size / unique_sequence.size
    if loop_rate <= 2:
        temperature = DEFAULT_TEMPERATURE
        return

    # average reward positive:
    rewards_sequence = lp.sasr_step[lp.step - size_sequence : lp.step, 3]
    if np.average(rewards_sequence) > 0:
        temperature = DEFAULT_TEMPERATURE
        return

    # low reward loop detected. Evasion:
    temperature += 0.25 * loop_rate
    if temperature > 50:
        temperature = 50
    # print(" Local maximum detected at: ",str(s_unique))
    # print(" Temperature changed to: %0.2f" %temperature)
    return
