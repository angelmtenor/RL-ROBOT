#   +-----------------------------------------------+
#   | RL-ROBOT. Reinforcement Learning for Robotics |
#   | Angel Martinez-Tenor                          |
#   | MAPIR. University of Malaga. 2016             |
#   +-----------------------------------------------+
""" Simulation from Markovian model """

import os.path
import random

import numpy as np

import task

# transition matrix of the model:
t = np.empty(0)
# Reward matrix of the model:
r = np.empty(0)
# Initial State (can be obtained directly from a 'SASR_step' datafile):
s0 = -1
freq_t = np.empty(0)
freq_r = np.empty(0)


def generate_t_and_r(datafile_model, n_episodes_model=1):
    """generate Transition and Reward functions from a 'SASR_step' datafile"""

    global t, r, s0, freq_t, freq_r

    try:
        t = np.zeros((task.n_states, task.n_actions, task.n_states), dtype=np.float16)

        r = np.zeros(
            (task.n_states, task.n_actions, task.n_states, task.REWARDS.size),
            dtype=np.float16,
        )

        freq_t = np.zeros(
            (task.n_states, task.n_actions, task.n_states), dtype=np.uint32
        )

        freq_r = np.zeros(
            (task.n_states, task.n_actions, task.n_states, task.REWARDS.size),
            dtype=np.uint32,
        )

    except MemoryError:
        # mem = (task.n_states **2) * task.n_actions * np.dtype(np.float16).itemsize / (2**20)
        print("There is Not Enough Memory to generate the Markovian model")
        print("Please, select another task or reduce the number of states.")
        exit()

    print("Generating T and R. Please wait ...")

    for epi in range(n_episodes_model):
        filename = datafile_model + "_SASR_step"
        if n_episodes_model > 1:
            filename = datafile_model + "_ep_" + str(epi) + "_SASR_step"
        try:
            data = np.load(filename)
        except OSError:
            import sys

            sys.exit("Error: " + filename + " not found")

        s0 = int(data[0, 0])
        for step in range(np.size(data, 0)):
            s = int(data[step, 0])
            a = int(data[step, 1])
            sp = int(data[step, 2])
            rew = data[step, 3]
            ty_re = np.where(task.REWARDS == rew)[0][0]
            freq_t[s, a, sp] += 1
            freq_r[s, a, sp, ty_re] += 1

    # normalize
    for s in range(task.n_states):
        if s % 100.0 == 0:
            print("state ", str(s), " of ", str(task.n_states))
        for a in range(task.n_actions):
            partial_sum_t = np.sum(freq_t[s, a, :])  # np.sum(freq_t, 2)
            for sp in range(task.n_states):
                if partial_sum_t == 0:
                    t[s, a, sp] = 1.0 / task.n_states
                else:
                    t[s, a, sp] = freq_t[s, a, sp] / partial_sum_t

                # Reward function:
                partial_sum_r = np.sum(freq_r[s, a, sp, :])  # np.sum(Freq_R,3)
                for ty_re in range(task.REWARDS.size):
                    if partial_sum_r == 0:
                        r[s, a, sp, ty_re] = 1.0 / task.REWARDS.size
                    else:
                        r[s, a, sp, ty_re] = freq_r[s, a, sp, ty_re] / partial_sum_r
    return


def get_sp(s, a):
    """return reached state from model"""
    sp = -1
    # random.seed()
    rd = random.random()
    accum = 0
    for i in range(task.n_states):
        accum = accum + t[s, a, i]
        if rd < accum:
            sp = i
            break
    if sp == -1:
        print("\n Warning: Model lacks data for T in state: %d" % s + "\n")
    return sp


def get_r(s, a, sp):
    """return obtained reward from model"""
    reward = 0
    # random.seed()
    rd = random.random()
    accum = 0
    for i in range(task.REWARDS.size):
        accum = accum + r[s, a, sp, i]
        if rd < accum:
            reward = task.REWARDS[i]
            break
    return reward


def load(filename, n_episodes_model=1):
    """Load model (T,R) from <filename>_model.npz. Update t, r, s0
    if no model is available, generate and save from SASR_step file"""
    global t, r, s0
    file_model = filename + ".npz"
    if os.path.isfile(file_model):
        print("Model file found")
        with np.load(file_model) as fm:
            t = fm["T"]
            r = fm["R"]
            s0 = fm["s0"]
    else:
        print("Model file not found")
        generate_t_and_r(filename, n_episodes_model)  # create t, r, s0
        """ Save model (T,R) to <filename>_model.npz """
        np.savez_compressed(file_model, T=t, R=r, s0=s0)
        return
