# -*- coding: utf-8 -*-
#   +-----------------------------------------------+
#   | RL-ROBOT. Reinforcement Learning for Robotics |
#   | Angel Martinez-Tenor                          |
#   | MAPIR. University of Malaga. 2016             |
#   +-----------------------------------------------+
""" Learning process """

import time

import numpy as np

import agent
import exp
import show
import task

learning_module = "algorithm_" + exp.ALGORITHM.lower()
learning_algorithm = __import__(learning_module)
np.set_printoptions(precision=2)

step_time = 0
step = -1  # Current learning step
a = -1  # Current action
s = -1  # current state
sp = -1  # state reached (s')
s0 = -1  # initial state
ap = -1  # next action selected (a')
r = 0  # current reward obtained (R)

delta = 0  # Temporal Difference (TD) Error
q = None  # Q-matrix
v = None  # Value function
policy = None  # Current Policy
q_count = None
alpha = 0

elapsed_time = 0  # seconds
actual_step_time = 0  # seconds: simulation time / actual_time
final_average_reward = 0  # Resulting average R at last step

ave_v_step = None  # Average V per step (for plotting)
ave_r_step = None  # Average R obtained per step
sasr_step = None  # History of (s,a,s',R) per step

q_limit = 0

if exp.USE_T_SAS_AND_R_SAS:
    t_sas = np.zeros((task.n_states, task.n_actions, task.n_states))
    # Number of steps in which from s, executing a, resulted sp
    r_sas = np.zeros((task.n_states, task.n_actions, task.n_states))
    # Sum of Rewards for s,a,sp
initiated = False

initial_step_time = step_time  # auxiliary


def setup():
    """ Create module variables """
    global step_time, step, s, sp, a, ap, r, alpha, delta, q, v, policy, q_count
    global t_sas, r_sas, elapsed_time, actual_step_time
    global final_average_reward, ave_v_step, ave_r_step, sasr_step, q_limit, s0
    global initiated, initial_step_time

    agent.setup()

    step_time = task.STEP_TIME / exp.SPEED_RATE
    initial_step_time = step_time
    # only used in case we want to modify step_time from a task module
    # to speed up the experiment when the robot reaches the goal
    step = 0
    # Get initial state:
    if exp.LEARN_FROM_MODEL:
        import model
        s = int(model.s0)
    else:
        s = agent.observe_state()
    s0 = s

    sp = -1
    a = task.INITIAL_POLICY
    ap = -1
    r = 0
    alpha = exp.ALPHA
    delta = 0

    q = np.zeros((task.n_states, task.n_actions), dtype=np.float64)
    v = np.zeros(task.n_states, dtype=np.float64)
    policy = np.full(task.n_states, task.INITIAL_POLICY, dtype=np.uint32)
    q_count = np.zeros((task.n_states, task.n_actions), dtype=np.uint64)

    if exp.USE_T_SAS_AND_R_SAS:
        t_sas = np.zeros((task.n_states, task.n_actions, task.n_states))
        r_sas = np.zeros((task.n_states, task.n_actions, task.n_states))

    elapsed_time = 0
    actual_step_time = 0
    final_average_reward = 0

    ave_v_step = np.zeros(exp.N_STEPS)
    ave_r_step = np.zeros(exp.N_STEPS)
    sasr_step = np.zeros((exp.N_STEPS, 4))

    learning_algorithm.setup()

    q_limit = round(max(task.REWARDS) / (1 - exp.GAMMA))
    #  q_limit = max(TASK.REWARDS)/(1-EXP.GAMMA)

    if q_limit != 100:
        print("q_limit = ", str(q_limit),
              ". Softmax regression will be normalized as q_limit = 100")
        time.sleep(2)

    initiated = True
    return


def run():
    """ Execute the learning Process"""
    global step, s, sp, a, ap, r, alpha
    global q, v, policy, q_count
    global t_sas, r_sas
    global ave_r_step, sasr_step
    global elapsed_time, actual_step_time
    global final_average_reward

    assert initiated, ("learning process not initiated! setup() "
                       "must be previously called")
    mark = time.time()

    # Start learning process: -----------------------------
    for step in range(0, exp.N_STEPS):

        if step > 0 and step % exp.DISPLAY_STEP == 0:
            print("STEP: ", step)

        if agent.goal_reached:  # rest of the process filled with highest reward
            sasr_step[step, 3] = r
            ave_r_step[step] = np.average(sasr_step[0: step, 3])
            continue

        # Execute the selected learning algorithm. Also change lp variables
        learning_algorithm.execute()

        # update analysis and model arrays
        q_count[s, a] += 1
        sasr_step[step, 0] = s
        sasr_step[step, 1] = a
        sasr_step[step, 2] = sp
        sasr_step[step, 3] = r
        ave_v_step[step] = np.mean(v)
        if step == 0:
            ave_r_step[step] = sasr_step[step, 3]
        else:
            ave_r_step[step] = np.average(sasr_step[0: step, 3])

        # Display information
        if step > 0 and step % exp.DISPLAY_STEP == 0:
            print("s:", s, " a:", a, " sp:", sp, " R: %0.2f" % r)
            print("Average Reward: %0.2f" % ave_r_step[step], "\n")
            if exp.TEACHING_PROCESS:
                print("---  Teaching ---")

        if exp.USE_T_SAS_AND_R_SAS:
            t_sas[s, a, sp] += 1
            r_sas[s, a, sp] += r

        # Update state
        s = sp
        a = ap

    # End of learning process ----------------------

    final_average_reward = ave_r_step[step]
    elapsed_time = (time.time() - mark)
    actual_step_time = elapsed_time / (step + 1)

    show.process_summary()
    return
