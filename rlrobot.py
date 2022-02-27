#!/usr/bin/env python
#   +-----------------------------------+-----------------------------------+
#   |                                 RL-ROBOT                              |
#   |                                                                       |
#   | Copyright (c) 2016, Individual contributors, see AUTHORS file.        |
#   | Machine Perception and Intelligent Robotics (MAPIR),                  |
#   | University of Malaga. <http://mapir.isa.uma.es>                       |
#   |                                                                       |
#   | This program is free software: you can redistribute it and/or modify  |
#   | it under the terms of the GNU General Public License as published by  |
#   | the Free Software Foundation, either version 3 of the License, or     |
#   | (at your option) any later version.                                   |
#   |                                                                       |
#   | This program is distributed in the hope that it will be useful,       |
#   | but WITHOUT ANY WARRANTY; without even the implied warranty of        |
#   | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
#   | GNU General Public License for more details.                          |
#   |                                                                       |
#   | You should have received a copy of the GNU General Public License     |
#   | along with this program. If not, see <http://www.gnu.org/licenses/>.  |
#   +-----------------------------------------------------------------------+
""" RL-ROBOT main script """

import signal
import sys
import time
from shutil import copyfile

import numpy as np

import exp

tasks_path = "tasks/"
results_path = "results/"


def run():
    """Perform experiments: setups, executions, save results"""
    import sys

    if sys.version_info[0] < 3:
        sys.exit("Sorry, Python 3 required")

    exp.check()  # Check experiment parameters

    # copy the selected taskfile to speed up the execution:
    try:
        copyfile("tasks/" + exp.TASK_ID + ".py", "task.py")
    except OSError:
        sys.exit("Task " + exp.TASK_ID + " not found. Please check exp.TASK_ID")
    import lp
    import robot
    import save
    import show
    import task

    task.setup()

    caption = exp.TASK_ID + "_" + exp.ALGORITHM + "_" + exp.ACTION_STRATEGY
    if exp.SUFFIX:
        caption += "_" + exp.SUFFIX

    save.new_dir(results_path, caption)  # create result directory

    epi = 0
    # Average Reward per step (aveR):
    ave_r = np.zeros((exp.N_REPETITIONS, exp.N_STEPS))
    # Mean(aveR) of all tests per step
    mean_ave_r = np.zeros(exp.N_STEPS)
    # AveR per episode
    epi_ave_r = np.zeros([exp.N_REPETITIONS, exp.N_EPISODES])
    # actual step time per episode (for computational cost only)
    actual_step_time = np.zeros(exp.N_REPETITIONS)

    if exp.LEARN_FROM_MODEL:
        import model

        file_model = tasks_path + exp.FILE_MODEL + "/" + exp.FILE_MODEL
        model.load(file_model, exp.N_EPISODES_MODEL)
    else:
        robot.connect()  # Connect to V-REP / ROS

    if exp.CONTINUE_PREVIOUS_EXP:
        prev_exp = __import__(exp.PREVIOUS_EXP_FILE)
        print("NOTE: Continue experiments from: " + exp.PREVIOUS_EXP_FILE)
        time.sleep(3)

    # Experiment repetition loop ------------------------------------------
    for rep in range(exp.N_REPETITIONS):
        if exp.CONTINUE_PREVIOUS_EXP:
            last_q, last_v = prev_exp.q, prev_exp.v
            last_policy, last_q_count = prev_exp.policy, prev_exp.q_count
        else:
            last_q = last_v = last_policy = last_q_count = None

        # Episode loop ------------------
        for epi in range(exp.N_EPISODES):

            if exp.LEARN_FROM_MODEL:
                print("Learning from Model")
                task.STEP_TIME = 0
                lp.step_time = 0
            else:
                robot.start()

            show.process_count(caption, rep, epi, exp.EPISODIC)

            lp.setup()  # Learning process setup

            if (exp.EPISODIC and epi > 0) or exp.CONTINUE_PREVIOUS_EXP:
                lp.q, lp.v = last_q, last_v
                lp.policy, lp.count = last_policy, last_q_count

            lp.run()  # Execute the learning process

            if not exp.LEARN_FROM_MODEL:
                robot.stop()

            ave_r[rep] = lp.ave_r_step
            actual_step_time[rep] = lp.actual_step_time

            if exp.EPISODIC:
                last_q, last_v = lp.q, lp.v
                last_policy, last_q_count = lp.policy, lp.q_count

                epi_ave_r[rep, epi] = lp.ave_r_step[lp.step]

            if exp.EXPORT_SASR_step:
                save.simple(lp.sasr_step, "SASR_step")

        # end of episode

        show.process_remaining(rep, epi)

        mean_ave_r = np.mean(ave_r, axis=0)

        # End of experiment repetition loop ----------------------------

    # Mean of AveR per step (last episode)

    save.plot_mean(mean_ave_r, epi)

    save.simple(ave_r, "aveR")
    #   If EPISODIC: Save ave_r of last episode

    if exp.EPISODIC:
        # Mean of AveR reached (last step) per episode
        mean_epi_ave_r = np.mean(epi_ave_r, axis=0)
        save.plot_mean(mean_epi_ave_r, "ALL")
        save.simple(epi_ave_r, "EPI")

    final_r = mean_ave_r[lp.step]
    final_actual_step_time = np.mean(actual_step_time)

    save.log(final_r, final_actual_step_time)
    save.arrays()
    print("Mean average Reward = %0.2f" % final_r, "\n")
    print("Mean actual step time (s): %0.6f" % final_actual_step_time, "\n")

    if not exp.LEARN_FROM_MODEL:
        robot.disconnect()


# ------------------------------------------------------------------------------
def signal_handler(sig, _):
    """capture Ctrl-C event and exists"""
    sys.exit("\n Process interrupted (signal " + str(sig) + ")")


# capture SIGINT when ROS threads are present:
signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":
    if exp.ENVIRONMENT_TYPE == "ROS":
        print(" \n linking ROS ... ")
        import rl_ros

        rl_ros.setup()  # create ROS talker and listener threads
        time.sleep(0.1)
        run()
        rl_ros.finish()  # finish ROS threads
    else:
        run()
