#   +-----------------------------------------------+
#   | RL-ROBOT. Reinforcement Learning for Robotics |
#   | Angel Martinez-Tenor                          |
#   | MAPIR. University of Malaga. 2016             |
#   +-----------------------------------------------+
""" Print custom messages """

import exp
import lp


def process_count(caption, rep, epi, episodic):
    """print current repetition and episode"""
    print("-" * 50)
    print(caption)
    if exp.N_REPETITIONS > 1:
        print(f"repetition \t{rep + 1} of {exp.N_REPETITIONS}")
    if episodic:
        print(f"episode \t{epi + 1} of {exp.N_EPISODES}")
    print("-" * 50)


def process_remaining(rep, epi):
    """print remaining processes"""
    if rep < exp.N_REPETITIONS - 1:
        remaining_processes = (
            exp.N_REPETITIONS * exp.N_EPISODES - rep * exp.N_EPISODES + epi + 1
        )
        remaining_time = remaining_processes * lp.elapsed_time
        print(
            "Remaining time: \t {:.2f}m ({:.2f}h)".format(
                remaining_time / 60.0, remaining_time / 3600.0
            ),
            "\n",
        )


def process_summary():
    """print process summary"""
    print("-" * 22, "END", "-" * 23)
    print("Number of steps: \t", str(lp.step + 1))
    print("Actual Step Time: \t %.6f" % lp.actual_step_time + "s")
    print(
        "Elapsed Time:\t\t %.2f" % lp.elapsed_time,
        "s  ( %.2f" % (lp.elapsed_time / 60),
        "m %.2f" % (lp.elapsed_time / 60 / 60),
        "h )",
    )
    print("Average Reward: \t %.2f" % lp.final_average_reward)
    print("-" * 50, "\n")
