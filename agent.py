#   +-----------------------------------------------+
#   | RL-ROBOT. Reinforcement Learning for Robotics |
#   | Angel Martinez-Tenor                          |
#   | MAPIR. University of Malaga. 2016             |
#   +-----------------------------------------------+
""" Abstract agent """

import numpy as np

import action_selection
import exp
import robot
import task

n_inputs = None
in_values = [None]
in_sizes = [None]
n_outputs = None
out_values = [None]
out_sizes = [None]

n_states = None
n_actions = None

Vs = np.empty(0)
Va = np.empty(0)
VAR = np.empty(0)
cont_VAR = np.empty(0)
initiated = False

goal_reached = False


def setup_task():
    """Task setup will be performed in the agent"""
    global n_inputs, in_values, n_outputs, out_values, Vs, Va, VAR, cont_VAR
    global in_sizes, out_sizes, n_states, n_actions, initiated

    inputvar = task.INPUT_VARIABLES
    outputvar = task.OUTPUT_VARIABLES

    n_inputs = len(inputvar)
    in_values = [None] * n_inputs
    in_names = [None] * n_inputs
    in_sizes = [int] * n_inputs
    i = 0
    for key, value in inputvar.items():
        in_names[i] = key
        in_values[i] = value
        in_sizes[i] = len(value)
        i += 1
    n_states = int(np.prod(in_sizes))
    input_data = np.zeros(n_inputs)

    n_outputs = len(outputvar)
    out_values = [None] * n_outputs
    out_names = [None] * n_outputs
    out_sizes = [int] * n_outputs
    i = 0
    for key, value in outputvar.items():
        out_names[i] = key
        out_values[i] = value
        out_sizes[i] = len(value)
        i += 1
    n_actions = int(np.prod(out_sizes))
    output_data = np.zeros(n_outputs)

    # in_values = np.array(in_values)
    # in_sizes = np.array(in_sizes)
    # input_data = np.array(input_data)
    # out_values = np.array(out_values)
    # out_sizes = np.array(out_sizes)
    # output_data = np.array(output_data)

    task.n_inputs = n_inputs
    task.in_values = in_values
    task.in_names = in_names
    task.in_sizes = in_sizes
    task.n_states = n_states
    task.in_data = input_data

    task.n_outputs = n_outputs
    task.out_values = out_values
    task.out_names = out_names
    task.out_sizes = out_sizes
    task.n_actions = n_actions
    task.out_data = output_data

    print(f"Task {task.NAME} \t {n_states} states \t {n_actions} actions")


def setup():
    """Create the variables needed for this module"""
    global Vs, Va, VAR, cont_VAR, initiated, goal_reached

    robot.setup(task.AGENT_ELEMENTS, task.ENV_ELEMENTS)

    Vs = 0
    Va = 0
    VAR = np.full((n_inputs, int(max(in_sizes)), n_states), -1, dtype=np.int)
    cont_VAR = np.full((n_inputs, int(max(in_sizes))), 0, dtype=np.int)

    generate_vs()
    generate_va()
    generate_var()

    action_selection.setup()

    goal_reached = False
    initiated = True
    print("Agent initiated")
    return


def observe_state():
    """Returns the reached state s' from robot"""
    assert initiated, "agent not initiated! setup() must be previously executed"

    unwrapped_s = np.zeros(n_inputs)

    # Special cases
    if exp.TEACHING_PROCESS:  # Observed states are already given
        from lp import step

        return exp.TAUGHT_SASR[step, 2]
    elif exp.LEARN_FROM_MODEL:
        import model
        from lp import a, s

        return model.get_sp(s, a)  # return reached state s'

    robot.update()

    input_data = task.get_input_data()

    for i in range(n_inputs):
        aux = np.digitize(input_data[i], in_values[i], right=True)
        unwrapped_s[i] = int(np.clip(aux - 1, 0, in_sizes[i] - 1))
        # print("var: "+str(i))
        # print(input_data[i])
        # print(INPUT[i])
        # print(aux)
        # print(unwrapped_s[i])

    state = wrap_state(unwrapped_s)

    assert 0 <= state < n_states, ("Wrong state: ", str(state))
    return state


def select_action(s):
    """Return action a by calling the action selection strategy"""
    a = action_selection.execute(s)
    return a


# ------------------------------------------------------------------------------
def execute_action(a):
    """Execute action in robot"""
    # Special cases
    if exp.LEARN_FROM_MODEL:
        return
    elif exp.TEACHING_PROCESS and exp.SKIP_VIEW_TEACHING:
        return
    assert 0 <= a < n_actions, ("Wrong action: ", str(a))

    unwrapped_a = unwrap_action(a)
    actuator = np.zeros(n_outputs)
    for i in range(n_outputs):
        actuator[i] = Va[i, unwrapped_a[i]]

    task.execute_action(actuator)
    return


# ------------------------------------------------------------------------------
def obtain_reward(s, a, sp):
    """Return the reward obtained"""
    # Special cases
    if exp.TEACHING_PROCESS:
        from lp import step

        if step >= exp.TEACHING_STEPS:
            exp.TEACHING_PROCESS = False  # End of teaching
        else:
            return exp.TAUGHT_SASR[step, 3]
    if exp.LEARN_FROM_MODEL:
        # from lp import s, a, sp
        import model

        return model.get_r(s, a, sp)

    r = task.get_reward()  # (s,a, sp) arguments not needed here
    return r


# ------------------------------------------------------------------------------
def wrap_state(unw_s):
    """Compose the global state from an array of substates"""
    s = unw_s[0]
    for i in range(1, n_inputs):
        pro = 1
        for j in range(0, i):
            pro *= in_sizes[j]
        s += pro * unw_s[i]
    assert 0 <= s < n_states, ("Wrong state: ", str(s))
    return int(s)


# ------------------------------------------------------------------------------
def unwrap_state(s):
    """Return the array of substates from the global state s"""
    assert 0 <= s < n_states, ("Wrong state: ", str(s))
    unwrapped_s = np.zeros(n_inputs, dtype=np.int)
    aux = s
    for i in range(n_inputs - 1):
        unwrapped_s[i] = aux % in_sizes[i]
        aux = int(aux / in_sizes[i])
    unwrapped_s[n_inputs - 1] = aux
    return unwrapped_s


# ------------------------------------------------------------------------------
def wrap_action(unw_a):
    """Compose the global action from an array of subactions"""
    a = unw_a[0]
    for i in range(1, n_outputs):
        pro = 1
        for j in range(0, i):
            pro *= out_sizes[j]
        a += pro * unw_a[i]
    assert 0 <= a < n_actions, ("Wrong action: ", str(a))
    return int(a)


# ------------------------------------------------------------------------------
def unwrap_action(a):
    """Return the array of subactions from the global action a"""
    assert 0 <= a < n_actions, ("Wrong action: ", str(a))
    unwrapped_a = np.zeros(n_outputs, dtype=np.int)
    aux = a
    for i in range(n_outputs - 1):
        unwrapped_a[i] = aux % out_sizes[i]
        aux = int(aux / out_sizes[i])
    unwrapped_a[n_outputs - 1] = aux
    return unwrapped_a


# ------------------------------------------------------------------------------
def generate_vs():
    """Generate array of substates"""
    global Vs
    Vs = np.zeros([n_inputs, int(max(in_sizes))])
    for i in range(n_inputs):
        for idx, item in enumerate(in_values[i]):
            Vs[i, idx] = item


# ------------------------------------------------------------------------------
def generate_va():
    """Generate array of subactions"""
    global Va
    Va = np.zeros([n_outputs, int(max(out_sizes))])

    for i in range(n_outputs):
        for idx, item in enumerate(out_values[i]):
            Va[i, idx] = item


# ------------------------------------------------------------------------------
def generate_var():
    """Generate Variable Matrix (input, input_value, count) -> state"""
    global VAR, cont_VAR
    VAR = np.full((n_inputs, int(max(in_sizes)), n_states), -1, dtype=np.int)
    cont_VAR = np.full((n_inputs, int(max(in_sizes))), 0, dtype=np.int)

    for s in range(n_states):
        ss = unwrap_state(s)
        for i in range(ss.size):
            # print ss
            # print ss.size
            # print i
            j = ss[i]
            k = cont_VAR[i, j]
            VAR[i, j, k] = s
            cont_VAR[i, j] += 1
    return
