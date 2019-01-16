import numpy as np

E = EMPTY = 0
B = BLOCKED = 1
G = GOAL = 2

# The maze from the assignment
MAZE = np.array(
    [[0, 0, 0, 0, 0, 0, 0],
     [0, 0, B, 0, 0, 0, 0],
     [0, B, B, 0, 0, 0, 0],
     [0, 0, B, 0, 0, B, 0],
     [0, 0, 0, 0, 0, B, G]])

# The possible actions
ACTIONS = np.array(['N', 'E', 'S', 'W'])
ACTION_IDX = {a: idx for idx, a in enumerate(ACTIONS)}

# Transition probabilities given a specific action to one of the 5 outcomes.
# probabilities for [staying, N, E, S, W]
ACTION_PROBS = {
    ACTIONS[0]: [0.0, 0.7, 0.1, 0.1, 0.1],
    ACTIONS[1]: [0.0, 0.1, 0.7, 0.1, 0.1],
    ACTIONS[2]: [0.0, 0.1, 0.1, 0.7, 0.1],
    ACTIONS[3]: [0.0, 0.1, 0.1, 0.1, 0.7]
}


def get_neighbours(state):
    """Return a list of all four neighbour states and the current position."""
    row, col = state
    return [
        (row, col),  # -
        (row - 1, col),  # N
        (row, col + 1),  # E
        (row + 1, col),  # S
        (row, col - 1),  # W
    ]


def is_in_maze(state):
    """Return True if the position, represented as state, is inside the maze and
    not blocked."""
    R, C = MAZE.shape
    row, col = state
    return (0 <= row < R) and (0 <= col < C) and MAZE[state] != BLOCKED


def state_prob(s, s2, a):
    """Return the probability of transitioning from s to s2 by action a."""
    assert is_in_maze(s)
    assert a in ACTIONS
    neighbours = get_neighbours(s)
    assert s2 in neighbours

    if MAZE[s] == GOAL:
        return 1.0 if s == s2 else 0.0

    # Copy transition probabilities of action a.
    pr = list(ACTION_PROBS[a])

    # Fall back to s if you can't go to a specific neighbour.
    for i, n in enumerate(neighbours):

        if not is_in_maze(n):
            # Illegal neighbour => move probabilities to staying.
            # pr[0] is the probability of staying in the current position.
            pr[0] += pr[i]
            # Reset the probability of the illegal move to staying (sum of all 5
            # paths equals 1 again)
            pr[i] = 0

    # Return the probability for the target state.
    return pr[neighbours.index(s2)]


def get_all_states():
    """Return a generator to iterate over all possible states.

    Can be used like this:
    for s in get_all_states():
        (do something with s)"""
    for x in range(MAZE.shape[0]):
        for y in range(MAZE.shape[1]):
            if is_in_maze((x, y)):
                yield (x, y)


# Implement the following functions regarding part 1 of the assignment.

def reward(s, s2, a):
    """Return the reward for taking action a in state s and ending up in s2.

    The reward is 0 for all state transitions, except when when entering the
    goal state, G, the reward is 10.0"""
    neighbours = get_neighbours(s)
    assert s2 in neighbours
    assert a in ACTIONS

    # Next state if we take action a from s.
    # n = neighbours[ACTION_IDX[a] + 1]

    if MAZE[s] != GOAL and MAZE[s2] == GOAL:
        return 10.0
    else:
        return 0


def value_backup(policy, s, V, discount=0.9):
    """Compute and return the new value only for state s using the current
    values V and the current policy.

    The value backup is related to policy evaluation (see slides 37)."""
    v = 0

    # Outer summation in the Bellman equation.
    for a in ACTIONS:
        # pi is the probability of taking action a given that we are in state s.
        pi = policy[s][ACTION_IDX[a]]

        # Inner summation in the Bellman equation.
        # Iterate through the possible next states of s, that is n (s').
        for i, n in enumerate(get_neighbours(s)):

            # Probability p is zero if is_in_maze(n) is false. It follows that
            # pi * p * (r + discount * V[n]) is zero in that case. So, there's
            # no need to execute the block under this if statement.
            if is_in_maze(n):
                p = state_prob(s, n, a)

                if p != 0:
                    r = reward(s, n, a)
                    # We can multiply pi here because we are simply distributing
                    # it over the inner summation.
                    v += pi * p * (r + discount * V[n])

    return v


def policy_evaluation(policy, discount=0.9, epsilon=1e-9):
    """Iteratively computes the values of each state following a specific
    policy.

    The starting values for each state should be 0 when evaluating a new policy!

    Stop the iterative procedure if no state has a bigger change than epsilon.

    Returns the final values for each state."""
    V = np.zeros_like(MAZE, dtype=np.float64)

    while True:
        delta = 0

        for s in get_all_states():
            v = value_backup(policy, s, V, discount)

            delta = max(delta, np.abs(v - V[s]))

            V[s] = v

        if delta < epsilon:
            break

    return V


def policy_improvement(policy, V, discount=0.9):
    """Update the policy by greedily choosing actions based on the current
    values of states.

    Returns the new policy."""

    def softmax(x):
        e_x = np.exp(x - np.max(x))
        return e_x / e_x.sum()

    def one_hot(p, size=len(ACTIONS)):
        return np.eye(size)[p]

    for s in get_all_states():

        action_values = np.zeros(len(ACTIONS))

        # Arg-max w.r.t to action, that is, we find the action which maximizes
        # the summation.
        for a in ACTIONS:

            # Summation over next states. This is basically the computation
            # of the Q given policy.
            for i, s_p in enumerate(get_neighbours(s)):
                if is_in_maze(s_p):
                    p = state_prob(s, s_p, a)
                    if p != 0:
                        r = reward(s, s_p, a)
                        action_values[ACTION_IDX[a]] += p * (
                            r + discount * V[s_p])

        # It seems it doesn't want to terminate soon using softmax.
        # policy[s] = softmax(action_values)

        best_action = np.argmax(action_values)
        policy[s] = one_hot(best_action)

    return policy


def policy_iteration(discount=0.9):
    """Use a random starting policy and iteratively improve it until it is
    converged.

    Use the functions policy_evaluation and policy_iteration when implementing
    this function.

    If you copy numpy array make sure to copy by value using np.copy().

    Returns the converged policy and the final values of each state."""

    # A policy is indexed by a state and action and returns a probability of
    # executing that action in that state.
    policy = 1 / len(ACTIONS) * np.ones((MAZE.shape[0], MAZE.shape[1],
                                         len(ACTIONS)))


    policy_stable = False
    # stats = {"iterations": 0, "time elapsed (sec)": 0}

    while not policy_stable:
        # Generate a value function from the policy.
        V = policy_evaluation(policy, discount)

        old_policy = np.copy(policy)

        policy = policy_improvement(policy, V, discount)

        policy_stable = np.array_equal(old_policy, policy)

    # print(V)

    return policy, V


if __name__ == "__main__":
    policy, V = policy_iteration()
    
    np.set_printoptions(precision=3, suppress=True)
    print(V)