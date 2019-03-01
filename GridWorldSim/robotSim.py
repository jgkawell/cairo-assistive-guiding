# Python 2 and 3 compatibility. Use Python 3 syntax
from __future__ import absolute_import, division, print_function

try:
    input = raw_input  # Python 3 style input()
except:
    pass

import sys
from planningAgent import PlanningAgent


# TODO: (1)Prevent simulator from erroring out when jobs complete (2)Fix wierd behavior from both agents (race condition maybe?)
class RobotSim():
    # bools for (1) having a PlanningAgent (2) Using graph abstraction (3) Optimal vs non-optimal, probabilistic human (4) whether or not PlanningAgent uses a probabilistic model
    def __init__(self, abstract=True, probabilistic_model=False):
        self.agent = PlanningAgent(abstract=abstract, probabilistic=probabilistic_model)

    def run(self):
        return self.agent.run()

if __name__ == '__main__':
    # sys.argv: 0=name, 1=abstract, 2=probabilistic_model
    abstract = eval(sys.argv[1])
    probabilistic_model = eval(sys.argv[2])
    assert isinstance(abstract, bool), 'param should be a bool'
    assert isinstance(probabilistic_model, bool), 'param should be a bool'

    sim = RobotSim(abstract=abstract, probabilistic_model=probabilistic_model)
    avg_time = sim.run()
    print("AVERAGE RUNTIME: ", avg_time)
    sys.exit(0)