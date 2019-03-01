# Python 2 and 3 compatibility. Use Python 3 syntax
from __future__ import absolute_import, division, print_function
try:
    input = raw_input  # Python 3 style input()
except:
    pass

from humanAgent import HumanAgent
import sys

#TODO: (1)Prevent simulator from erroring out when jobs complete (2)Fix wierd behavior from both agents (race condition maybe?)
class HumanSim():
    #bools for (1) having a PlanningAgent (2) Using graph abstraction (3) Optimal vs non-optimal, probabilistic human (4) whether or not PlanningAgent uses a probabilistic model
    def __init__(self, planner=True, optimal_human=True):
        self.agent = HumanAgent(planner=planner, optimal=optimal_human)

    def run(self):
        return self.agent.run()

if __name__ == '__main__':
    # sys.argv: 0=name, 1=planner, 2=optimal
    planner = eval(sys.argv[1])
    optimal_human = eval(sys.argv[2])
    assert isinstance(planner, bool), 'param should be a bool'
    assert isinstance(optimal_human, bool), 'param should be a bool'

    sim = HumanSim(planner=planner, optimal_human=optimal_human)
    damage = sim.run()
    print("FINAL DAMAGE: ", damage)
    sys.exit(0)