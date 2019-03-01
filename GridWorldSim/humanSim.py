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
        self.planner = planner
        self.optimal_human = optimal_human
        self.agent = HumanAgent(planner=self.planner, optimal=self.optimal_human)

    def run(self):
        
        return self.agent.run()

    def end_program(self):
        self.agent.robot.end_program()




if __name__ == '__main__':
    sim = HumanSim(planner=False, optimal_human=False)
    damage = sim.run()
    print("FINAL DAMAGE: ", damage)