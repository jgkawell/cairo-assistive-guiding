# Python 2 and 3 compatibility. Use Python 3 syntax
from __future__ import absolute_import, division, print_function

try:
    input = raw_input  # Python 3 style input()
except:
    pass

from planningAgent import PlanningAgent


# TODO: (1)Prevent simulator from erroring out when jobs complete (2)Fix wierd behavior from both agents (race condition maybe?)
class RobotSim():
    # bools for (1) having a PlanningAgent (2) Using graph abstraction (3) Optimal vs non-optimal, probabilistic human (4) whether or not PlanningAgent uses a probabilistic model
    def __init__(self, planner=True, abstract=True, optimal_human=False, probabilistic_model=False):
        self.planner = planner
        self.abstract = abstract
        self.optimal_human = optimal_human
        self.probabilistic_model = probabilistic_model
        self.agent = PlanningAgent(abstract=self.abstract, probabilistic=self.probabilistic_model)

    def run(self):
        return self.agent.run()


    def end_program(self):
        self.agent.robot.end_program()

if __name__ == '__main__':
    sim = RobotSim()
    avg_time = sim.run()
    print("AVERAGE RUNTIME: ", avg_time)
    sim.end_program()