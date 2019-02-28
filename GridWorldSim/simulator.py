# Python 2 and 3 compatibility. Use Python 3 syntax
from __future__ import absolute_import, division, print_function
try:
    input = raw_input  # Python 3 style input()
except:
    pass

import multiprocessing as mp
from humanAgent import HumanAgent
from planningAgent import PlanningAgent

#wrapper for human & planner agents to make them easily multithreadable
class Agent(object):
    def __init__(self, agent, type):
        self.agent = agent
        self.type = type

    def run(self):
        self.agent.run()

#TODO: (1)Prevent simulator from erroring out when jobs complete (2)Fix wierd behavior from both agents (race condition maybe?)
class Simulator():
    #bools for (1) having a PlanningAgent (2) Using graph abstraction (3) Optimal vs non-optimal, probabilistic human (4) whether or not PlanningAgent uses a probabilistic model
    def __init__(self, planner=True, abstract=True, optimal_human=False, probabilistic_model=False):
        self.planner = planner
        self.abstract = abstract
        self.optimal_human = optimal_human
        self.probabilistic_model = probabilistic_model

    def run(self):
        #if we enable PlanningAgent, need to run human and planner concurrently. Else, just run the human and get its output
        if self.planner:
            manager = mp.Manager()
            return_dict = manager.dict()


            agents = []
            human_agent = Agent(agent=HumanAgent(self.optimal_human), type="human")
            planning_agent = Agent(agent=PlanningAgent(self.abstract, self.probabilistic_model), type="planner")
            agents.append(human_agent)
            agents.append(planning_agent)

            jobs = []
            for agent in agents:
                print("Running job for: ", agent.type)
                p = mp.Process(target=self.get_results, args=(return_dict, agent))
                jobs.append(p)
                p.start()

            for p in jobs:
                p.join()

            print("SIM: Done...")


            for agent, output in return_dict.values():
                #TODO: Make planner return something more useful
                #human agent returns total damage inflicted, planner returns whether abstract is true or false
                print(agent.type, output)

        else:
            human_agent = HumanAgent(self.optimal_human)
            output = human_agent.run()
            print("Human w/ no assistance: ", output)


    def get_results(self, return_dict, agent):
        return_dict[agent] = agent.run()


if __name__ == '__main__':
    sim = Simulator()
    sim.run()