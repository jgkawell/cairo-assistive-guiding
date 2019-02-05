# Python 2 and 3 compatibility. Use Python 3 syntax
from __future__ import absolute_import, division, print_function
try:
    input = raw_input  # Python 3 style input()
except:
    pass

try:
    # Python 3 tkinter
    import tkinter.filedialog as fd
except:
    # Else Python 2 Tkinter
    import tkFileDialog as fd

# Setup imports
from grobot import GRobot
from path_planning import *
import pickle


class PlanningAgent():

    def __init__(self):
        # Initialise globals
        self.robot = GRobot("PlanningAgent", colour="yellow")
        self.heading = 90 #0=forward, 90 = right, 180 = down, 270 = left
        self.path = []
        file_name = fd.askopenfilename(filetypes=[("Map Files", "*.map")], initialdir="../Maps/")

        # import world
        new_world = pickle.load(open(file_name, 'rb'))
        
        # take out the buffer walls if old map
        if len(new_world) == 33:
            self.world_size = len(new_world) - 2
            self.world = [[None] * (self.world_size) for i in range(self.world_size)]  # World map
            for i in range(self.world_size):
                for j in range(self.world_size):
                    self.world[i][j] = new_world[i+1][j+1]
        else:
            self.world_size = len(new_world)
            self.world = [[None] * (self.world_size) for i in range(self.world_size)]  # World map
            for i in range(self.world_size):
                for j in range(self.world_size):
                    self.world[i][j] = new_world[i][j]

    def run(self):
        self.plan()
        self.move()

    def plan(self):
        self.simplifyWorld(level=0)

    def simplifyWorld(self, level):
        x = 0



        
    def move(self):
        x = 0
        

if __name__ == "__main__":
    Agent = PlanningAgent()
    Agent.run()