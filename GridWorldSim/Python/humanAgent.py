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
import numpy as np

class HumanAgent():

    def __init__(self, knowledge=0, optimal=False):
        # Initialise globals
        self.robot = GRobot("HumanAgent", colour="yellow")
        self.heading = 90 #0=forward, 90 = right, 180 = down, 270 = left
        self.path = []
        self.knowledge = knowledge
        self.optimal = optimal
        worldPath = fd.askopenfilename(filetypes=[("Map Files", "*.map")], initialdir="../Maps/")

        # import world
        newworld = pickle.load(open(worldPath, 'rb'))

        # take out the buffer walls if old map
        if len(newworld) == 33:
            self.mapsize = len(newworld) - 2
            self.world = [[None] * (self.mapsize) for i in range(self.mapsize)]  # World map
            for i in range(self.mapsize):
                for j in range(self.mapsize):
                    self.world[i][j] = newworld[i+1][j+1]
        else:
            self.mapsize = len(newworld)
            self.world = [[None] * (self.mapsize) for i in range(self.mapsize)]  # World map
            for i in range(self.mapsize):
                for j in range(self.mapsize):
                    self.world[i][j] = newworld[i][j]

        # Erase hazards from memory
        # TODO: We will need to modify this to remove random rewards as well

        for i in range(0, self.mapsize):
            for j in range(0, self.mapsize):
                if self.world[i][j] == "Hazard":
                    self.world[i][j] = None

        if self.knowledge == 0:
            for i in range(0, self.mapsize):
                for j in range(0, self.mapsize):
                    if self.world[i][j] == "Reward" and np.random.uniform() >= 0.5:
                        self.world[i][j] = None


    def run(self):
        goal, graph = self.plan()
        self.move(goal, graph)

    def plan(self):
        #path plan with a*
        G = Graph(self.mapsize, self.world)
        start_key = G.get_key(1, 0)
        start = G.get_vertex(start_key)
        print("Start: 1, 0")

        goal_key = G.get_key(1, 15)
        goal = G.get_vertex(goal_key)
        print("Goal: 1, 15")
        t = a_star(G, start, goal)
        self.path = list(reversed(t))
        return goal, G

    def move(self, goal, G):
        i = 0
        goal_pose = goal.get_xy(self.mapsize)
        while (self.robot.posx, self.robot.posy) != goal_pose:
            if self.optimal == False and np.random.uniform() <= 0.2:
                print("Random Move")
                coord = (self.robot.posx + np.random.randint(-1, 1), self.robot.posy + np.random.randint(-1, 1))
                self.move_helper(coord, G)

                i = 0
                start = G.get_vertex(G.get_key(self.robot.posx, self.robot.posy))
                start.parent = -1
                t = a_star(G, start, goal)
                self.path = list(reversed(t))

            else:
                coord = self.path[i]
                self.move_helper(coord, G)
                i += 1


    def move_helper(self, coord, G):
        (i, j) = coord
        (x, y) = (i, j)
        direction = (x - self.robot.posx, y - self.robot.posy)
        print(x, y)
        node = G.get_vertex(G.get_key(x, y))
        print((x,y), (self.robot.posx, self.robot.posy))

        if direction == (1, 0): #right
            if self.heading == 0:
                if node.cell_type != "Wall":
                    self.robot.forward()
            else:
                for i in range(int(self.heading / 90)): self.robot.right()
                if node.cell_type != "Wall":
                    self.robot.forward()
                self.heading = 0

            if node.cell_type != "Wall": self.robot.posx += 1

        elif direction == (0, 1): #up
            if self.heading == 90:
                if node.cell_type != "Wall":
                    self.robot.forward()
            elif self.heading > 90:
                for i in range(int((self.heading - 90)/90)): self.robot.right()
                if node.cell_type != "Wall":
                    self.robot.forward()
                self.heading = 90
            else: #facing right
                self.robot.left()
                if node.cell_type != "Wall":
                    self.robot.forward()
                self.heading = 90

            if node.cell_type != "Wall": self.robot.posy += 1

        elif direction == (-1, 0): #left
            if self.heading == 180:
                if node.cell_type != "Wall":
                    self.robot.forward()
            elif self.heading < 180:
                for i in range(int((180-self.heading)/90)): self.robot.left()
                if node.cell_type != "Wall":
                    self.robot.forward()
                self.heading = 180
            else: # facing down = 270
                self.robot.right()
                if node.cell_type != "Wall":
                    self.robot.forward()
                self.heading = 180

            if node.cell_type != "Wall": self.robot.posx -= 1

        elif direction == (0, -1): #down
            if self.heading == 270:
                if node.cell_type != "Wall":
                    self.robot.forward()
            else:
                for i in range(int((270-self.heading)/90)): self.robot.left()
                if node.cell_type != "Wall":
                    self.robot.forward()
                self.heading = 270

            if node.cell_type != "Wall": self.robot.posy -= 1



if __name__ == "__main__":
    Agent = HumanAgent()
    Agent.run()
