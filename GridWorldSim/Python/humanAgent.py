# Python 2 and 3 compatibility. Use Python 3 syntax
from __future__ import absolute_import, division, print_function
try:
    input = raw_input  # Python 3 style input()
except:
    pass

# Setup imports
from grobot import GRobot
from path_planning import *
import pickle

class HumanAgent():

    def __init__(self):
        # Initialise globals
        self.robot = GRobot("HumanAgent", colour="yellow")
        self.heading = 90 #0=forward, 90 = right, 180 = down, 270 = left
        self.path = []
        worldPath = "./../Maps/fullOffice.map" # this must be the same as that used in RobotGridWorld.pyw (and any other agents operating together)

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

    def run(self):
        self.plan()
        self.move()

    def plan(self):
        #path plan with a*
        G = Graph(self.mapsize, self.world)
        start_key = G.get_key(1, 0)
        start = G.get_vertex(start_key)
        print("Start: 1, 0")

        goal_key = G.get_key(29, 30)
        goal = G.get_vertex(goal_key)
        print("Goal: 29, 30")
        t = a_star(G, start, goal)
        self.path = reversed(t)

    def move(self):
        for coord in self.path:
            (i, j) = coord
            (x, y) = (i, j)
            direction = (x - self.robot.posx, y - self.robot.posy)
            print((x,y), (self.robot.posx, self.robot.posy))
            
            if direction == (1, 0): #right
                if self.heading == 0:
                    self.robot.forward()
                else:
                    for i in range(int(self.heading / 90)): self.robot.right()
                    self.robot.forward()
                    self.heading = 0

                self.robot.posx += 1

            elif direction == (0, 1): #up
                if self.heading == 90:
                    self.robot.forward()
                elif self.heading > 90:
                    for i in range(int((self.heading - 90)/90)): self.robot.right()
                    self.robot.forward()
                    self.heading = 90
                else: #facing right
                    self.robot.left()
                    self.robot.forward()
                    self.heading = 90

                self.robot.posy += 1

            elif direction == (-1, 0): #left
                if self.heading == 180:
                    self.robot.forward()
                elif self.heading < 180:
                    for i in range(int((180-self.heading)/90)): self.robot.left()
                    self.robot.forward()
                    self.heading = 180
                else: # facing down = 270
                    self.robot.right()
                    self.robot.forward()
                    self.heading = 180

                self.robot.posx -= 1

            elif direction == (0, -1): #down
                if self.heading == 270:
                    self.robot.forward()
                else:
                    for i in range(int((270-self.heading)/90)): self.robot.left()
                    self.robot.forward()
                    self.heading = 270

                self.robot.posy -= 1

if __name__ == "__main__":
    Agent = HumanAgent()
    Agent.run()