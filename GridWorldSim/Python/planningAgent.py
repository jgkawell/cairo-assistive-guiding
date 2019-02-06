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
        if len(new_world) == 34:
            self.world_size = len(new_world) - 3
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
        self.G = Graph()
        self.G.setup_graph(self.world, self.world_size)

        self.plan()
        self.move()

    def plan(self):
        found_sol = False
        
        if not found_sol:
            for i in range(1):
                self.simplifyWorld(level=i)

        for vertex in self.simple_graph:
            print(vertex.get_xy(self.world_size))

        # testing a vertex
        vertex = self.simple_graph.get_vertex(699)
        print("Vertex: " + str(vertex.get_xy(self.world_size)))
        for neighbor in vertex.get_neighbors():
            print("Neighbor: " + str(neighbor.get_xy(self.world_size)))


    def simplifyWorld(self, level):
        if level == 0:
            # initialize empty graph
            self.simple_graph = Graph()

            # add vertices with a choice value > 2 (intersections)
            for vertex in self.G:
                if len(vertex.get_neighbors()) > 2:
                    print("Adding: " + str(vertex.get_xy(self.world_size)))
                    self.simple_graph.add_vertex(vertex)

            # connect vertices with edges
            for vertex in self.simple_graph:
                # copy neighbor list
                neighbors = vertex.get_neighbors()
                # clear neighbor list
                vertex.neighbor_list = {}
                
                for neighbor in neighbors:
                    new_key = self.recurse(vertex.key, neighbor.key)
                    if new_key != -1:
                        self.simple_graph.add_edge(vertex.key, new_key)


    def recurse(self, key_a, key_b):
        if key_b in self.simple_graph.get_vertices():
            # if the key is already a vertex, return the vertex
            return key_b
        else:
            # else keep recursing until either a vertex or deadend is found
            neighbors = self.G.get_vertex(key_b).get_neighbors()
            if len(neighbors) > 1:
                for next_neighbor in neighbors:
                    # make sure not to recurse back the way we came
                    if next_neighbor.key != key_a:
                        return self.recurse(key_b, next_neighbor.key)
            else:
                # return a bad key to signal deadend
                return -1

        



        



        
    def move(self):
        x = 0
        

if __name__ == "__main__":
    Agent = PlanningAgent()
    Agent.run()