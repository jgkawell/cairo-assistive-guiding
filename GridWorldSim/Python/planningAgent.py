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
        self.full_graph = Graph()
        self.full_graph.setup_graph(self.world, self.world_size)

        self.plan()
        self.move()

    def plan(self):
        found_sol = False
        
        if not found_sol:
            for i in range(5):
                self.simplifyWorld(level=i)


        # show vertices marked as intersections

        for vertex in self.simple_graph:
            x, y = vertex.get_xy(self.world_size)
            self.robot.modifyCell(x, y, "Door")

        self.robot.forward()

        # testing a vertex
        vertex = self.simple_graph.get_vertex(699)
        print("Vertex: " + str(vertex.get_xy(self.world_size)))
        neighbors = vertex.get_neighbors()
        for key, info in vertex.get_neighbors().items():
            neighbor = self.simple_graph.get_vertex(key)
            print("Neighbor: " + str(neighbor.get_xy(self.world_size)))
            print("Distance: " + str(info[0]))
            print("Value: " + str(info[1]))


    def simplifyWorld(self, level):

        # first level that just pulls out intersections
        if level == 0:
            # initialize empty graph
            self.simple_graph = Graph()

            # add vertices with a choice-value > 2 (intersections)
            vertex_list = []
            for vertex in self.full_graph:
                if len(vertex.get_neighbors()) > 2:
                    print("Adding: " + str(vertex.get_xy(self.world_size)))
                    vertex_list.append(vertex)

            # connect vertices with edges
            for vertex in vertex_list:
                # copy neighbor list
                neighbors = vertex.get_neighbors().keys()
                # clear neighbor list and add vertex to new graph
                vertex.neighbor_list = {}
                self.simple_graph.add_vertex(vertex)
                
                # recurse through current neighbors to add edges between new vertices
                for key in neighbors:
                    new_key, distance, value = self.generateEdge(vertex.key, key, distance=1, value=0)
                    if new_key != -1:
                        self.simple_graph.add_edge(vertex.key, new_key, distance, value)
        # second level which
        # elif level == 1:


            


    # recurses through path from vertex with key_a to another (already defined) vertex
    # returns a tuple = (key, distance, value) that represents the edge
    def generateEdge(self, key_a, key_b, distance, value):
        if key_b in self.simple_graph.get_vertices():
            # if the key is already a vertex, return the vertex with the distance and value
            # up to that point in the recursion
            return (key_b, distance, value)
        else:
            # else keep recursing until either a vertex or deadend is found
            cur_vertex = self.full_graph.get_vertex(key_b)
            neighbors = cur_vertex.get_neighbors().keys()
            if len(neighbors) > 1:
                for next_key in neighbors:
                    # make sure not to recurse back the way we came
                    if next_key != key_a:
                        distance += 1
                        value += cur_vertex.value
                        return self.generateEdge(key_b, next_key, distance, value)
            else:
                # return a bad key to signal deadend
                return (-1, 0, 0)

        



        



        
    def move(self):
        x = 0
        

if __name__ == "__main__":
    Agent = PlanningAgent()
    Agent.run()