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
import time


class PlanningAgent():

    def __init__(self):
        # Initialise globals
        self.robot = GRobot("PlanningAgent", colour="yellow")
        self.heading = 90 #0=forward, 90 = right, 180 = down, 270 = left
        self.path = []
        # file_name = fd.askopenfilename(filetypes=[("Map Files", "*.map")], initialdir="../Maps/")

        # import world
        new_world = pickle.load(open("/home/jgkawell/Documents/Repos/Boulder/cairo-assistive-guiding-project/GridWorldSim/Maps/NewMaze.map", 'rb'))
        
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
                # simplify for certain level
                self.simplifyWorld(level=i)

                # show vertices found
                for vertex in self.abstract_graph:
                    x, y = vertex.get_xy(self.world_size)
                    self.robot.modifyCell(x, y, "Door")

                # pauses
                time.sleep(1)


    def simplifyWorld(self, level):

        # first level that just pulls out intersections
        if level == 0:
            # initialize empty graph
            self.abstract_graph = Graph()

            # add vertices with a choice-value > 2 (intersections)
            key_list = []
            for vertex in self.full_graph:
                if len(vertex.get_neighbors()) > 2:
                    print("Adding: " + str(vertex.get_xy(self.world_size)))
                    key_list.append(vertex.key)

                    copy_vertex = vertex

                    # clear neighbors and add to abstract_graph
                    copy_vertex.neighbor_list = {}
                    self.abstract_graph.add_vertex(copy_vertex)

            self.generateEdges(key_list)
            
        # second level which adds in intermediate vertices
        elif level == 1:
            distance_limit = 5

            start_vertices = self.abstract_graph.get_vertices()
            completed_paths = []
            key_list = []

            for start_key in start_vertices:
                abstract_neighbors = self.abstract_graph.get_vertex(start_key).get_neighbors()

                for end_key, end_info in abstract_neighbors.items():
                    if (start_key, end_key) not in completed_paths:
                        abstract_distance = end_info[1]
                        if  abstract_distance > distance_limit:
                            abstract_direction = end_info[0]
                            full_neighbors = self.full_graph.get_vertex(start_key).get_neighbors()

                            for full_key, full_info in full_neighbors.items():
                                full_direction = full_info[0]
                                if full_direction == abstract_direction:
                                    # find new vertex
                                    new_key = self.generateAbstractNeighbor(start_key, full_key, cur_depth=1, max_depth=int(abstract_distance/2))
                                    new_vertex = self.full_graph.get_vertex(new_key)

                                    # add to completed paths to keep from finding another new vertex
                                    completed_paths.extend([(start_key, full_key), (full_key, start_key), (end_key, full_key), (full_key, end_key)])

                                    # add to key list for edge generation
                                    key_list.append(new_key)

                                    # clear neighbors and add to abstract_graph
                                    new_vertex.neighbor_list = {}
                                    self.abstract_graph.add_vertex(new_vertex)

                                    print("Adding: " + str(new_vertex.get_xy(self.world_size)))




            
            for vertex in self.abstract_graph:
                key_list.append(vertex.key)

            self.generateEdges(key_list)

            
    def generateEdges(self, key_list):

        # connect vertices with edges
        for cur_key in key_list:
            # copy neighbor list
            neighbors = self.full_graph.get_vertex(cur_key).get_neighbors()
            
            # recurse through current neighbors to add edges between new vertices
            for next_key, info in neighbors.items():
                new_key, distance, value = self.findAbstractNeighbor(cur_key, next_key, distance=1, value=0)
                if new_key != -1:
                    direction = info[0]
                    self.abstract_graph.add_edge(cur_key.key, new_key, direction, distance, value)

    # recurses through path from vertex with key_a to another (already defined) vertex
    # returns a tuple = (key, distance, value) that represents the edge
    def findAbstractNeighbor(self, key_a, key_b, distance, value):
        if key_b in self.abstract_graph.get_vertices():
            # if the key is already a vertex, return the vertex or the max depth has been reached
            # with the distance and value up to that point in the recursion
            return (key_b, distance, value)
        else:
            # else keep recursing until either a vertex or deadend is found
            cur_vertex = self.full_graph.get_vertex(key_b)
            neighbors = cur_vertex.get_neighbors()
            if len(neighbors) > 1:
                for key_c in neighbors.keys():
                    # make sure not to recurse back the way we came (a -> b -> c)
                    if key_c != key_a:
                        distance += 1
                        value += cur_vertex.value
                        return self.findAbstractNeighbor(key_b, key_c, distance, value)
            else:
                # return a bad key to signal deadend
                return (-1, 0, 0)

    def generateAbstractNeighbor(self, key_a, key_b, cur_depth, max_depth):
        if cur_depth == max_depth:
            return key_b
        else:
            # else keep recursing until either a vertex or deadend is found
            cur_vertex = self.full_graph.get_vertex(key_b)
            neighbors = cur_vertex.get_neighbors()
            for key_c in neighbors.keys():
                # make sure not to recurse back the way we came (a -> b -> c)
                if key_c != key_a:
                    cur_depth += 1
                    return self.generateAbstractNeighbor(key_b, key_c, cur_depth, max_depth)

        



        



        
    def move(self):
        x = 0
        

if __name__ == "__main__":
    Agent = PlanningAgent()
    Agent.run()