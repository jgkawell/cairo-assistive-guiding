# Python 2 and 3 compatibility. Use Python 3 syntax
from __future__ import absolute_import, division, print_function
try:
    input = raw_input  # Python 3 style input()
except:
    pass

# Setup imports
from grobot import GRobot
from path_planning import Graph
import path_planning
import pickle
import time
import copy
import sys


class PlanningAgent():

    def __init__(self):
        # Initialise globals
        self.robot = GRobot("PlanningAgent", colour="yellow")
        self.heading = 90 #0=forward, 90 = right, 180 = down, 270 = left
        self.path = []
        self.humanGraph = None

        # get file name from simulator
        file_name = self.robot.getFile()
        # if file_name[0] == ".":
        #     file_name = "../" + file_name

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
        self.real_graph = Graph()
        self.real_graph.setup_graph(self.world, self.world_size)
        self.humanGraph = self.getHumanGraph()
        print("Received Graph ", self.humanGraph)
        print(self.humanGraph)
        self.plan()
        self.move()

    def getHumanGraph(self):
        sys.modules['path_planning'] = path_planning
        return pickle.loads(self.robot.getGraph())

    # plan a path to execute
    def plan(self):
        found_sol = False
        maxed = False
        level = 0

        old_size = 0
        while not found_sol and not maxed:

            # simplify for certain level
            new_size = self.simplifyWorld(old_size, level=level)
            level += 1

            # check to see if abstraction has changed sizes, if not, stop
            if old_size == new_size:
                maxed = True
            else:
                old_size = new_size

            # find solution path
            found_sol = self.findSolution()

            # pauses
            time.sleep(1)

    def simplifyWorld(self, world_size, level):
        # first level that just pulls out intersections
        if level == 0:
            # initialize empty graph
            self.abstract_graph = Graph()
            # generate the intial set of vertices
            key_list = self.generateInitialVertices()
            # generate the new edges between vertices
            self.generateEdges(key_list)

        # other levels which adds in intermediate vertices
        else:
            # set distance limit for generating new vertices
            distance_limit = 5 - level
            if distance_limit < 1:
                distance_limit = 1
            # generate new vertices between current vertices
            key_list = self.generateIntermediateVertices(distance_limit)
            # generate the new edges between vertices
            self.generateEdges(key_list)

        # show and count current vertices
        size = 0
        for vertex in self.abstract_graph:
            size += 1
            x, y = vertex.get_xy(self.world_size)
            self.robot.modifyCellLook(x, y, "Door")

        return size

    # add vertices with a choice-value > 2 (intersections) AND reward vertices
    def generateInitialVertices(self):
        key_list = []
        for vertex in self.real_graph:
            if len(vertex.get_neighbors()) > 2 or vertex.cell_type == "Reward":
                # create new vertex
                copy_vertex = copy.deepcopy(vertex)

                # add to key list for edge generation
                key_list.append(vertex.key)

                # clear neighbors and add to abstract_graph
                copy_vertex.neighbor_list = {}
                self.abstract_graph.add_vertex(copy_vertex)

        return key_list

    def generateIntermediateVertices(self, distance_limit):
        # pull out the current vertex keys to find new neighbors for
        start_vertices = []
        for key in self.abstract_graph.get_vertices():
            start_vertices.append(key)

        # keep track of new vertex keys to add
        key_list = []

        # iterate through current vertex keys to find neighbors
        for start_key in start_vertices:
            # pull out current vertex to search, copy neighbors, and clear neighbors
            start_vertex = self.abstract_graph.get_vertex(start_key)
            abstract_neighbors = copy.deepcopy(start_vertex.get_neighbors())
            start_vertex.neighbor_list = {}

            # iterate through current neighbors and find intermediate vertices along the paths between
            for end_info in abstract_neighbors.values():
                # pull out the neighbor distance
                abstract_distance = end_info[1]

                # check to see if the vertex has already been completed and if the distance is great enough
                if abstract_distance > distance_limit:
                    # pull out the neighbor direction and the real neighbors
                    abstract_direction = end_info[0]
                    real_neighbors = self.real_graph.get_vertex(start_key).get_neighbors()

                    # iterate through the real neighbors to find the new, intermediate neighbor
                    for real_key, real_info in real_neighbors.items():
                        # pull out the real direction
                        real_direction = real_info[0]

                        # check to make sure the directions match for the neighbors
                        if real_direction == abstract_direction:
                            # create new vertex
                            max_depth = int(abstract_distance/2)
                            if max_depth == 0:
                                max_depth += 1

                            new_key = self.findAbstractNeighbor(start_key, real_key, cur_depth=1, max_depth=max_depth)

                            # error check
                            if new_key == -1:
                                print("ERROR")
                            else:
                                # make deep copy of the new vertex
                                new_vertex = copy.deepcopy(self.real_graph.get_vertex(new_key))

                                # add to key list for edge generation
                                key_list.append(new_key)

                                # clear neighbors and add to abstract_graph
                                new_vertex.neighbor_list = {}
                                self.abstract_graph.add_vertex(new_vertex)

        # add other vertices to key list for edge generation
        for vertex in self.abstract_graph:
            key_list.append(vertex.key)

        return key_list

    def generateEdges(self, key_list):
        # iterate through list of keys creating edges for each one
        for cur_key in key_list:
            # copy neighbor list
            real_neighbors = self.real_graph.get_vertex(cur_key).get_neighbors()

            # iterate through real neighbors to build edges with direction, distance, and valuation
            for neighbor_key, neighbor_info in real_neighbors.items():
                # find info for new edge
                new_key, new_distance, new_value = self.findEdgeInfo(cur_key, neighbor_key, distance=1, value=0)

                # make sure the key is valid
                if new_key != -1:
                    # pull out the neighbor direction
                    direction = neighbor_info[0]

                    # add the edge to the abstract graph
                    self.abstract_graph.add_edge(cur_key, new_key, direction, new_distance, new_value)

    # recurses through path from vertex with key_a to max depth or another already defined vertex (whichever comes first)
    # returns a tuple = (key, distance, value) that represents the edge
    def findEdgeInfo(self, key_a, key_b, distance, value):
        if key_b in self.abstract_graph.get_vertices():
            # if the key is already a vertex, return the vertex or the max depth has been reached
            # with the distance and value up to that point in the recursion
            return (key_b, distance, value)
        else:
            # else keep recursing until either a vertex or deadend is found
            cur_vertex = self.real_graph.get_vertex(key_b)
            neighbors = cur_vertex.get_neighbors()
            if len(neighbors) > 1:
                for key_c in neighbors.keys():
                    # make sure not to recurse back the way we came (a -> b -> c)
                    if key_c != key_a:
                        distance += 1
                        value += cur_vertex.value
                        return self.findEdgeInfo(key_b, key_c, distance, value)
            else:
                # return a bad key to signal deadend
                return (-1, 0, 0)

    def findAbstractNeighbor(self, key_a, key_b, cur_depth, max_depth):
        if cur_depth == max_depth:
            return key_b
        else:
            # else keep recursing until either a vertex or deadend is found
            cur_vertex = self.real_graph.get_vertex(key_b)
            neighbors = cur_vertex.get_neighbors()
            if len(neighbors) > 1:
                for key_c in neighbors.keys():
                    # make sure not to recurse back the way we came (a -> b -> c)
                    if key_c != key_a:
                        cur_depth += 1
                        return self.findAbstractNeighbor(key_b, key_c, cur_depth, max_depth)
            else:
                # return a bad key to signal deadend
                return -1

    def findSolution(self):
        return False

    def move(self):
        return


if __name__ == "__main__":
    Agent = PlanningAgent()
    Agent.run()
