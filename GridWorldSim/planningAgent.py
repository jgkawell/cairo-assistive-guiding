# Python 2 and 3 compatibility. Use Python 3 syntax
from __future__ import absolute_import, division, print_function
try:
    input = raw_input  # Python 3 style input()
except:
    pass

# Setup imports
from grobot import GRobot
from path_planning import Graph, Path
from timeit import default_timer as timer
import path_planning
import pickle
import time
import copy
import sys


class PlanningAgent():

    def __init__(self):
        # Initialise globals
        self.robot = GRobot("PlanningAgent", colour="yellow")

        # human agent variables
        self.human_graph = None
        self.human_position = 1
        self.human_name = "HumanAgent"

        # abstraction/optimization variables
        self.value_limit = -sys.maxsize
        self.goal_keys = []
        self.show_abstraction = False
        self.desired_path = Path()
        self.mitigation_path = Path()

        # import world
        self.world = pickle.load(open(self.robot.get_cur_file(), 'rb'))
        self.world_size = len(self.world)

    def run(self):
        print("Running...")

        # set up the real true graph world
        self.real_graph = Graph()
        self.real_graph.setup_graph(self.world, self.world_size)

        # get reward keys
        for vertex in self.real_graph:
            if vertex.cell_type == "Reward":
                self.goal_keys.append(vertex.key)

        # get human graph from sim
        self.human_graph = self.getHumanGraph()


        # request the current human position from the sim
        self.human_position = self.real_graph.get_key(self.robot.get_xy_pos(self.human_name))

        # run plan and move until human reaches a goal
        while self.human_position not in self.goal_keys:
            # plan action
            found_sol = self.plan()

            if found_sol:
                # execute action
                self.move()

            # allow the human to move
            self.robot.set_can_human_move(True)

            # request the current human position from the sim
            self.human_position = self.real_graph.get_key(self.robot.get_xy_pos(self.human_name))

    # gets the human_graph from the sim
    def getHumanGraph(self):
        sys.modules['path_planning'] = path_planning
        return pickle.loads(self.robot.get_cur_human_graph())

    # plan a path to execute
    def plan(self):
        # if the human has diverged from the desired path, replan
        if self.human_position not in self.desired_path.vertex_keys:
            found_sol = False
            maxed = False
            level = 0
            old_size = 0

            # loop plan until a solution is found or the abstraction is maxed
            while not found_sol and not maxed:

                start = timer()

                # simplify for certain level
                new_size = self.simplifyWorld(old_size, level=level)
                level += 1

                # check to see if abstraction has changed sizes; if not, stop
                if old_size == new_size:
                    maxed = True
                else:
                    old_size = new_size

                # find best solution for the current abstraction level
                # and save to self.desired_path and self.mitigation_path
                found_sol = self.findSolution(self.human_position)

                end = timer()
                total = end - start
                print("Took: ", total)

                # pauses
                time.sleep(0.25)

            return found_sol

    def findSolution(self, start_key):
            # find all solution paths and rank them by total value
            best_paths = path_planning.find_paths(self.abstract_graph, start_key, self.goal_keys, self.value_limit)
            best_paths.sort(key=lambda x: x.total, reverse=True)

            # generate the expected human path
            human_path = path_planning.a_star(self.human_graph, start_key, self.goal_keys)

            # find paths with too low of a total value
            remove_list = []
            for path in best_paths:
                if path.total < 0:
                    remove_list.append(path)

            # remove paths with too low of a total value
            for path in remove_list:
                best_paths.remove(path)

            # print best paths info
            print("Found paths: ", len(best_paths))
            for path in best_paths:
                print("Robot: (Distance, Value, Total): ", (path.distance, path.value, round(path.total, 3)))

            # print human path info
            print("Human: (Distance, Value, Total): ", (human_path.distance, human_path.value, round(human_path.total, 3)))

            # find the locations for obstacles for each path
            obstacles_for_paths = {}
            for path in best_paths:
                # copy graph for recursion
                copy_graph = copy.deepcopy(self.human_graph)
                obstacles_for_paths[path] = []
                self.findObstaclePlacements(copy_graph, obstacles_for_paths, path, human_path)


            # iterate through all the obstacle lists to find a solution
            found_sol = False
            for obstacle_list in obstacles_for_paths.values():
                found_sol = self.planMitigationPath(obstacle_list, self.real_graph.get_vertex(start_key))

            return True

    def planMitigationPath(self, obstacle_list, human_position):
        found_sol = False
        copy_graph = copy.deepcopy(self.real_graph)
        mitigation_paths = {}
        robot_position = self.real_graph.get_key((self.robot.posx, self.robot.posy))

        # iterate through all the obstacles in the list and find the mitigation paths
        for obstacle in obstacle_list:
            obstacle_vertex = self.real_graph.get_vertex(obstacle)
            robot_path_to_obstacle = path_planning.a_star(self.real_graph, robot_position, [obstacle_vertex])
            human_path_to_obstacle = path_planning.a_star(self.human_graph, human_position, [obstacle_vertex])

            # check to see if the robot can get their before the human
            if robot_path_to_obstacle.distance < human_path_to_obstacle.distance:
                found_sol = True
                for key in robot_path_to_obstacle:
                    self.mitigation_path.add_vertex(key, new_distance=1, new_value=0)


        return found_sol

    # finds the obstacles to place that forces the human on the desired path
    # recursively checks the human's predicted route after each obstacle to generate list
    def findObstaclePlacements(self, copy_graph, obstacles_for_paths, path, human_path):
        # find obstacle location
        key_from, key_to = self.findDivergence(path.vertex_keys, human_path.vertex_keys)

        # check for bad key
        if key_from != -1:
            # print for debug
            xy_from = self.real_graph.get_vertex(key_from).get_xy(self.world_size)
            xy_to = self.real_graph.get_vertex(key_to).get_xy(self.world_size)
            print("Need obstacle between: ", (xy_from, xy_to))

            # save the obstacle location
            obstacles_for_paths[path].append((key_from, key_to))

            # apply obstacle (remove edge)
            self.removeEdgeFromGivenGraph(copy_graph, key_from, key_to)

            # generate the expected human path after obstacle
            human_path_new = path_planning.a_star(copy_graph, key_from, self.goal_keys)
            # recurse to find obtacles with this new human path
            obstacles_for_paths = self.findObstaclePlacements(copy_graph, obstacles_for_paths, path, human_path_new)
        else:
            return obstacles_for_paths

    # finds the point at which the human will diverge from the desired path
    # this is done by looking the the predicted human path and comparing
    # it with the desired path
    def findDivergence(self, desired_path, human_path, cur_pos=0):
        key_from = -1
        key_to = -1

        # iterate through desired path and check for divergence
        for i in range(len(desired_path)):
            key_desired = desired_path[i]
            key_human = human_path[i]

            # if the desired key doesn't match the human key then return
            # the human key and the previous human key
            if key_desired != key_human:
                key_from = human_path[i-1]
                key_to = key_human
                del(desired_path[0:i-1])
                break

        return key_from, key_to

    def move(self):
        return

    # given two keys in the abstract graph, this removes the keys from
    # real_graph and propagates the change to the sim
    def removeEdgeFromRealGraph(self, key_a, key_b):
       # remove from local real world
        self.removeEdgeFromGivenGraph(self.real_graph, key_a, key_b)

        # remove from real world in sim
        self.robot.remove_edge(key_a, key_b)

    # bidirectional removal of and edge given the graph and keys
    def removeEdgeFromGivenGraph(self, graph, key_a, key_b):
        self.removeSingleEdgeFromGivenGraph(graph, key_a, key_b)
        self.removeSingleEdgeFromGivenGraph(graph, key_b, key_a)

    # removes the edge in a single direction
    def removeSingleEdgeFromGivenGraph(self, graph, from_key, to_key):
        # pull out neighbors of from vertex
        neighbors_from = graph.get_vertex(from_key).get_neighbors()

        # remove neighbor with matching key
        for key in neighbors_from.keys():
            if key == to_key:
                del(neighbors_from[key])
                break

    # creates the abstract representaion of the world
    # self.real_graph -> self.abstract_graph
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
            if self.show_abstraction: self.robot.modify_cell_look(x, y, "Door")

        return size

    # add vertices with a choice-value > 2 (intersections) AND reward vertices AND human position
    def generateInitialVertices(self):
        key_list = []
        for vertex in self.real_graph:
            if len(vertex.get_neighbors()) > 2 or vertex.cell_type == "Reward" or vertex.key == self.human_position:
                # create new vertex
                copy_vertex = copy.deepcopy(vertex)

                # add to key list for edge generation
                key_list.append(vertex.key)

                # clear neighbors and add to abstract_graph
                copy_vertex.neighbor_list = {}
                self.abstract_graph.add_vertex(copy_vertex)

        return key_list

    # creates new vertices between previous vertices given a distance limit
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

    # generates the edges between vertices given a list of vertex keys
    def generateEdges(self, key_list):
        # iterate through list of keys creating edges for each one
        for cur_key in key_list:
            # copy neighbor list
            real_neighbors = self.real_graph.get_vertex(cur_key).get_neighbors()

            # iterate through real neighbors to build edges with direction, distance, and valuation
            for neighbor_key, neighbor_info in real_neighbors.items():
                # find info for new edge
                new_key, new_distance, new_value = self.findEdgeInfo(cur_key, neighbor_key, distance=0, value=0)

                # make sure the key is valid
                if new_key != -1:
                    # pull out the neighbor direction
                    direction = neighbor_info[0]

                    # add the edge to the abstract graph
                    self.abstract_graph.add_edge(cur_key, new_key, direction, new_distance, new_value)

    # recurses through path from vertex with key_a to max depth or another already defined vertex (whichever comes first)
    # returns a tuple = (key, distance, value) that represents the edge
    def findEdgeInfo(self, key_a, key_b, distance, value):
        # pull out current vertex
        cur_vertex = self.real_graph.get_vertex(key_b)

        if key_b in self.abstract_graph.get_vertices():
            # if the key is already a vertex, return the vertex or the max depth has been reached
            # with the distance and value up to that point in the recursion
            distance += 1
            value += cur_vertex.value
            return (key_b, distance, value)
        else:
            # else keep recursing until either a vertex or deadend is found
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

    # recurses to create a new neighbor at the max depth
    # given the keys to move between
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

if __name__ == "__main__":
    Agent = PlanningAgent()
    Agent.run()
