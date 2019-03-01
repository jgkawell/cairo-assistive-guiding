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
from random import randint
from path_planning import Graph, PlanningPath, a_star
import path_planning
import pickle
import numpy as np
import copy
import time
import sys

class HumanAgent():

    def __init__(self, knowledge=1, optimal=True, planner=True):

        # Initialise globals
        self.robot = GRobot("HumanAgent", colour="blue")
        self.heading = 90 #0=forward, 90 = right, 180 = down, 270 = left
        self.path = []
        self.knowledge = knowledge
        self.optimal = optimal
        self.optimality_constant = 0.05
        self.reward_removal_constant = 0.5
        self.human_graph = None
        self.damage_tracking_graph = None
        self.damage_taken = 0
        self.damage_limit = 0.5
        self.distance_damage = 0.001
        self.start_distance = 10 #start 20 cells away from randomly chosen goal
        self.planner = planner #if no planner, don't wait to move

        # get file name from simulator
        file_name = self.robot.get_cur_file()

        # import world
        self.world = pickle.load(open(file_name, 'rb'))
        self.world_size = len(self.world)

        #To keep track of dmg human accumulates
        self.damage_tracking_graph = Graph()
        self.damage_tracking_graph.setup_graph(self.world, self.world_size)

        # Erase hazards and (some) rewards from memory
        self.reward_states = []
        self.empty_states = []

        for i in range(0, self.world_size):
            for j in range(0, self.world_size):
                cell_type = self.world[i][j]
                if cell_type == "Hazard":
                    # remove all hazard knowledge
                    self.world[i][j] = None
                elif cell_type == "Reward":
                    if self.knowledge == 0 and np.random.uniform() >= self.reward_removal_constant:
                        # remove rewards randomly
                        self.world[i][j] = None
                    else:
                        # save reward state for goal generation
                        self.reward_states.append((i,j))
                elif cell_type == None:
                    # save empy states for start generation
                    self.empty_states.append((i,j))


    def sendGraph(self):
        serialized_graph = pickle.dumps(copy.deepcopy(self.human_graph), protocol=2)
        self.robot._send(serialized_graph, "byte")

    def run(self):
        self.plan()
        self.move()

        # close simulation
        self.robot.set_exited(True)

        print("HUMAN: Exit")
        if not self.planner:
            self.robot.end_program()
        return self.damage_taken

    def plan(self):
        # generate graph world
        self.human_graph = Graph()
        self.human_graph.setup_graph(self.world, self.world_size)
        self.sendGraph()

        # build goal info
        self.goals = []
        for xy in self.reward_states:
            goal = self.human_graph.get_key(xy)
            self.goals.append(goal)

        # generate a start position distant from goal states
        start_x, start_y = 0, 0
        done = False
        while not done:
            done = True
            x1, y1 = self.empty_states[randint(0, len(self.empty_states)-1)]

            for x2, y2 in self.reward_states:
                dist = np.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
                if dist < self.start_distance:
                    done = False

            if done:
                start_x, start_y = x1, y1

        # recreate human agent with start positions
        self.robot = GRobot("HumanAgent", posx=start_x, posy=start_y, colour="orange")

        # build start info
        xy = (start_x, start_y)
        start = self.human_graph.get_key(xy)
        print("HUMAN:  Start: " + str(xy))

        #path plan with A*
        self.path = a_star(self.human_graph, start, self.goals)

    def move(self):
        i = 1
        cur_key = self.human_graph.get_key((self.robot.posx, self.robot.posy))
        can_move = False
        self.robot.set_can_robot_move(True)
        while cur_key not in self.goals:
            # pause before moving
            time.sleep(0.1)
            # check sim to find allowance to move
            while not can_move and self.planner:
                can_move = self.robot.can_human_move()
                if not can_move:
                    # print("HUMAN:  Waiting...")
                    time.sleep(1)
                    
            # check current world state
            valid, changed = self.robot.look()

            # found new world knowledge
            if changed:
                #print("HUMAN:  New world knowledge!")
                # update world knowledge
                self.human_graph = self.getHumanGraph()

                # reset position and find new path
                i = 0
                xy = (self.robot.posx, self.robot.posy)
                start = self.human_graph.get_key(xy)
                self.path = a_star(self.human_graph, start, self.goals)
                del(self.path.vertex_keys[0])
                #print("HUMAN:  New path: ", self.path.vertex_keys)

            # if making a random move, rerun A*
            if self.optimal == False and np.random.uniform() <= self.optimality_constant:
                #print("HUMAN:  Random Move")

                # get a random neighbor
                xy = (self.robot.posx, self.robot.posy)
                cur_neighbors = list(self.human_graph.get_vertex(self.human_graph.get_key(xy)).get_neighbors().keys())
                
                # # only make random move if at intersection
                # if len(cur_neighbors) > 2:
                next_key = cur_neighbors[np.random.randint(0, len(cur_neighbors))]

                # generate path in direction of random move
                self.path = self.getHumanPathByDirection(self.human_graph, cur_key, next_key, cur_neighbors)
                i = 0

                # make sure there is a valid path
                if len(self.path.vertex_keys) == 0:
                    self.path = path_planning.a_star(self.human_graph, cur_key, self.goals)

                coord = self.human_graph.get_vertex(self.path.vertex_keys[i]).get_xy(self.world_size)

                # move and reset cur_key
                self.move_helper(coord)
                cur_key = self.human_graph.get_key((self.robot.posx, self.robot.posy))
                #accumulate damage
                temp_damage = self.damage_tracking_graph.get_vertex(cur_key).cost + self.distance_damage
                self.damage_taken += temp_damage
                self.robot.set_human_damage(self.damage_taken)
                #human and planner take turns moving
                self.robot.set_can_robot_move(True)
                can_move = False

                i += 1

            else:
                coord = self.human_graph.get_vertex(self.path.vertex_keys[i]).get_xy(self.world_size)

                # move and reset cur_key
                self.move_helper(coord)
                cur_key = self.human_graph.get_key((self.robot.posx, self.robot.posy))
                #accumulate damage
                temp_damage = self.damage_tracking_graph.get_vertex(cur_key).cost + self.distance_damage
                self.damage_taken += temp_damage
                self.robot.set_human_damage(self.damage_taken)

                #human and planner take turns moving
                self.robot.set_can_robot_move(True)
                can_move = False

                i += 1

            if self.damage_taken >= self.damage_limit:
                #print("HUMAN: Damage limit reached (FAILURE)")
                return


    def getHumanGraph(self):
        sys.modules['path_planning'] = path_planning
        return pickle.loads(self.robot.get_cur_human_graph())

    def move_helper(self, coord):
        (x, y) = coord
        direction = (x - self.robot.posx, y - self.robot.posy)
        #print("HUMAN:  Current, Intended:", (self.robot.posx, self.robot.posy), (x, y))

        # check for success
        msg = ""

        # heading: 0=E, 90=N, 180=W, 270=S

        # east
        if direction == (1, 0):
            if self.heading == 0: # E
                msg = self.robot.move_forward()
            elif self.heading == 90: # N
                self.robot.move_right()
                msg = self.robot.move_forward()
            elif self.heading == 180: # W
                self.robot.move_right()
                self.robot.move_right()
                msg = self.robot.move_forward()
            elif self.heading == 270: # S
                self.robot.move_left()
                msg = self.robot.move_forward()

            self.heading = 0

            if msg == "OK": self.robot.posx += 1

        # north
        elif direction == (0, 1):
            if self.heading == 0: # E
                self.robot.move_left()
                msg = self.robot.move_forward()
            elif self.heading == 90: # N
                msg = self.robot.move_forward()
            elif self.heading == 180: # W
                self.robot.move_right()
                msg = self.robot.move_forward()
            elif self.heading == 270: # S
                self.robot.move_left()
                self.robot.move_left()
                msg = self.robot.move_forward()

            self.heading = 90

            if msg == "OK": self.robot.posy += 1

        # west
        elif direction == (-1, 0):
            if self.heading == 0: # E
                self.robot.move_left()
                self.robot.move_left()
                msg = self.robot.move_forward()
            elif self.heading == 90: # N
                self.robot.move_left()
                msg = self.robot.move_forward()
            elif self.heading == 180: # W
                msg = self.robot.move_forward()
            elif self.heading == 270: # S
                self.robot.move_right()
                msg = self.robot.move_forward()

            self.heading = 180

            if msg == "OK": self.robot.posx -= 1

        # south
        elif direction == (0, -1):
            if self.heading == 0: # E
                self.robot.move_right()
                msg = self.robot.move_forward()
            elif self.heading == 90: # N
                self.robot.move_left()
                self.robot.move_left()
                msg = self.robot.move_forward()
            elif self.heading == 180: # W
                self.robot.move_left()
                msg = self.robot.move_forward()
            elif self.heading == 270: # S
                msg = self.robot.move_forward()

            self.heading = 270

            if msg == "OK": self.robot.posy -= 1

    # get the human's predicted path in a certain direction
    def getHumanPathByDirection(self, graph, cur_key, next_key, neighbors):
        # create a new copy to add temporary obstacles to
        copy_graph = copy.deepcopy(graph)

        # force path_planning to generate human path in given direction to accumulate cost
        # by removing other neighbors
        for temp_key in neighbors:
            if temp_key != next_key:
                self.removeEdgeFromGivenGraph(copy_graph, cur_key, temp_key)

        # generate the cost by using a_star (human model)
        return path_planning.a_star(copy_graph, cur_key, self.goals)

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


if __name__ == "__main__":
    # sys.argv: 0=name, 1=knowledge, 2=optimal
    # Agent = HumanAgent(sys.argv[1], sys.argv[2])
    Agent = HumanAgent()
    Agent.run()
