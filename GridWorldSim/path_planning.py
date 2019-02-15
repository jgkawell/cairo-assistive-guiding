import sys
import numpy as np
import heapq
import copy

class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

class Vertex():
    def __init__(self, key, world_size, cell_type = None):
        self.key = key
        self.cell_type = cell_type
        self.value = 0
        self.neighbor_list = {}
        self.parent = -1
        self.dist = sys.maxsize #distance to start vertex 0
        self.visited = False
        self.xy = self.get_xy(world_size)

        if cell_type == "Reward":
            self.value = 1
        elif cell_type == "Hazard":
            self.value = -0.25

    # adds a neighbor to the vertex with a distance and value across the edge
    # direction: 1=N, 2=E, 3=S, 4=W
    def add_neighbor(self, key, direction, distance=1, value=0):
        self.neighbor_list[key] = (direction, distance, value)

    def get_neighbors(self):
        return self.neighbor_list

    def get_xy(self, world_size):
        x = self.key % world_size
        y = int((self.key - x) / world_size)
        return (x, y)

    def __lt__(self, other):
        if self.dist < other.dist: return True
        return False

    def __eq__(self, other):
        if self.dist == other.dist: return True
        return False

    def __hash__(self):
        return hash(self.key)

class Graph():
    def __init__(self, directed=False):
        self.vertices = {}
        self.num_walls = 0

    def setup_graph(self, world, world_size):
        self.world_size = world_size

        for i in range(0, world_size):
            for j in range(0, world_size):
                key = i + world_size * j
                celltype = world[i][j]
                v = Vertex(key, self.world_size, celltype)
                self.add_vertex(v)

        for i in range(world_size**2):
            v_cur = self.get_vertex(i)
            if v_cur.cell_type != "Wall": # 'i' rows by 'j' columns, key = world_size * i + j
                if i + world_size < world_size**2 and self.get_vertex(i+world_size).cell_type != "Wall":
                    self.add_edge(i, i+world_size, direction=1) #cell north
                if i % world_size != world_size-1 and self.get_vertex(i+1).cell_type != "Wall":
                    self.add_edge(i, i+1, direction=2) #cell east
                if i - world_size > 0 and self.get_vertex(i-world_size).cell_type != "Wall":
                    self.add_edge(i, i-world_size, direction=3) #cell south
                if i % world_size != 0 and self.get_vertex(i-1).cell_type != "Wall":
                    self.add_edge(i, i-1, direction=4) #cell west


    def add_vertex(self, vertex):
        self.vertices[vertex.key] = vertex

    def get_vertex(self, key):
        return self.vertices[key]

    def get_vertices(self):
        return self.vertices.keys()

    def get_key(self, xy):
        x = xy[0]
        y = xy[1]
        return x + self.world_size * y

    def __iter__(self):
        return self.vertices.values().__iter__()

    def add_edge(self, from_key, to_key, direction, distance=1, value=0):
        self.vertices[from_key].add_neighbor(to_key, direction, distance, value)

class Path():
    def __init__(self, vertex_keys=[], distance=0, value=0):
        self.vertex_keys = vertex_keys
        self.distance = distance
        self.value = value
        self.total = self.calculate_total()
        self.size = len(self.vertex_keys)
        self.idx = 0
        self.obstacle = None

        # TODO: Change obstacle to list of obstacles or add to vertex object

    def add_vertex(self, new_key, new_distance, new_value, obstacle=None):
        self.vertex_keys.append(new_key)
        self.distance += new_distance
        self.value += new_value
        self.total = self.calculate_total()
        self.size += 1
        self.obstacle = obstacle

    def calculate_total(self):
        return self.value - (0.001 * self.distance)

    def __iter__(self):
        return self

    def __next__(self):
        self.idx += 1
        try:
            return self.vertex_keys[self.idx-1]
        except IndexError:
            self.idx = 0
            raise StopIteration

def heuristic(goal_pos, vertex_pos): #manhattan distance - admissible
    (x1, y1) = goal_pos
    (x2, y2) = vertex_pos
    return abs(x1 - x2) + abs(y1 - y2)

def trace(vertex, graph):
    trace = []
    trace.append(vertex.get_xy(graph.world_size))
    curr = vertex
    while curr.parent != -1:
        v_parent = graph.get_vertex(curr.parent)
        trace.append(v_parent.get_xy(graph.world_size))
        curr = v_parent
    return trace

def a_star(graph, start_key, goal_keys): #pass in start vertex, goal vertices
    copy_graph = copy.deepcopy(graph)

    # converte keys to vertices
    start = copy_graph.get_vertex(start_key)
    start.parent = -1
    goals = []
    for key in goal_keys:
        goals.append(copy_graph.get_vertex(key))


    frontier = PriorityQueue()
    frontier.put(start, 0)
    frontier_tracker = {}
    frontier_tracker[start] = 0

    start.dist = 0
    cost_so_far = {}
    cost_so_far[start] = 0
    found_goal = False
    while not frontier.empty():
        current = frontier.get()

        for goal in goals:
            if current == goal:
                closest_goal = goal
                found_goal = True
                
        if found_goal == True:
            break

        neighbors = current.get_neighbors()
        for key, info in neighbors.items():
            neighbor = copy_graph.get_vertex(key)
            neighbor_distance = info[1]
            cost = current.dist + neighbor_distance #assume uniform cost across all edges

            #found a better path to neighbor
            if cost < neighbor.dist and neighbor in frontier_tracker:
                frontier_tracker.pop(neighbor)

            if cost < neighbor.dist and neighbor in cost_so_far:
                cost_so_far.pop(neighbor)

            if neighbor not in frontier_tracker and neighbor not in cost_so_far:
                neighbor.dist = cost
                frontier_tracker[neighbor] = cost

                highest_priority = sys.maxsize #highest_priority = goal with lowest estimated cost
                for goal in goals:
                    priority = cost + heuristic(goal.get_xy(copy_graph.world_size), neighbor.get_xy(copy_graph.world_size))
                    if priority < highest_priority:
                        highest_priority = priority

                frontier.put(neighbor, highest_priority)
                neighbor.parent = current.key

    # reverse and convert to list
    list_path = list(reversed(trace(closest_goal, copy_graph)))
    path = Path(vertex_keys=[], distance=0, value=0)
    for xy in list_path:
        path.add_vertex(copy_graph.get_key(xy), new_distance=1, new_value=0)

    return path

# finds all the possible paths from a start (key) position to given goals (keys)
# returns a list of path objects
def find_paths(graph, start_key, goal_keys, value_limit):
    # initialize paths and start vertex
    paths = []
    cur_path = Path([start_key])
    start_vertex = graph.get_vertex(start_key)

    # start recursion to build out solution path list
    recurse_path_finding(graph, start_vertex, goal_keys, value_limit, cur_path, paths)

    return paths

def recurse_path_finding(graph, cur_vertex, goal_keys, value_limit, cur_path, paths):
    # pull out neighbors to iterate through
    cur_neighbors = cur_vertex.get_neighbors()

    # iterate through neighbors recursively diving deeper into the paths
    for key, info in cur_neighbors.items():
        # make sure not to recurse to previous states
        if key not in cur_path.vertex_keys:
            # make a copy of the path with the new vertex
            new_vertex_list = []
            for temp_key in cur_path.vertex_keys:
                new_vertex_list.append(temp_key)

            new_path = Path(new_vertex_list, cur_path.distance, cur_path.value)
            new_path.add_vertex(key, new_distance=info[1], new_value=info[2])

            # found solution
            if key in goal_keys:
                # if the total value is greater than limit, add to paths
                if new_path.total > value_limit:
                    paths.append(new_path)
            else:
                # pull out new vertex and pass it for the recursion
                new_vertex = graph.get_vertex(key)
                recurse_path_finding(graph, new_vertex, goal_keys, value_limit, new_path, paths)
