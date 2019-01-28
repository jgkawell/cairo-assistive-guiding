import sys
import numpy as np
import heapq

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
    def __init__(self, key, cell_type = None):
        self.key = key
        self.cell_type = cell_type
        self.adj_list = {}
        self.parent = -1
        self.dist = sys.maxsize #distance to start vertex 0
        self.visited = False

    def add_neighbor(self, neighbor, weight=1):
        self.adj_list[neighbor] = weight

    def get_neighbors(self):
        return self.adj_list.keys()

    def get_xy(self, mapsize):
        y = self.key % mapsize
        x = int((self.key - y) / mapsize)
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
    def __init__(self, mapsize, directed=False):
        self.vertices = {}
        self.num_walls = 0
        self.mapsize = mapsize

    def add_vertex(self, vertex):
        self.vertices[vertex.key] = vertex

    def get_vertex(self, key):
        return self.vertices[key]

    def get_vertices(self):
        return self.vertices.keys()

    def __iter__(self):
        return self.vertices.value()

    def add_edge(self, from_key, to_key): #assume bidirectional
        self.vertices[from_key].add_neighbor(self.vertices[to_key])
        self.vertices[to_key].add_neighbor(self.vertices[from_key])

def heuristic(goal_pos, vertex_pos): #manhattan distance - admissible
    (x1, y1) = goal_pos
    (x2, y2) = vertex_pos
    return abs(x1 - x2) + abs(y1 - y2)

def trace(vertex, graph):
    trace = []
    trace.append(vertex.key)
    curr = vertex
    while curr.parent != -1:
        trace.append(curr.parent)
        curr = graph.get_vertex(curr.parent)
    return trace

def a_star(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    frontier_tracker = {}
    frontier_tracker[start] = 0

    start.dist = 0
    explored = {}
    cost_so_far = {}
    cost_so_far[start] = 0
    while not frontier.empty():
        current = frontier.get()

        if current == goal: break

        neighbors = current.get_neighbors()
        for neighbor in neighbors:
            cost = current.dist + 1 #assume uniform cost across all edges
            if cost < neighbor.dist and neighbor in frontier_tracker: #found a better path to neighbor
                frontier_tracker.pop(neighbor)
            if cost < neighbor.dist and neighbor in cost_so_far:
                cost_so_far.pop(neighbor)
            if neighbor not in frontier_tracker and neighbor not in cost_so_far:
                neighbor.dist = cost
                frontier_tracker[neighbor] = cost
                priority = cost + heuristic(goal.get_xy(mapsize), neighbor.get_xy(mapsize))
                frontier.put(neighbor, priority)
                neighbor.parent = current.key
    return trace(goal, graph)
            
