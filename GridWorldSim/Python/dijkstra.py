#!/usr/bin/env python3
import sys
import numpy as np

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

class Graph():
    def __init__(self, directed=False):
        self.vertices = {}
        self.num_walls = 0

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

def min_dist(graph):
    min_key = -1
    min_dist = sys.maxsize
    for i in range(mapsize**2):
        v = graph.get_vertex(i)
        if v.dist < min_dist and v.visited == False:
            min_dist = v.dist
            min_key = i
    return graph.get_vertex(min_key)

def dijkstra(graph):
    v_source = graph.get_vertex(0)
    v_source.dist = 0
    visited = 0
    while visited < mapsize**2 - graph.num_walls:
        visited += 1
        min_vertex = min_dist(graph)
        min_vertex.visited = True
        neighbors = min_vertex.get_neighbors()
        for neighbor in neighbors:
            if neighbor.visited == False:
                cur_dist = neighbor.dist
                if min_vertex.dist + 1 < cur_dist:
                    neighbor.parent = min_vertex.key
                    neighbor.dist = min_vertex.dist + 1
    trace = [99]
    vertex = graph.get_vertex(99)
    parent = vertex.parent
    while parent != -1:
        trace.append(parent)
        parent = graph.get_vertex(parent).parent
    return trace

if __name__ == "__main__":
    #test run
    mapsize = 10
    num_walls = 0
    G = Graph()
    for i in range(mapsize**2):
        v = Vertex(i)
        G.add_vertex(v)
    G.get_vertex(89).cell_type = "Wall"
    G.num_walls += 1
    for i in range(mapsize**2):
        v_cur = G.get_vertex(i)
        if v_cur.cell_type != "Wall":
            if i % mapsize != mapsize-1 and G.get_vertex(i+1).cell_type != "Wall": G.add_edge(i, i+1) #right
            if i % mapsize != 0 and G.get_vertex(i-1).cell_type != "Wall": G.add_edge(i, i-1) #left
            if i + mapsize < mapsize**2 and G.get_vertex(i+mapsize).cell_type != "Wall": G.add_edge(i, i+mapsize) #tile above
            if i - mapsize > 0 and G.get_vertex(i-mapsize).cell_type != "Wall": G.add_edge(i, i-mapsize) #tile below
    trace = dijkstra(G)
    print(trace, len(trace))
