# Author: Nicholas Massad
# Date: 28/02/2023

from a_star import AStar2D
from node import Node
import random

def calulate_distance(node1, node2):
    return ((node1.pos[0] - node2.pos[0]) ** 2 + (node1.pos[1] - node2.pos[1]) ** 2) ** 0.5


class RRT2D(AStar2D):
    def __init__(self, environement, game_engine=None, K=1000):
        super().__init__(environement=environement, game_engine=game_engine)
        self.K = K

    def find_path(self, start_pos, end_pos, progress=False):
        path = []
        graph = []

        start_node = Node(None, start_pos)
        end_node = Node(None, end_pos)

        graph.append(start_node)

        for i in range(self.K):
            near_end_node = self.nearest_neighbour(graph, end_node)
            if near_end_node is not None:
                end_node.parent = near_end_node
                graph.append(end_node)
                if self.game_engine is not None:
                    if progress:
                        self.game_engine.add_path(near_end_node.pos[0], near_end_node.pos[1]
                                        , end_node.pos[0], end_node.pos[1], (148, 0, 211))
                return self.reconstruct_path(end_node)
            node = self.create_node()
            nearest_node = self.nearest_neighbour(graph, node)
            if nearest_node is not None:
                node.parent = nearest_node
                graph.append(node)
                if self.game_engine is not None:
                    if progress:
                        self.game_engine.add_path(nearest_node.pos[0], nearest_node.pos[1]
                                        , node.pos[0], node.pos[1], (148, 0, 211))
        return None

    def generate_random_point(self, bound=None, dimension=2):
        if bound is None:
            bound = ((self.environement['grid_size'], self.environement['width']-self.environement['grid_size']),
                     (self.environement['grid_size'], self.environement['height']-self.environement['grid_size']))
        return [random.randint(bound[i][0], bound[i][1]) for i in range(dimension)]

    def create_node(self):
        node = None
        while node is None:
            point = self.generate_random_point()
            if self.collision_manager.collision_check(point):
                node = Node(position=point)
                return node

    def nearest_neighbour(self, graph, node):
        nearest_node = None
        for n in graph:
            if nearest_node is None and self.collision_manager.collision_check(n.pos, node.pos):
                nearest_node = n
            elif nearest_node is not None:
                if calulate_distance(n, node) < calulate_distance(nearest_node, node) and self.collision_manager.collision_check(n.pos, node.pos):
                    nearest_node = n
        return nearest_node
