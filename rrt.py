# Author: Nicholas Massad
# Date: 28/02/2023

from a_star import AStar2D
from node import Node
import random
import math

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


class RRTStar2D(RRT2D):
    def __init__(self, environement, game_engine=None, K=1000, r=50):
        super().__init__(environement=environement, game_engine=game_engine, K=K)
        self.r = r

    def find_path(self, start_pos, end_pos, progress=False):
        path = []
        graph = []

        start_node = Node(None, start_pos)
        end_node = Node(None, end_pos)

        graph.append(start_node)

        for i in range(self.K):
            node = self.create_node()
            nearest_node = self.nearest_neighbour(graph, node)
            if nearest_node is not None:
                if self.collision_manager.collision_check(nearest_node.pos, node.pos):
                    new_node = self.steer(nearest_node, node)
                    if self.collision_manager.collision_check(nearest_node.pos, new_node.pos):
                        self.rewire(graph, new_node, nearest_node)
                        if self.game_engine is not None:
                            if progress:
                                self.game_engine.add_path(new_node.parent.pos[0], new_node.parent.pos[1]
                                                , new_node.pos[0], new_node.pos[1], (148, 0, 211))
                    if calulate_distance(new_node, end_node) < self.r and self.collision_manager.collision_check(new_node.pos, end_node.pos):
                        end_node.parent = new_node
                        graph.append(end_node)
                        if self.game_engine is not None:
                            if progress:
                                self.game_engine.add_path(new_node.pos[0], new_node.pos[1]
                                                , end_node.pos[0], end_node.pos[1], (148, 0, 211))
                        return self.reconstruct_path(end_node)

        return None

    def steer(self, nearest_node, node):
        new_node = Node(None, node.pos)
        if calulate_distance(nearest_node, node) > self.r:
            new_node.pos = (nearest_node.pos[0] + (self.r * (node.pos[0] - nearest_node.pos[0]) / calulate_distance(nearest_node, node)),
                            nearest_node.pos[1] + (self.r * (node.pos[1] - nearest_node.pos[1]) / calulate_distance(nearest_node, node)))
        return new_node

    def rewire(self, graph, new_node, nearest_node):
        near_nodes = self.get_near_node(graph, new_node)
        min_cost_node = nearest_node
        min_cost = self.get_cost(nearest_node) + calulate_distance(nearest_node, new_node)
        for near_node in near_nodes:
            if self.collision_manager.collision_check(near_node.pos, new_node.pos):
                cost = self.get_cost(near_node) + calulate_distance(near_node, new_node)
                if cost < min_cost and self.collision_manager.collision_check(near_node.pos, new_node.pos):
                    min_cost = cost
                    min_cost_node = near_node
        new_node.parent = min_cost_node
        new_node.cost = min_cost
        graph.append(new_node)

    def get_cost(self, node):
        return node.cost

    def get_near_node(self, graph, node):
        near_nodes = []
        for n in graph:
            if calulate_distance(n, node) < self.r:
                near_nodes.append(n)
        return near_nodes


# Description: InformedRRT* pathfinding algorithm
class InformedRRTStar2D(RRTStar2D):
    def __init__(self, environement, game_engine, search_area, K=1000, r=50, goal_sample_rate=5, min_distance_to_goal=90, path_resolution=1):
        super().__init__(environement=environement, game_engine=game_engine, K=K, r=r)
        self.search_area = search_area
        self.goal_sample_rate = goal_sample_rate
        self.min_distance_to_goal = min_distance_to_goal
        self.path_resolution = path_resolution

    def find_path(self, start_pos, end_pos, progress=False):
        graph = []
        path = []
        start_node = Node(None, start_pos)
        end_node = Node(None, end_pos)
        graph.append(start_node)

        for i in range(self.K):
            if random.randint(0, 100) > self.goal_sample_rate:
                node = self.create_node()
            else:
                node = end_node
            nearest_node = self.nearest_neighbour(graph, node)
            new_node = self.steer(nearest_node, node)
            if self.collision_manager.collision_check(nearest_node.pos, new_node.pos):
                near_nodes = self.get_near_node(graph, new_node)
                new_node = self.choose_parent(new_node, near_nodes)
                if new_node is not None:
                    self.rewire(graph, new_node, near_nodes)
                    graph.append(new_node)
                    if calulate_distance(new_node, end_node) < self.min_distance_to_goal:
                        end_node.parent = new_node
                        graph.append(end_node)
                        if self.game_engine is not None:
                            if progress:
                                self.game_engine.add_path(new_node.pos[0], new_node.pos[1]
                                                , end_node.pos[0], end_node.pos[1], (148, 0, 211))
                        return self.reconstruct_path(end_node)
        return None

    def generate_random_point(self, bounds=None, min_dist=None):
        x = random.randint(self.search_area[0][0], self.search_area[1])
        y = random.randint(self.search_area[2], self.search_area[3])
        return (x, y)

    def steer(self, from_node, to_node):
        extend_length = self.r
        new_node = Node(None, from_node.pos)
        d, theta = self.calulate_distance_and_angle(new_node, to_node)
        if d < extend_length:
            extend_length = d
        new_node.cost += extend_length
        new_node.pos = self.calulate_new_position(new_node, theta, extend_length)
        new_node.parent = from_node
        return new_node

    def calulate_distance_and_angle(self, from_node, to_node):
        dx = to_node.pos[0] - from_node.pos[0]
        dy = to_node.pos[1] - from_node.pos[1]
        d = math.sqrt(dx**2 + dy**2)
        theta = math.atan2(dy, dx)
        return d, theta
    
    def propagate_cost_to_leaves(self, parent_node, graph, expand_distance):
