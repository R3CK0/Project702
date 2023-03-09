# Author: Nicholas Massad
# Date: 03/03/2023

from node import Node
from collisionManager import CollisionManager2D
import math
from queue import PriorityQueue
import random
import numpy as np
import time

# Description: FMT* path planning algorithm
class FMTStar2D:
    def __init__(self, environement, game_engine=None, free_space_volume=None, radius_multiplier=1.1, dimension=2, benchmark=False):
        self.environement = environement
        if not benchmark:
            self.game_engine = game_engine
        else:
            self.game_engine = None
        self.collision_manager = CollisionManager2D(game_engine.obstacles, environement)
        self.radius_multiplier = radius_multiplier
        self.dimension = dimension
        if free_space_volume is None:
            self.free_space_volume = (environement['width']-2*environement['grid_size']) * (environement['height']-2*environement['grid_size']) - len(self.collision_manager.obstacles) * environement['grid_size']**2
        else:
            self.free_space_volume = free_space_volume

    def find_path(self, start_pos, end_pos, progress=False, K = 150):
        closed_set = set()
        unvisited_set = self.sampleFree(K)
        open_set = PriorityQueue()
        radius = self.calculate_radius(K)

        start_node = Node(None, start_pos)
        end_node = Node(None, end_pos)

        open_set.put((0, start_node))
        unvisited_set[end_node.pos] = end_node
        unvisited_set = self.assureGoalState(unvisited_set, end_node, radius)

        current_node = start_node
        start = True
        safety_time = time.time()
        stop_safety = 5
        while current_node is not end_node and time.time() - safety_time < stop_safety:
            self.expand_tree(current_node, open_set, unvisited_set, radius, progress)
            closed_set.add(current_node)
            if open_set.empty():
                return None
            if start:
                remove_start = open_set.get()[1]
                start = False
            current_node = open_set.get()[1]
            if current_node == end_node:
                return self.reconstruct_path(current_node)

    def reconstruct_path(self, current_node):
        path = []
        while current_node is not None:
            path.append(current_node.pos)
            current_node = current_node.parent
        path.reverse()
        return path

    def create_node(self, newNode=None):
        if newNode is not None:
            node = None
            while node is None:
                point = self.generate_random_point(bound=newNode)
                if self.collision_manager.collision_check(point):
                    node = Node(position=point)
            return node
        node = None
        while node is None:
            point = self.generate_random_point()
            if self.collision_manager.collision_check(point):
                node = Node(position=point)
        return node

    def generate_random_point(self, bound=None, dimension=2):
        if bound is None:
            bound = ((self.environement['grid_size'], self.environement['width']-self.environement['grid_size']),
                     (self.environement['grid_size'], self.environement['height']-self.environement['grid_size']))
        return tuple([random.randint(bound[i][0], bound[i][1]) for i in range(dimension)])

    def sampleFree(self, n):
        sampleSpace = {}
        for i in range(n):
            node = self.create_node()
            sampleSpace[node.pos] = node
        return sampleSpace

    def save(self, samples, node):
        node.save(samples)

    def near(self, node, samples, radius):
        if node.inspected:
            return node.samples
        else:
            return [n for n in samples if self.cost(n, node) < radius and n != node]

    def calculate_unit_ball_volume(self, dimension):
        if dimension == 0:
            return 1
        if dimension == 1:
            return 2
        return 2*math.pi/dimension*self.calculate_unit_ball_volume(dimension-2)

    def calculate_radius(self, samples: int):
        a = 1/self.dimension
        unitBallVolume = self.calculate_unit_ball_volume(self.dimension)
        return self.radius_multiplier * 2 * a**a * (self.free_space_volume / unitBallVolume)**a * (math.log(samples)/samples)**a

    def cost(self, node1, node2):
        return math.sqrt((node1.pos[0] - node2.pos[0])**2 + (node1.pos[1] - node2.pos[1])**2)

    def assureGoalState(self, samples, goal, radius):
        if not any([self.cost(n, goal) < radius for n in samples.values()]):
            bound = ((goal.pos[0]-radius, goal.pos[0]+radius), (goal.pos[1]-radius, goal.pos[1]+radius))
            for _ in range(2*self.dimension):
                node = self.create_node(bound)
                samples[node.pos] = node
        return samples

    def expand_tree(self, x, open, unvisited, radius, progress):
        nearX = self.near(x, unvisited.values(), radius)
        self.save(nearX, x)
        for y in nearX:
            if y is not x.parent:
                nearY = self.near(y, list(dict(open.queue).values()), radius)
                if len(nearY) > 0:
                    ymin = np.argmin(self.cost(y, z) + z.f for z in nearY)
                    if self.collision_manager.collision_check(nearY[ymin].pos, y.pos):
                        y.parent = nearY[ymin]
                        y.f = nearY[ymin].f + self.cost(y, nearY[ymin])
                        open.put((y.f, y))
                        del unvisited[y.pos]
                        if self.game_engine is not None:
                            if progress:
                                self.game_engine.add_path(y.pos[0], y.pos[1], y.parent.pos[0], y.parent.pos[1], rgb=(148, 0, 211))


