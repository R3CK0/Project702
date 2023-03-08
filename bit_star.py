#Author: Nicholas Massad
#Date: 28/02/2023

from rrt import InformedRRTStar2D
from node import Node
from queue import PriorityQueue
from collisionManager import CollisionManager2D
import numpy as np
from fmt import FMTStar2D
import time
import math
import random

def calculate_distance(node1, node2):
    return ((node1.pos[0] - node2.pos[0]) ** 2 + (node1.pos[1] - node2.pos[1]) ** 2) ** 0.5

class BitStar(InformedRRTStar2D, FMTStar2D):
    def __init__(self, environement, game_engine, radius_multiplier=2.3, dimension=2, free_space_volume=None, K = 200):
        super().__init__(environement, game_engine)
        self.K = K
        self.radius_multiplier = radius_multiplier
        self.game_engine = game_engine
        self.environement = environement
        self.dimension = dimension
        self.collision_manager = CollisionManager2D(self.game_engine.obstacles, self.environement)
        if free_space_volume is None:
            self.free_space_volume = (environement['width'] - 2 * environement['grid_size']) * (
                        environement['height'] - 2 * environement['grid_size']) - len(self.game_engine.obstacles) * \
                                     environement['grid_size'] ** 2
        else:
            self.free_space_volume = free_space_volume
        self.ellipsoid_params = None
        self.start_node = None
        self.end_node = None
        self.c_best = None

    # TODO: fix the bug with the block in front of the start
    def find_path(self, start_pos, end_pos, progress=False, optimise_time=None):
        unvisited_set = self.sampleFree(self.K)
        open_set = PriorityQueue()
        closed_set = set()

        self.start_node = Node(None, start_pos)
        self.end_node = Node(None, end_pos)
        self.c_best = calculate_distance(self.start_node, self.end_node)+10
        self.ellipsoid_params = self.calculate_elipsoid_params_homebrew(self.start_node, self.end_node, self.c_best)

        open_set.put((0, self.start_node))
        unvisited_set[self.end_node.pos] = self.end_node
        radius = self.calculate_radius(self.K)
        unvisited_set = self.assureGoalState(unvisited_set, self.end_node, radius)

        current_node = self.start_node
        start = True
        Found_path = False
        while current_node is not self.end_node:
            self.expand_tree(current_node, open_set, unvisited_set, radius, progress)
            closed_set.add(current_node)
            if start:
                remove_start = open_set.get()[1]
                start = False
            if open_set.empty() and not Found_path:
                self.moveClosedSetToOpenSet(open_set, closed_set)
                self.c_best += 10
                self.ellipsoid_params = self.calculate_elipsoid_params_homebrew(self.start_node, self.end_node, self.c_best)
            current_node = open_set.get()[1]
            if current_node == self.end_node:
                Found_path = True
                if optimise_time is not None:
                    return self.optimise_path(current_node, time.time(), optimise_time, radius, progress, graph=closed_set)
                return self.reconstruct_path(self.end_node)


    def create_node(self, elipsoid_params=None):
        node = None
        if elipsoid_params is not None:
            while node is None:
                center = elipsoid_params['center']
                a = elipsoid_params['c_best']/2
                b = elipsoid_params['b']/2
                d = random.random()
                theta = random.random()*2*math.pi
                r = (a * b)/math.sqrt((a * math.sin(theta))**2 + (b * math.cos(theta))**2)
                pos = (r*d*math.cos(theta)+center[0], r*d*math.sin(theta)+center[1])
                pos = self.rotate_vector((pos[0]-center[0], pos[1]-center[1]), elipsoid_params['angle'])
                pos = (pos[0]+center[0], pos[1]+center[1])
                if self.environement['width'] > pos[0] > self.environement['grid_size'] and self.environement['height'] > pos[1] > self.environement['grid_size']:
                    if self.collision_manager.collision_check(pos):
                        node = Node(position=tuple(pos))
                        return node
        while node is None:
            point = self.generate_random_point()
            if self.collision_manager.collision_check(point):
                node = Node(position=tuple(point))
                return node

    def sampleFree(self, n, ellipsoid_params=None):
        sampleSpace = {}
        for i in range(n):
            node = self.create_node(ellipsoid_params)
            sampleSpace[node.pos] = node
        return sampleSpace

    def moveClosedSetToOpenSet(self, open_set, closed_set):
        for node in closed_set:
            open_set.put((node.cost, node))
        closed_set.clear()

    def near(self, node, samples, radius):
        if self.ellipsoid_params is not None:
            in_range = [n for n in samples if calculate_distance(n, node) < radius and n != node]
            return [n for n in in_range if self.inEllipse(n, self.ellipsoid_params)]
        else:
            return [n for n in samples if calculate_distance(n, node) < radius and n != node]

    def expand_tree(self, x, open, unvisited, radius, progress, start_time=None):
        nearX = self.near(x, unvisited.values(), radius)
        if len(nearX) > 0:
            for y in nearX:
                if y is not x.parent:
                    nearY = self.near(y, list(dict(open.queue).values()), radius)
                    if len(nearY) > 0:
                        ymin = np.argmin(calculate_distance(y, z) + z.cost for z in nearY)
                        if self.collision_manager.collision_check(nearY[ymin].pos, y.pos):
                            y.parent = nearY[ymin]
                            y.cost = nearY[ymin].cost + calculate_distance(y, nearY[ymin])
                            open.put((y.cost, y))
                            del unvisited[y.pos]
                            if progress:
                                if start_time is not None:
                                    temp_start_time = time.time()
                                    self.game_engine.add_path(y.pos[0], y.pos[1], y.parent.pos[0], y.parent.pos[1], rgb=(148, 0, 211))
                                    temp_stop_time = time.time()
                                    start_time += temp_stop_time - temp_start_time
                                else:
                                    self.game_engine.add_path(y.pos[0], y.pos[1], y.parent.pos[0], y.parent.pos[1], rgb=(148, 0, 211))
    def optimise_path(self, current_node, time_start, time_optimise, radius=None, progress=False, graph=None):
        path_list = PriorityQueue()
        open_set = PriorityQueue()
        closed_set = graph
        unvisited_set = {}
        c_best_init = current_node.cost # current node is the end node
        path = self.reconstruct_path(current_node)

        self.c_best = self.end_node.cost
        path_list.put((self.c_best, path))


        while time.time() - time_start < time_optimise:
            self.ellipsoid_params = self.calculate_elipsoid_params_homebrew(self.start_node, self.end_node, self.c_best)
            unvisited_set = self.sampleFree(self.K, self.ellipsoid_params)
            self.prune(open_set, closed_set, unvisited_set, self.ellipsoid_params)
            unvisited_set[self.end_node.pos] = self.end_node
            radius = self.calculate_radius(self.K)
            unvisited_set = self.assureGoalState(unvisited_set, self.end_node, radius)
            current_node = self.start_node
            start = True
            while current_node is not self.end_node:
                self.expand_tree(current_node, open_set, unvisited_set, radius, progress, time_start)
                closed_set.add(current_node)
                if open_set.empty():
                    break
                if start:
                    remove_start = open_set.get()[1]
                    start = False
                current_node = open_set.get()[1]
                if current_node == self.end_node:
                    path = self.reconstruct_path(self.end_node)
                    if path is not None:
                        self.c_best = self.end_node.cost
                        path_list.put((self.c_best, path))

        (self.c_best, path) = path_list.get()
        print("Initial cost: ", c_best_init)
        print("Optimised cost: ", self.c_best)
        return path

    def prune(self, open_set, closed_set, unvisited_set, ellipsoid_params):
        for node in closed_set:
            if not self.inEllipse(node, ellipsoid_params):
                unvisited_set[node.pos] = node
        open_set.put((0, self.start_node))
        closed_set.clear()