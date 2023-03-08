# Author: Nicholas Massad
# Date: 28/02/2023

from a_star import AStar2D
from node import Node
import random
import math
import numpy as np
import time
from queue import PriorityQueue


def calculate_distance(node1, node2):
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
            bound = ((self.environement['grid_size'], self.environement['width'] - self.environement['grid_size']),
                     (self.environement['grid_size'], self.environement['height'] - self.environement['grid_size']))
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
                if calculate_distance(n, node) < calculate_distance(nearest_node, node) and self.collision_manager.collision_check(
                        n.pos, node.pos):
                    nearest_node = n
        return nearest_node


class RRTStar2D(RRT2D):
    def __init__(self, environement, game_engine, K=1000, r=50):
        super().__init__(environement=environement, game_engine=game_engine, K=K)
        self.r = r

    def find_path(self, start_pos, end_pos, progress=False, optimise_time=None):
        start = 0
        if optimise_time is not None:
            start = time.time()
        graph = []

        start_node = Node(None, start_pos)
        end_node = Node(None, end_pos)

        graph.append(start_node)

        for i in range(self.K):
            node = self.create_node()
            nearest_node = self.nearest_neighbour(graph, node)
            if nearest_node is not None:
                new_node = self.steer(nearest_node, node)
                if self.collision_manager.collision_check(nearest_node.pos, new_node.pos):
                    self.rewire(graph, new_node, nearest_node)
                    if self.game_engine is not None:
                        if progress:
                            self.game_engine.add_path(new_node.parent.pos[0], new_node.parent.pos[1]
                                                      , new_node.pos[0], new_node.pos[1], (148, 0, 211))
                if calculate_distance(new_node, end_node) < self.r and self.collision_manager.collision_check(
                        new_node.pos, end_node.pos):
                    end_node.parent = new_node
                    end_node.cost = new_node.cost + calculate_distance(new_node, end_node)
                    graph.append(end_node)
                    if self.game_engine is not None:
                        if progress:
                            self.game_engine.add_path(new_node.pos[0], new_node.pos[1]
                                                      , end_node.pos[0], end_node.pos[1], (148, 0, 211))
                    if optimise_time is not None:
                        path = self.optimise_path(end_node, start, optimise_time, progress, graph)
                        return self.reconstruct_path(path[-1])
                    else:
                        return self.reconstruct_path(end_node)
        return None

    def steer(self, nearest_node, node):
        new_node = Node(None, node.pos)
        if calculate_distance(nearest_node, node) > self.r:
            new_node.pos = (nearest_node.pos[0] + (
                    self.r * (node.pos[0] - nearest_node.pos[0]) / calculate_distance(nearest_node, node)),
                            nearest_node.pos[1] + (
                                    self.r * (node.pos[1] - nearest_node.pos[1]) / calculate_distance(nearest_node,
                                                                                                      node)))
        return new_node

    def rewire(self, graph, new_node, nearest_node):
        near_nodes = self.get_near_node(graph, new_node)
        min_cost_node = nearest_node
        min_cost = self.get_cost(nearest_node) + calculate_distance(nearest_node, new_node)
        for near_node in near_nodes:
            if self.collision_manager.collision_check(near_node.pos, new_node.pos):
                cost = self.get_cost(near_node) + calculate_distance(near_node, new_node)
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
            if calculate_distance(n, node) < self.r:
                near_nodes.append(n)
        return near_nodes

    def optimise_path(self, current_node, time_start, time_optimise, progress=False, graph=None):
        path = []
        while current_node is not None:
            path.append(current_node)
            if current_node.parent is not None:
                current_node.parent.set_child(current_node)
            current_node = current_node.parent
        path.reverse()
        start_node = path[0]
        end_node = path[-1]
        c_best_init = end_node.cost
        c_best = c_best_init
        while time.time() - time_start < time_optimise:
            for node in path:
                if node is not start_node and node is not end_node:
                    new_node = self.sample_near(node)
                    nearest_node = self.nearest_neighbour(path, new_node)
                    if nearest_node is not None and nearest_node.parent is not None and nearest_node.child is not None:
                        if self.collision_manager.collision_check(new_node.pos, nearest_node.parent.pos) \
                                and self.collision_manager.collision_check(new_node.pos, nearest_node.child.pos):
                            if nearest_node.child.cost > (nearest_node.parent.cost + calculate_distance(
                                    nearest_node.parent, new_node)
                                                          + calculate_distance(new_node, nearest_node.child)):
                                self.rewire_path(nearest_node, new_node, path)
                                if progress:
                                    temp_time_start = time.time()
                                    self.game_engine.add_path(nearest_node.parent.pos[0], nearest_node.parent.pos[1],
                                                              new_node.pos[0], new_node.pos[1], (148, 0, 211))
                                    self.game_engine.add_path(nearest_node.child.pos[0], nearest_node.child.pos[1],
                                                              new_node.pos[0], new_node.pos[1], (148, 0, 211))
                                    temp_time_stop = time.time()
                                    time_start += temp_time_stop - temp_time_start
                                self.propagate_cost_to_leaves(path[0])
        c_best = path[-1].cost
        print("Initial cost: ", c_best_init)
        print("Optimised cost: ", c_best)
        return path

    def sample_near(self, node):
        new_node = Node(None, node.pos)
        new_node.pos = (node.pos[0] + random.uniform(-self.r / 2, self.r / 2),
                        node.pos[1] + random.uniform(-self.r / 2, self.r / 2))
        return new_node

    def rewire_path(self, nearest_node, new_node, path):
        cost = nearest_node.parent.cost
        new_node.parent = nearest_node.parent
        new_node.child = nearest_node.child
        new_node.cost = cost + calculate_distance(nearest_node.parent, new_node)
        nearest_node.parent.set_child(new_node)
        nearest_node.child.set_parent(new_node)
        nearest_node.child.cost = new_node.cost + calculate_distance(new_node, nearest_node.child)
        path[path.index(nearest_node)] = new_node

    def propagate_cost_to_leaves(self, node):
        if node.child is not None:
            node.child.cost = node.cost + calculate_distance(node, node.child)
            self.propagate_cost_to_leaves(node.child)


# Description: InformedRRT* pathfinding algorithm
class InformedRRTStar2D(RRTStar2D):
    def __init__(self, environement, game_engine, K=1000, r=100, goal_sample_rate=5):
        super().__init__(environement=environement, game_engine=game_engine, K=K, r=r)
        self.goal_sample_rate = goal_sample_rate

    def optimise_path(self, current_node, time_start, time_optimise, progress=False, graph=None):
        graph = graph
        path_list = PriorityQueue()
        c_best_init = current_node.cost # current node is the end node
        path = self.build_path(current_node)
        start_node = path[0]
        end_node = path[-1]
        c_best = end_node.cost
        path_list.put((c_best, path))
        while time.time() - time_start < time_optimise:
            elipsoid_params = self.calculate_elipsoid_params_homebrew(start_node, end_node, c_best)
            for i in range(self.K):
                if time.time() - time_start > time_optimise:
                    break
                node = self.create_node(elipsoid_params)
                nearest_node = self.nearest_neighbour(graph, node, elipsoid_params)
                if nearest_node is not None:
                    new_node = self.steer(nearest_node, node)
                    if self.collision_manager.collision_check(nearest_node.pos, new_node.pos):
                        self.rewire(graph, new_node, nearest_node, elipsoid_params)
                        if self.game_engine is not None:
                            if progress:
                                temp_time_start = time.time()
                                self.game_engine.add_path(new_node.parent.pos[0], new_node.parent.pos[1]
                                                          , new_node.pos[0], new_node.pos[1], (148, 0, 211))
                                temp_time_stop = time.time()
                                time_start += temp_time_stop - temp_time_start
                    if calculate_distance(new_node, end_node) < self.r and self.collision_manager.collision_check(
                            new_node.pos, end_node.pos):
                        end_node.parent = new_node
                        end_node.cost = new_node.cost + calculate_distance(new_node, end_node)
                        if self.game_engine is not None:
                            if progress:
                                temp_time_start = time.time()
                                self.game_engine.add_path(new_node.pos[0], new_node.pos[1]
                                                          , end_node.pos[0], end_node.pos[1], (148, 0, 211))
                                temp_time_stop = time.time()
                                time_start += temp_time_stop - temp_time_start
                        path = self.build_path(end_node)
                        if path is not None:
                            c_best = end_node.cost
                            path_list.put((c_best, path))
                        break
        (c_best, path) = path_list.get()
        print("Initial cost: ", c_best_init)
        print("Optimised cost: ", c_best)
        return path

    def build_path(self, current_node):
        path = []
        count = 0
        while current_node is not None and count < 200:
            count += 1
            path.append(current_node)
            current_node = current_node.parent
        path.reverse()
        if count == 200:
            return None
        return path

    def nearest_neighbour(self, graph, node, elipsoid_params=None):
        nearest_node = None
        if elipsoid_params is not None:
            for n in graph:
                if self.inEllipse(n, elipsoid_params):
                    if nearest_node is None and self.collision_manager.collision_check(n.pos, node.pos):
                        nearest_node = n
                    elif nearest_node is not None:
                        if calculate_distance(n, node) < calculate_distance(nearest_node,
                                                                            node) and self.collision_manager.collision_check(
                                n.pos, node.pos):
                            nearest_node = n
            return nearest_node
        for n in graph:
            if nearest_node is None and self.collision_manager.collision_check(n.pos, node.pos):
                nearest_node = n
            elif nearest_node is not None:
                if calculate_distance(n, node) < calculate_distance(nearest_node, node) and self.collision_manager.collision_check(
                        n.pos, node.pos):
                    nearest_node = n
        return nearest_node

    def get_near_node(self, graph, node, elipsoid_params=None):
        near_nodes = []
        if elipsoid_params is not None:
            for n in graph:
                if self.inEllipse(n, elipsoid_params):
                    if calculate_distance(n, node) < self.r:
                        near_nodes.append(n)
            return near_nodes
        for n in graph:
            if calculate_distance(n, node) < self.r:
                near_nodes.append(n)
        return near_nodes

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
                        node = Node(position=pos)
                        return node
        while node is None:
            point = self.generate_random_point()
            if self.collision_manager.collision_check(point):
                node = Node(position=point)
                return node

    def rotationToWorldFrame(self, start_node, goal_node):
        a = []
        d = calculate_distance(start_node, goal_node)
        for i in range(len(start_node.pos)):
            a.append((goal_node.pos[i]-start_node.pos[i])/d)
        I = np.identity(len(start_node.pos))
        M = np.asarray(a)@I.T
        return M

    def sampleUnitBall(self):
        d = random.random()
        theta = random.random()*2*math.pi
        return (d*math.cos(theta), d*math.sin(theta))

    def calculate_elipsoid_params(self, start_node, goal_node, c_best):
        c_min = calculate_distance(start_node, goal_node)
        center = ((start_node.pos[0] + goal_node.pos[0]) / 2, (start_node.pos[1] + goal_node.pos[1]) / 2)
        C = self.rotationToWorldFrame(start_node, goal_node)
        r1 = c_best/2
        r2 = math.sqrt(c_best**2 - c_min**2)/2
        L = np.asarray([r1, r2])
        return {'C': C, 'L': L, 'center': center, 'c_best': c_best, 'c_min': c_min}

    def calculate_elipsoid_params_homebrew(self, start_node, goal_node, c_best):
        c_min = calculate_distance(start_node, goal_node)
        center = ((start_node.pos[0] + goal_node.pos[0]) / 2, (start_node.pos[1] + goal_node.pos[1]) / 2)
        b = math.sqrt(c_best**2 - c_min**2)
        angle = math.atan2(goal_node.pos[1]-start_node.pos[1], goal_node.pos[0]-start_node.pos[0])
        if start_node.pos[1]-goal_node.pos[1] == 0:
            self.game_engine.draw_rectangle(center[0], center[1])
            self.game_engine.draw_ellipse(center[0], center[1], c_best, b)
        if start_node.pos[0] - goal_node.pos[0] == 0:
            self.game_engine.draw_rectangle(center[0], center[1])
            self.game_engine.draw_ellipse(center[0], center[1], b, c_best)
        return {'center': center, 'c_best': c_best, 'c_min': c_min,  'b': b, 'angle': angle}

    def inEllipse(self, node, elipsoid_params):
        dx = node.pos[0] - elipsoid_params['center'][0]
        dy = node.pos[1] - elipsoid_params['center'][1]

        a = elipsoid_params['c_best']
        b = math.sqrt(elipsoid_params['c_best']**2-elipsoid_params['c_min']**2)
        dist = dx**2 / a**2 + dy**2 / b**2

        if dist <= 1:
            return True
        else:
            return False

    def rewire(self, graph, new_node, nearest_node, elipsoid_params=None):
        near_nodes = self.get_near_node(graph, new_node, elipsoid_params)
        min_cost_node = nearest_node
        min_cost = self.get_cost(nearest_node) + calculate_distance(nearest_node, new_node)
        for near_node in near_nodes:
            if self.collision_manager.collision_check(near_node.pos, new_node.pos):
                cost = self.get_cost(near_node) + calculate_distance(near_node, new_node)
                if cost < min_cost and self.collision_manager.collision_check(near_node.pos, new_node.pos):
                    min_cost = cost
                    min_cost_node = near_node
        new_node.parent = min_cost_node
        new_node.cost = min_cost
        graph.append(new_node)

    def rotate_vector(self, v, angles):
        """     Rotate an n-dimensional vector v by a list of angles for each dimension.     """
        # Create rotation matrix
        if type(angles) is float:
            angles = np.asarray(angles)
            angles = np.append(angles, 0)
        else:
            angles = np.asarray(angles)
        rotation_matrix = np.identity(len(v))
        for i in range(len(v)):
            for j in range(i+1, len(v)):
                rotation_matrix[i][i] = math.cos(angles[i])
                rotation_matrix[i][j] = -math.sin(angles[i])
                rotation_matrix[j][i] = math.sin(angles[i])
                rotation_matrix[j][j] = math.cos(angles[i])
                # Rotate vector
        rotated_vector = np.matmul(rotation_matrix, v)
        return rotated_vector