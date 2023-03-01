# Author: Nicholas Massad
# Date: 28/02/2023

from a_star import AStar
from node import Node
import random
import pygame
import time

def calulate_distance(node1, node2):
    return ((node1.pos[0] - node2.pos[0]) ** 2 + (node1.pos[1] - node2.pos[1]) ** 2) ** 0.5


class RRT(AStar):
    def __init__(self, window_width, window_height, grid_size, obstacles, window, K):
        super().__init__(window_width, window_height, grid_size, obstacles, window)
        self.K = K

    # TODO: Implement the RRT algorithm
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
                if progress:
                    pygame.draw.line(self.window, (148, 0, 211), (near_end_node.pos[0] + self.grid_size / 2, near_end_node.pos[1] + self.grid_size / 2)
                                    , (end_node.pos[0] + self.grid_size / 2, end_node.pos[1] + self.grid_size / 2), 2)
                    pygame.display.update()
                    time.sleep(0.1)
                return self.reconstruct_path(end_node)
            node = self.create_node()
            nearest_node = self.nearest_neighbour(graph, node)
            if nearest_node is not None:
                node.parent = nearest_node
                graph.append(node)
                if progress:
                    pygame.draw.line(self.window, (148, 0, 211), (nearest_node.pos[0] + self.grid_size / 2, nearest_node.pos[1] + self.grid_size / 2)
                                    , (node.pos[0] + self.grid_size / 2, node.pos[1] + self.grid_size / 2), 2)
                    pygame.display.update()
                    time.sleep(0.1)
        return None

    def generate_random_point(self):
        x = random.randint(self.grid_size, self.width-self.grid_size)
        y = random.randint(self.grid_size, self.height-self.grid_size)
        return (x, y)

    def create_node(self):
        node = None
        while node is None:
            point = self.generate_random_point()
            if self.collision_check(point):
                node = Node(position=point)
                return node

    def nearest_neighbour(self, graph, node):
        nearest_node = None
        for n in graph:
            if nearest_node is None and self.collision_check(n.pos, node.pos):
                nearest_node = n
            elif nearest_node is not None:
                if calulate_distance(n, node) < calulate_distance(nearest_node, node) and self.collision_check(n.pos, node.pos):
                    nearest_node = n
        return nearest_node

    def collision_check(self, point, point2=None):
        if point2 is None:
            if point[0] < self.grid_size or point[0] > self.width-self.grid_size:
                return False
            if point[1] < self.grid_size or point[1] > self.height-self.grid_size:
                return False
            if not self.isValid(point):
                return False
        else:
            line = (point[0], point[1], point2[0], point2[1])
            for obstacle in self.obstacles.values():
                if obstacle.x != 0 and obstacle.y != 0 and obstacle.x != self.width-self.grid_size and obstacle.y != self.height-self.grid_size:
                    if self.obstacle_collision(line, obstacle):
                        return False

        return True # passes all checks

    def obstacle_collision(self, line, obstacle):
        object_vertices = [(obstacle.x, obstacle.y, obstacle.x + obstacle.width, obstacle.y),
                           (obstacle.x, obstacle.y, obstacle.x, obstacle.y + obstacle.height),
                           (obstacle.x + obstacle.width, obstacle.y + obstacle.height, obstacle.x + obstacle.width, obstacle.y),
                           (obstacle.x + obstacle.width, obstacle.y + obstacle.height, obstacle.x, obstacle.y + obstacle.height)]
        for vertex in object_vertices:
            if self.lineLine_collision(line, vertex):
                return True
        return False

    def lineLine_collision(self, line1, line2):
        x1, y1, x2, y2 = line1
        x3, y3, x4, y4 = line2
        den = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
        if den == 0:
            return False
        else:
            t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4))/den
            u = -((x1-x2)*(y1-y3) - (y1-y2)*(x1-x3))/den
            if 0 <= t <= 1 and 1 >= u >= 0:
                return True
            else:
                return False
