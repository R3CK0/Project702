#Author: Nicholas Massad
#Date: 28/02/2023

from a_star import AStar
from queue import PriorityQueue
from game_engine import Obstacle
import random

class RRT(AStar):
    def __init__(self, window_width, window_height, grid_size, obstacles):
        super().__init__(window_width, window_height, grid_size)
        self.obstacles = obstacles

    # TODO: Implement the RRT algorithm
    def find_path(self, start_pos, end_pos):
        path = []
        return path

    def generate_random_point(self):
        x = random.randint(0, self.width)
        y = random.randint(0, self.height)
        return (x, y)

    def collision_check(self, point):
        if point[0] < 0 or point[0] > self.width:
            return False
        if point[1] < 0 or point[1] > self.height:
            return False
        if not self.isValid(point):
            return False
        return True

    def isValid(self, point):
        for obstacle in self.obstacles:
            if obstacle.collidepoint(point):
                return False
        return True