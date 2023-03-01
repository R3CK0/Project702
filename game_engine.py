#Author: Nicholas Massad
#Date: 28/02/2023

import pygame

class GameEngine:
    def __init__(self, width=600, height=600, grid_size=30):
        pygame.init()
        self.width = width
        self.height = height
        self.grid_size = grid_size
        self.window = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Pathfinding Visualizer")
        self.obstacles = {}

    # Draw a grid on the pygame window
    def draw_grid(self):
        for i in range(0, self.width, self.grid_size):
            pygame.draw.line(self.window, (255, 255, 255), (i, 0), (i, self.height))
        for i in range(0, self.height, self.grid_size):
            pygame.draw.line(self.window, (255, 255, 255), (0, i), (self.width, i))

    # Add an obstacle to the pygame window
    def add_obstacle(self, x, y):
        x = x - (x % self.grid_size)
        y = y - (y % self.grid_size)
        pygame.draw.rect(self.window, (255, 0, 0), pygame.Rect(x, y, self.grid_size, self.grid_size))
        obstacle = Obstacle(x, y, self.grid_size)
        if (x, y) not in self.obstacles:
            self.obstacles[(x, y)] = obstacle

    # add a start point to the pygame window
    def add_start_point(self, x, y):
        x = x - (x % self.grid_size)
        y = y - (y % self.grid_size)
        pygame.draw.rect(self.window, (0, 255, 0), pygame.Rect(x, y, self.grid_size, self.grid_size))
        return x, y

    # add an end point to the pygame window
    def add_end_point(self, x, y):
        x = x - (x % self.grid_size)
        y = y - (y % self.grid_size)
        pygame.draw.rect(self.window, (0, 0, 255), pygame.Rect(x, y, self.grid_size, self.grid_size))
        return x, y

    # add a path to the pygame window
    def add_path(self, x, y, x2, y2):
        x = x - (x % self.grid_size) + (self.grid_size / 2)
        y = y - (y % self.grid_size) + (self.grid_size / 2)
        x2 = x2 - (x2 % self.grid_size) + (self.grid_size / 2)
        y2 = y2 - (y2 % self.grid_size) + (self.grid_size / 2)
        pygame.draw.line(self.window, (0, 255, 0), (x, y), (x2, y2), 2)

    def add_path2(self, x, y, x2, y2):
        pygame.draw.line(self.window, (255, 0, 0), (x, y), (x2, y2), 2)

    def remove_obstacle(self, x, y):
        x = x - (x % self.grid_size)
        y = y - (y % self.grid_size)
        pygame.draw.rect(self.window, (0, 0, 0), pygame.Rect(x, y, self.grid_size, self.grid_size))
        if (x, y) in self.obstacles:
            del self.obstacles[(x, y)]

    def clear(self):
        self.window.fill((0, 0, 0))
        self.obstacles = {}
        self.draw_grid()

    def draw_search_path(self, path):
        for i in range(len(path)-1):
            node_a = path[i]
            node_b = path[i+1]
            self.add_path(node_a[0], node_a[1], node_b[0], node_b[1])


class Obstacle:
    def __init__(self, x, y, grid_size):
        self.x = x
        self.y = y
        self.width = grid_size
        self.height = grid_size

    def collidepoint(self, point):
        if self.x <= point[0] <= self.x + self.width:
            if self.y <= point[1] <= self.y + self.height:
                return True
        return False