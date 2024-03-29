#Author: Nicholas Massad
#Date: 28/02/2023

import pygame
import time

class GameEngine:
    def __init__(self, width=600, height=600, grid_size=30, benchmark=False):
        self.width = width
        self.height = height
        self.grid_size = grid_size
        if not benchmark:
            pygame.init()
            self.window = pygame.display.set_mode((width, height))
            pygame.display.set_caption("Pathfinding Visualizer")
        else:
            self.window = None
        self.obstacles = {}

    # Draw a grid on the pygame window
    def draw_grid(self):
        if self.window is not None:
            for i in range(0, self.width, self.grid_size):
                pygame.draw.line(self.window, (255, 255, 255), (i, 0), (i, self.height))
            for i in range(0, self.height, self.grid_size):
                pygame.draw.line(self.window, (255, 255, 255), (0, i), (self.width, i))
        pass

    def create_boundary(self):
        for i in range(0, self.width, self.grid_size):
            self.add_obstacle(i, 0)
            self.add_obstacle(i, self.height - self.grid_size)
        for i in range(0, self.height, self.grid_size):
            self.add_obstacle(0, i)
            self.add_obstacle(self.width - self.grid_size, i)

    # Add an obstacle to the pygame window
    def add_obstacle(self, x, y):
        x = x - (x % self.grid_size)
        y = y - (y % self.grid_size)
        if self.window is not None:
            obstacle = pygame.Rect(x, y, self.grid_size, self.grid_size)
            pygame.draw.rect(self.window, (255, 0, 0), obstacle)
        else:
            obstacle = Rect(x, y)
        if (x, y) not in self.obstacles:
            self.obstacles[(x, y)] = obstacle

    # add a start point to the pygame window
    def add_start_point(self, x, y):
        x = x - (x % self.grid_size)
        y = y - (y % self.grid_size)
        if self.window is not None:
            pygame.draw.rect(self.window, (0, 255, 0), pygame.Rect(x, y, self.grid_size, self.grid_size))
        return x, y

    # add an end point to the pygame window
    def add_end_point(self, x, y):
        x = x - (x % self.grid_size)
        y = y - (y % self.grid_size)
        if self.window is not None:
            pygame.draw.rect(self.window, (0, 0, 255), pygame.Rect(x, y, self.grid_size, self.grid_size))
        return x, y

    # add a path to the pygame window
    def add_path_centered(self, x, y, x2, y2, rgb=(0, 255, 0)):
        x = x - (x % self.grid_size) + (self.grid_size / 2)
        y = y - (y % self.grid_size) + (self.grid_size / 2)
        x2 = x2 - (x2 % self.grid_size) + (self.grid_size / 2)
        y2 = y2 - (y2 % self.grid_size) + (self.grid_size / 2)
        if self.window is not None:
            pygame.draw.line(self.window, rgb, (x, y), (x2, y2), 2)
            pygame.display.update()
            time.sleep(0.01)

    def add_path(self, x, y, x2, y2, rgb=(0, 255, 0)):
        if self.window is not None:
            pygame.draw.line(self.window, rgb, (x, y), (x2, y2), 2)
            pygame.display.update()
            time.sleep(0.01)
        pass

    def remove_obstacle(self, x, y):
        if x < self.grid_size or x > self.width - self.grid_size:
            return
        if y < self.grid_size or y > self.height - self.grid_size:
            return
        x = x - (x % self.grid_size)
        y = y - (y % self.grid_size)
        if self.window is not None:
            pygame.draw.rect(self.window, (0, 0, 0), pygame.Rect(x, y, self.grid_size, self.grid_size))
        if (x, y) in self.obstacles:
            del self.obstacles[(x, y)]

    def clear(self, keep_obstacles=False):
        if self.window is not None:
            self.window.fill((0, 0, 0))
            self.draw_grid()
        if not keep_obstacles:
            self.obstacles = {}
            self.create_boundary()
        else:
            for obstacle in self.obstacles.values():
                if self.window is not None:
                    pygame.draw.rect(self.window, (255, 0, 0), obstacle)
                pass

    def draw_search_path(self, path, center=False):
        if center:
            for i in range(len(path)-1):
                node_a = path[i]
                node_b = path[i+1]
                self.add_path_centered(node_a[0], node_a[1], node_b[0], node_b[1])
        else:
            for i in range(len(path)-1):
                node_a = path[i]
                node_b = path[i+1]
                self.add_path(node_a[0], node_a[1], node_b[0], node_b[1])
        if self.window is not None:
            pygame.display.update()

    def draw_ellipse(self, x, y, a, b, rgb=(255, 255, 0)):
        if self.window is not None:
            pygame.draw.ellipse(self.window, rgb, pygame.Rect(x-a/2, y-b/2, a, b), 2)
            pygame.display.update()
            time.sleep(0.01)
        pass

    def draw_rectangle(self, x, y):
        if self.window is not None:
            pygame.draw.rect(self.window, (255, 255, 0), pygame.Rect(x, y, 10, 10))
            pygame.display.update()
            time.sleep(0.01)
        pass

    def draw_small_rectangle(self, x, y):
        if self.window is not None:
            pygame.draw.rect(self.window, (255, 255, 0), pygame.Rect(x, y, 5, 5))
            pygame.display.update()
        pass

    def draw_ellipse_angled(self, rect, angle, color=(255, 255, 0), width=2):
        if self.window is not None:
            surface = pygame.display.set_mode((rect[2], rect[3]))
            target_rect = pygame.Rect(rect)
            shape_surf = pygame.Surface(target_rect.size, pygame.SRCALPHA)
            pygame.draw.ellipse(shape_surf, color, (0, 0, *target_rect.size), width)
            rotated_surf = pygame.transform.rotate(shape_surf, angle)
            surface.blit(rotated_surf, rotated_surf.get_rect(center=target_rect.center))
        pass


class Rect:
    def __init__(self, x, y):
        self.x = x
        self.y = y

