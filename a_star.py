#Author: Nicholas Massad
#Date: 28/02/2023

from queue import PriorityQueue
from node import Node
from collisionManager import CollisionManager2D


# Description: A* pathfinding algorithm
class AStar2D:
    def __init__(self, environement, game_engine=None, benchmark=False):
        self.environement = environement
        if not benchmark:
            self.game_engine = game_engine
        else:
            self.game_engine = None
        self.collision_manager = CollisionManager2D(game_engine.obstacles, self.environement)

    # Find a path from start to end
    def find_path(self, start_pos, end_pos, progress=False):
        # Create the open and closed sets
        open_set = PriorityQueue()
        closed_set = set()

        # Create the start and end nodes
        start_node = Node(None, start_pos)
        end_node = Node(None, end_pos)

        # Add the start node to the open set
        open_set.put((0, start_node))

        # Loop until the open set is empty
        while not open_set.empty():
            # Get the current node
            current_node = open_set.get()[1]
            closed_set.add(current_node)

            # If the current node is the end node, return the path
            if current_node == end_node:
                return self.reconstruct_path(current_node)

            # Get the neighbours of the current node
            neighbours = self.get_neighbours(current_node.pos)

            # Loop through the neighbours
            for neighbour in neighbours:
                # Create a neighbour node
                neighbour_node = Node(current_node, neighbour)
                if self.game_engine is not None:
                    if progress:
                        self.game_engine.add_path_centered(current_node.pos[0], current_node.pos[1], neighbour_node.pos[0], neighbour_node.pos[1], rgb=(148, 0, 211))

                # If the neighbour is in the closed set, continue
                if neighbour_node in closed_set:
                    continue

                # Calculate the cost of the neighbour
                neighbour_node.g = current_node.g + 1
                neighbour_node.h = self.heuristic(neighbour_node.pos, end_node.pos)
                neighbour_node.f = neighbour_node.g + neighbour_node.h

                # If the neighbour is not in the open set, add it
                if open_set.empty():
                    open_set.put((neighbour_node.f, neighbour_node))
                elif not self.isInQueue(neighbour_node, open_set):
                    open_set.put((neighbour_node.f, neighbour_node))
                else:
                    # If the neighbour is already in the open set, check if the current path is better
                    for item in open_set.queue:
                        if item[1] == neighbour_node:
                            if item[1].f > neighbour_node.f:
                                item = (item[0], neighbour_node)
                                break

        # Return an empty path if no path was found
        return None

    def isInQueue(self, node, open_set):
        for item in open_set.queue:
            if item[1] == node:
                return True
        return False

    def reconstruct_path(self, current_node):
        path = []
        while current_node is not None:
            path.append(current_node.pos)
            current_node = current_node.parent
        path.reverse()
        return path

    # Calculate the heuristic
    def heuristic(self, a, b):
        x1, y1 = a
        x2, y2 = b
        return abs(x1 - x2) + abs(y1 - y2)

    # Get the neighbours of a position
    def get_neighbours(self, pos):
        x, y = pos
        neighbours = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                if i == 0 and j == 0:
                    continue
                if self.collision_manager.collision_check((x + i*self.environement['grid_size'], y + j*self.environement['grid_size'])):
                    neighbours.append((x + i*self.environement['grid_size'], y + j*self.environement['grid_size']))
        return neighbours