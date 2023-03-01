#Author: Nicholas Massad
#Date: 28/02/2023

import a_star

class BitStar(a_star.AStar):
    def __init__(self, window_width, window_height, grid_size):
        super().__init__(window_width, window_height, grid_size)

    # TODO: Implement the BIT* algorithm
    def find_path(self, start_pos, end_pos):
        path = []
        return path