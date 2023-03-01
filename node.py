#Author: Nicholas Massad
#Date: 28/02/2023

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.pos = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.pos == other.pos

    def __hash__(self):
        return hash(self.pos)

    def __lt__(self, other):
        return self.f < other.f

    def get_pos(self):
        return self.pos