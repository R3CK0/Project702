#Author: Nicholas Massad
#Date: 28/02/2023

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.child = None
        self.pos = position
        self.samples = []
        self.inspected = False

        self.g = 0
        self.h = 0
        self.f = 0
        self.cost = 0

    def __eq__(self, other):
        return self.pos == other.pos

    def __hash__(self):
        return hash(self.pos)

    def __lt__(self, other):
        return self.f < other.f

    def get_pos(self):
        return self.pos

    def save(self, samples):
        self.samples = samples
        self.inspected = True
