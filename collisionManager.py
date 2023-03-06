

class CollisionManager2D:
    def __init__(self, obstacles, environment):
        self.obstacles = obstacles
        self.environment = environment

    def collision_check(self, point, point2=None):
        if point2 is None:
            if point[0] < 0 or point[0] > self.environment['width']:
                return False
            if point[1] < 0 or point[1] > self.environment['height']:
                return False
            if not self.isValid(point):
                return False
        else:
            line = (point[0], point[1], point2[0], point2[1])
            for obstacle in self.obstacles.values():
                if obstacle.x != 0 and obstacle.y != 0 and obstacle.x != self.environment['width']-self.environment['grid_size']\
                        and obstacle.y != self.environment['height']-self.environment['grid_size']:
                    if self.obstacle_collision(line, obstacle):
                        return False
        return True

    def obstacle_collision(self, line, obstacle):
        object_edges = [(obstacle.x, obstacle.y, obstacle.x + self.environment['grid_size'], obstacle.y),
                           (obstacle.x, obstacle.y, obstacle.x, obstacle.y + self.environment['grid_size']),
                           (obstacle.x + self.environment['grid_size'], obstacle.y, obstacle.x + self.environment['grid_size'], obstacle.y + self.environment['grid_size']),
                           (obstacle.x, obstacle.y + self.environment['grid_size'], obstacle.x + self.environment['grid_size'], obstacle.y + self.environment['grid_size'])]

        for edge in object_edges:
            if self.LineCollisionCheck(line, edge):
                return True
        return False

    def isValid(self, point):
        x = point[0] - (point[0] % self.environment['grid_size'])
        y = point[1] - (point[1] % self.environment['grid_size'])
        if (x, y) in self.obstacles:
            return False
        return True

    def LineCollisionCheck(self, line1, line2):
        x1, y1, x2, y2 = line1
        x3, y3, x4, y4 = line2
        den = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
        if den == 0:
            return False
        else:
            t = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / den
            u = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / den
            if 0 <= t <= 1 and 1 >= u >= 0:
                return True
            else:
                return False
