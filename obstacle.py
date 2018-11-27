import math

class Obstacle:
    def __init__(self, x, y, theta, velocity):
        self.x, self.y, self.theta, self.velocity = x, y, theta, velocity

    def time_to_point(self, x, y):
        # TODO: incorporate current velocity/direction
        return math.sqrt((self.x - x)**2 + (self.y - y)**2) / self.velocity

    def cost_of_point_after_time(self, x, y, time):
        x = self.time_to_point(x, y)
        return 1 - 1 / (1 + math.exp(-x))