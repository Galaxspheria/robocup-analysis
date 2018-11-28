import math


class Robot:
    def __init__(self, x, y, theta, velocity, acceleration):
        self.x = x
        self.y = y
        self.theta = theta
        self.velocity = velocity
        self.acceleration = acceleration
        self.horizontal_velocity = velocity * math.cos(theta)
        self.vertical_velocity = velocity * math.sin(theta)
        self.horizontal_acceleration = acceleration * math.cos(theta)
        self.vertical_acceleration = acceleration * math.sin(theta)

