import math

VELOCITY_CAP = 800
ACCELERATION_CONSTANT = 600


class Robot:
    def __init__(self, x, y, theta, velocity):
        self.velocity_cap = VELOCITY_CAP
        self.x = x
        self.y = y
        self.theta = theta
        self.velocity = velocity
        self.acceleration = ACCELERATION_CONSTANT
        self.horizontal_velocity = velocity * math.cos(theta)
        self.vertical_velocity = velocity * math.sin(theta)
        self.horizontal_acceleration = self.acceleration * math.cos(theta)
        self.vertical_acceleration = self.acceleration * math.sin(theta)

