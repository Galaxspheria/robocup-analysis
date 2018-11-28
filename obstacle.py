import math

DELTA_DEGREES = math.pi/3

class Obstacle:
    def __init__(self, x, y, theta, velocity, acceleration, robot):
        self.x = x
        self.y = y
        self.theta = theta
        self.velocity = velocity
        self.acceleration = acceleration
        self.horizontal_velocity = velocity * math.cos(theta)
        self.vertical_velocity = velocity * math.sin(theta)
        self.horizontal_acceleration = acceleration * math.cos(theta)
        self.vertical_acceleration = acceleration * math.sin(theta)
        self.robot = robot

    # determines a set of points along the perimeter of the obstacle cost field
    def perimeter_points(self):
        # todo implement velocity cap
        horizontal_collision_time = self.time_to_collision(self.robot.horizontal_velocity,
                                                           self.horizontal_velocity,
                                                           self.robot.horizontal_acceleration,
                                                           self.horizontal_acceleration)
        vertical_collision_time = self.time_to_collision(self.robot.vertical_velocity,
                                                         self.vertical_velocity,
                                                         self.robot.vertical_acceleration,
                                                         self.vertical_acceleration)
        return math.sqrt(horizontal_collision_time ** 2 + vertical_collision_time ** 2)

    def time_to_collision(self, va0, vb0, xa0, xb0):
        a = self.acceleration  # acceleration constant
        b = va0 - vb0  # single-axis component of each bot's initial velocity
        c = xa0 - xb0  # single-axis difference between each bot's initial position

        # calculate the discriminant
        d = abs((b ** 2) - (4 * a * c))

        return (-b + math.sqrt(d)) / (2 * a)


    def time_to_point(self, x, y):
        # TODO: incorporate current velocity/direction
        return math.sqrt((self.x - x)**2 + (self.y - y)**2) / self.velocity

    def cost_of_point_after_time(self, x, y, time):
        x = self.time_to_point(x, y)
        return 1 - 1 / (1 + math.exp(-x))