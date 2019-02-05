import math
import robot

SAMPLE_COUNT = 20
DELTA_DEGREES = 2 * math.pi / SAMPLE_COUNT
# TODO: experiment with all of these eventually
COLLISION_TIME_CONSTANT = 1
DIST_FROM_OUTER_PERIMETER = 0.2
DIST_FROM_CENTER = 0.2


class Obstacle(robot.Robot):
    def __init__(self, x, y, theta, velocity, main_robot):
        super().__init__(x, y, theta, velocity)
        self.main_robot = main_robot  # the robot that we are path finding for

    # determines a set of points along the perimeter of the obstacle cost field
    def perimeter_points(self):

        perimeter = []
        inner_perimeter = []
        center_perimeter = []

        time = self.time_to_collision(self.main_robot.x - self.x,
                                      self.main_robot.y - self.y,
                                      self.main_robot.velocity_cap)

        for i in range(SAMPLE_COUNT):
            accTheta = i * DELTA_DEGREES

            xCon = round(math.cos(accTheta), 3)
            yCon = round(math.sin(accTheta), 3)

            xPos = self.position_after_acc(self.x,
                                           self.velocity * round(math.cos(self.theta), 3),
                                           self.main_robot.acceleration * round(math.cos(accTheta), 3),
                                           time,
                                           self.main_robot.velocity_cap * (0 if (xCon == 0) else (1 if (xCon > 0) else -1)))
            yPos = self.position_after_acc(self.y,
                                           self.velocity * round(math.sin(self.theta), 3),
                                           self.main_robot.acceleration * round(math.sin(accTheta), 3),
                                           time,
                                           self.main_robot.velocity_cap * (0 if (yCon == 0) else (1 if (yCon > 0) else -1)))
            perimeter.append((int(xPos), int(yPos)))
        x_coordinates = list(map(lambda coord: coord[0], perimeter))
        y_coordinates = list(map(lambda coord: coord[1], perimeter))
        center_x_pos = sum(x_coordinates)/len(x_coordinates)
        center_y_pos = sum(y_coordinates)/len(y_coordinates)
        inner_perimeter = map(lambda coordinate:
                              (round((coordinate[0] - center_x_pos) * (1 - DIST_FROM_OUTER_PERIMETER) + center_x_pos),
                               round((coordinate[1] - center_y_pos) * (1 - DIST_FROM_OUTER_PERIMETER) + center_y_pos)),
                              perimeter)
        center_perimeter = map(lambda coordinate:
                               (round((coordinate[0] - center_x_pos) * DIST_FROM_CENTER + center_x_pos),
                                round((coordinate[1] - center_y_pos) * DIST_FROM_CENTER + center_y_pos)),
                               perimeter)
        return perimeter, list(inner_perimeter), list(center_perimeter)

    def time_to_collision(self, dx, dy, cap):
        return (math.sqrt(dx**2 + dy**2) / cap) * COLLISION_TIME_CONSTANT

    def position_after_acc(self, x0, v0, acc, time, cap):
        # print("acc: " + str(acc))
        if (acc == 0):
            return x0 + (time * v0)
        else:
            precapTime = (cap - v0) / acc
            if (precapTime > time):
                return x0 + (v0 * time + 0.5 * acc * time ** 2)
            else:
                return x0 + (v0 * precapTime + 0.5 * acc * precapTime ** 2) + ((time - precapTime) * cap)

    def time_to_point(self, x, y):
        # TODO: incorporate current velocity/direction
        return math.sqrt((self.x - x)**2 + (self.y - y)**2) / self.velocity

    def cost_of_point_after_time(self, x, y, time):
        x = self.time_to_point(x, y)
        return 1 - 1 / (1 + math.exp(-x))