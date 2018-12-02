import math
import robot

DELTA_DEGREES = math.pi/3


class Obstacle(robot.Robot):
    def __init__(self, x, y, theta, velocity, main_robot):
        super().__init__(x, y, theta, velocity)
        self.main_robot = main_robot  # the robot that we are path finding for

    # determines a set of points along the perimeter of the obstacle cost field
    def perimeter_points(self):
        # calculates time components for each axis and then finds the time componenet for the hypotenuse
        horizontal_acc_to_main, horizontal_acc_to_obstacle, vertical_acc_to_main, vertical_acc_to_obstacle = \
            self.__acceleration_components_between_robots()
        horizontal_collision_time = self.__time_to_collision(self.main_robot.horizontal_velocity,
                                                             self.horizontal_velocity,
                                                             self.main_robot.x,
                                                             self.x,
                                                             horizontal_acc_to_obstacle,
                                                             horizontal_acc_to_main)
        return horizontal_collision_time
        # vertical_collision_time = self.time_to_collision(self.main_robot.vertical_velocity,
        #                                                  self.vertical_velocity,
        #                                                  self.main_robot.y,
        #                                                  self.y)
        # return math.sqrt(horizontal_collision_time ** 2 + vertical_collision_time ** 2)

    def __acceleration_components_between_robots(self):
        # todo fill this in (it should yield the same result as a_acceleration_theta)
        x0 = self.x
        y0 = self.y
        x1 = self.main_robot.x
        y1 = self.main_robot.y
        theta_obstacle_to_main = math.atan((x0 - x1) / (y0 - y1))
        theta_main_to_obstacle = math.atan((y0 - y1) / (x0 - x1))
        if x0 - x1 >= 0:  # if the main robot is in the second or third quadrant compared to the obstacle
            theta_obstacle_to_main = theta_obstacle_to_main + math.pi
        else:
            theta_main_to_obstacle = theta_main_to_obstacle + math.pi
        horizontal_acc_to_main = self.acceleration * math.cos(theta_obstacle_to_main)
        horizontal_acc_to_obstacle = self.acceleration * math.cos(theta_main_to_obstacle)
        vertical_acc_to_main = self.acceleration * math.sin(theta_obstacle_to_main)
        vertical_acc_to_obstacle = self.acceleration * math.sin(theta_main_to_obstacle)
        return horizontal_acc_to_main, horizontal_acc_to_obstacle, vertical_acc_to_main, vertical_acc_to_obstacle

    def __time_to_collision(self, va0, vb0, xa0, xb0, aa, ab):
        # makes sure xa0 is greater than xa0 so the rest of the calculations work
        if xb0 > xa0:
            placeholder_position = xa0
            placeholder_velocity = va0
            xa0 = xb0
            va0 = vb0
            xb0 = placeholder_position
            vb0 = placeholder_velocity
        # todo change these to factor in the robots are pointing at each other. figure out theta of acceleration.
        aa = -self.acceleration
        ab = self.acceleration
        # acceleration constant
        a = .5 * (ab - aa)
        # single-axis component of each bot's initial velocity
        b = vb0 - va0
        # single-axis difference between each bot's initial position
        c = xb0 - xa0

        # figure out how long it will take each robot to hit the velocity cap
        a_time_under_cap = (-self.velocity_cap - va0) / aa
        b_time_under_cap = (self.velocity_cap - vb0) / ab
        shortest_time_under_cap = a_time_under_cap if a_time_under_cap < b_time_under_cap else b_time_under_cap
        longest_time_under_cap = a_time_under_cap if a_time_under_cap > b_time_under_cap else b_time_under_cap

        # calculate positions of both at the shortest time under cap
        a_position = xa0 + va0 + .5 * aa * shortest_time_under_cap ** 2
        b_position = xb0 + vb0 + .5 * ab * shortest_time_under_cap ** 2

        if a_position < b_position:  # if they cross each other before hitting the velocity cap
            # returns the time the robots will meet at
            d = (b ** 2) - (4 * a * c)
            return (-b + math.sqrt(d)) / (2 * a)
        else:
            # sets a new starting position for the robots
            xa0 = a_position
            xb0 = b_position
            delta_time = longest_time_under_cap - shortest_time_under_cap # calculates new time for next calculation
            if a_time_under_cap is shortest_time_under_cap:  # if b is still accelerating but a isn't
                vb0 = vb0 + ab * shortest_time_under_cap  # calculate initial velocity of b at the shortest time cap
                # calculates the final positions
                a_position = xa0 - self.velocity_cap * delta_time
                b_position = xb0 + vb0 * delta_time + .5 * ab * delta_time ** 2
                component_of_acceleration = .5 * ab  # sets the a of the quadractic
            else:
                va0 = va0 + aa * shortest_time_under_cap  # calculates initial velocity of a at the shortest time cap
                # calculates the final position
                a_position = xa0 + va0 * delta_time + .5 * aa * delta_time ** 2
                b_position = xb0 + self.velocity_cap * delta_time
                component_of_acceleration = .5 * aa  # sets the a of the quadratic
            if a_position < b_position:  # if they cross each other in the previous time interval
                # same as above - quadratic formula but with one of them not accelerating
                b = vb0 - va0
                c = xb0 - xa0
                d = (b ** 2) - (4 * component_of_acceleration * c)
                return (-b + math.sqrt(d)) / 2 * component_of_acceleration
            else:
                # new starting points for the robots post hitting the velocity cap
                xa0 = a_position
                xb0 = b_position
                return (xa0 - xb0) / (2 * self.velocity_cap)  # returns the time (not accelerating anymore)

    def time_to_point(self, x, y):
        # TODO: incorporate current velocity/direction
        return math.sqrt((self.x - x)**2 + (self.y - y)**2) / self.velocity

    def cost_of_point_after_time(self, x, y, time):
        x = self.time_to_point(x, y)
        return 1 - 1 / (1 + math.exp(-x))