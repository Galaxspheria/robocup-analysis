from matplotlib import cm
import matplotlib.pyplot as plt
import matplotlib.path as pth
import matplotlib.patches as patches
import obstacle
import math
import robot

# Distances in cm
FIELD_DIM_X = 600
FIELD_DIM_Y = 900
# STEP_COST = 3
# SPLITS = 4

# def pathfind(location, target, curPath = []):
#
#     if location == target:
#         return target
#     if len(curPath) > 20 or location[0] < 0 or location[1] < 0 or location[0] > FIELD_DIM or location[1] > FIELD_DIM:
#         return
#     print(location)
#     paths = []
#     for split in range(SPLITS):
#         paths.append(pathfind([location[0] + random.randint(-1, 1), location[1] + random.randint(-1, 1)], target, curPath + location))

# print(pathfind([5, 3], [6, 5]))


def main():
    bot = robot.Robot(0, 0, math.pi / 2, 2)
    # obstacles = [obstacle.Obstacle(10, 2, 1, 3, bot), obstacle.Obstacle(3, 4, 1, 1, bot),
    #              obstacle.Obstacle(5, 2, 1, 1, bot), obstacle.Obstacle(15, 8, 1, 1, bot),
    #              obstacle.Obstacle(5, 13, 1, 1, bot)]
    obstacles = [obstacle.Obstacle(200, 300, 1, 50, bot), obstacle.Obstacle(500, 100, -2, 20, bot)]
    start = [1, 1]
    target = [18, 18]

    field = [[0 for _ in range(FIELD_DIM_X)] for _ in range(FIELD_DIM_Y)]

    for obs in obstacles:
        perimeter = obs.perimeter_points();
        for point in perimeter:
            if 0 <= point[0] < FIELD_DIM_X and 0 <= point[1] < FIELD_DIM_Y:
                field[point[0]][point[1]] += 100
                for x in range(-10, 10):
                    for y in range(-10, 10):
                        if not x * y == 0:
                            field[point[0] + x][point[1] + y] += 100 / (abs(x * y))
        # for y in range(FIELD_DIM_Y):
        #     for x in range(FIELD_DIM_X):
        #         field[y][x] += obs.cost_of_point_after_time(x, y, 2)

    for y in range(FIELD_DIM_Y):
        for x in range(FIELD_DIM_X):
            field[y][x] /= len(obstacles)

    fig, ax = plt.subplots()
    ax.axis('equal')

    bestPath = [target, [13, 12], [1, 12], start]
    codes = [pth.Path.MOVETO] + [pth.Path.CURVE4 for _ in range(len(bestPath) - 1)]
    path = pth.Path(bestPath, codes)
    pathPatch = patches.PathPatch(path, facecolor='none', edgecolor='black', alpha=0.5, lw=3)
    startCir = patches.Circle(start, radius=0.5, color='black')
    targetCir = patches.Circle(target, radius=0.5, color='black')
    ax.add_patch(pathPatch)
    ax.add_patch(startCir)
    ax.add_patch(targetCir)

    CS = ax.contourf(field, cmap=cm.Spectral_r)
    cbar = fig.colorbar(CS)
    ax.set_title('Field Pathfinding Costs')
    plt.show(block=True)


if __name__ == '__main__':
    main()
    # robot = robot.Robot(0, 0, math.pi/2, 2)
    # obstacle = obstacle.Obstacle(5, 5, 0*math.pi/4, 5, robot)
    # print(obstacle.perimeter_points())
