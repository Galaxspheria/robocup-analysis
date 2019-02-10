from matplotlib import cm
import matplotlib.pyplot as plt
import matplotlib.path as pth
import matplotlib.patches as patches
import obstacle
import math
import robot
# import bezier
# import numpy as np
from functools import reduce

# Distances in cm
FIELD_DIM_X = 600
FIELD_DIM_Y = 900
GOAL = (500, 600)
STEPTH = 0.2
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
    bot = robot.Robot(10, 10, math.pi / 2, 2)
    # obstacles = [obstacle.Obstacle(10, 2, 1, 3, bot), obstacle.Obstacle(3, 4, 1, 1, bot),
    #              obstacle.Obstacle(5, 2, 1, 1, bot), obstacle.Obstacle(15, 8, 1, 1, bot),
    #              obstacle.Obstacle(5, 13, 1, 1, bot)]
    obstacles = [obstacle.Obstacle(200, 200, 1, 50, bot), obstacle.Obstacle(500, 100, -2, 20, bot)]
    start = [1, 1]
    target = [18, 18]

    field = [[0 for _ in range(FIELD_DIM_X)] for _ in range(FIELD_DIM_Y)]

    for y in range(FIELD_DIM_Y):
        for x in range(FIELD_DIM_X):
            field[y][x] += math.sqrt((GOAL[0] - x) ** 2 + (GOAL[1] - y) ** 2) * STEPTH

    for obs in obstacles:
        p = obs.perimeter_points()
        perimeter = p[0] + p[1] + p[2]
        minX = min(list(map(lambda pt: pt[0], p[0])))
        maxX = max(list(map(lambda pt: pt[0], p[0])))
        minY = min(list(map(lambda pt: pt[1], p[0])))
        maxY = max(list(map(lambda pt: pt[1], p[0])))
        # perimeter = []
        # for i in range(len(p[0])):
        #     perimeter.append(p[0].pop())
        #     perimeter.append(p[1][i])

        # for i in range(len(p[1])):
        #     perimeter.append(p[1].pop())
        #     perimeter.append(p[2].pop())
        # for point in perimeter:
        #     if 0 <= point[0] < FIELD_DIM_X and 0 <= point[1] < FIELD_DIM_Y:
        #         field[point[0]][point[1]] += 100
        #         for x in range(-10, 10):
        #             for y in range(-10, 10):
        #                 if not x * y == 0:
        #                     field[point[0] + x][point[1] + y] += 100 / (abs(x * y))

        for y in range(minY, maxY):
            for x in range(minX, maxX):
                if (0 <= x < FIELD_DIM_X and 0 <= y < FIELD_DIM_Y):
                    field[y][x] += (((maxY - minY) * (maxX - minX)) ** (1/2) * 200 / sum(map(lambda pt: math.sqrt((pt[0] - x) ** 2 + (pt[1] - y) ** 2), perimeter)))**2

        # pln = len(perimeter)
        # for pindex in range(pln):
        #     x = perimeter[pindex][0]
        #     y = perimeter[pindex][1]
        #     while not perimeter[(pindex + 1) % pln][0] - x == 0 or not perimeter[(pindex + 1) % pln][1] - y == 0:
        #         if 0 <= perimeter[pindex][0] < FIELD_DIM_X and 0 <= perimeter[pindex][1] < FIELD_DIM_Y:
        #             field[int(y)][int(x)] += 100
        #         if not perimeter[(pindex + 1) % pln][0] - x == 0:
        #             x += (perimeter[(pindex + 1) % pln][0] - x) / abs(perimeter[(pindex + 1) % pln][0] - x)
        #         if not perimeter[(pindex + 1) % pln][1] - y == 0:
        #             y += (perimeter[(pindex + 1) % pln][1] - y) / abs(perimeter[(pindex + 1) % pln][1] - y)
        # mid = (int(sum(map(lambda point: point[0], perimeter)) / pln), int(sum(map(lambda point: point[1], perimeter)) / pln))


        # print(mid)
        # for y in range(-50, 50):
        #     for x in range(-50, 50):
        #         if 0 <= x < FIELD_DIM_X and 0 <= y < FIELD_DIM_Y:
        #             if not x * y == 0:
        #                 field[mid[1] + y][mid[0] + x] += 200 / (abs(x * y))
        # for y in range(FIELD_DIM_Y):
        #     for x in range(FIELD_DIM_X):
        #         field[y][x] += obs.cost_of_point_after_time(x, y, 2)

    for y in range(FIELD_DIM_Y):
        for x in range(FIELD_DIM_X):
            field[y][x] /= len(obstacles)

    fig, ax = plt.subplots()
    ax.axis('equal')

    bestPath = pathfind(field, (bot.x, bot.y), GOAL)
    smoothPath = smooth(bestPath)
    # smoothPath = smoothPath[:-1]
    # print(smoothPath)
    # npPath = np.asfortranarray(smoothPath).transpose().astype(float)
    # print(npPath)
    # curve = bezier.Curve.from_nodes(npPath)
    # print(curve)
    # print(curve.plot(num_pts=128))
    # bestPath = [target, [13, 12], [1, 12], start]
    codes = [pth.Path.MOVETO] + [pth.Path.CURVE3 for _ in range(len(smoothPath) - 1)]
    path = pth.Path(smoothPath, codes)
    pathPatch = patches.PathPatch(path, facecolor='none', edgecolor='black', alpha=0.5, lw=3)
    startCir = patches.Circle(start, radius=0.5, color='black')
    targetCir = patches.Circle(target, radius=0.5, color='black')
    ax.add_patch(pathPatch)
    ax.add_patch(startCir)
    ax.add_patch(targetCir)

    CS = ax.pcolormesh(field, cmap=cm.Spectral_r)
    cbar = fig.colorbar(CS)
    ax.set_title('Field Pathfinding Costs')
    plt.show(block=True)

def unitWith0(val):
    if val == 0:
        return 0
    out = val/(abs(val))
    return out

def pathfind(field, start, end):
    x = start[0]
    y = start[1]
    cost = 0
    path = [start]

    while x != end[0] or y != end[1]:
        minChoice = [0, 0]
        minCost = math.inf
        for i in range(-1, 2):
            for j in range (-1, 2):
                if (i != 0 or j != 0) and (0 < y+i < FIELD_DIM_Y and 0 < x+j < FIELD_DIM_X):
                    cost = field[y+i][x+j] # + (-13 * j * unitWith0(end[0]-x)) + (-13 * i * unitWith0(end[1]-y))
                    # TODO: isn't down gradient by definition the right direction?
                    if cost < minCost:
                        minChoice = [j, i]
                        minCost = cost
        x += minChoice[0]
        y += minChoice[1]
        path.append([x, y])
        cost += minCost
    return path

def smooth(path):
    index = 0
    corrections = 0

    while index < len(path) - 1:
        dx1 = path[index][0] - path[index - 1][0]
        dx2 = path[index + 1][0] - path[index][0]
        dx1 = dx1 if not dx1 == 0 else 0.001
        dx2 = dx2 if not dx2 == 0 else 0.001
        dy1 = path[index][1] - path[index - 1][1]
        dy2 = path[index + 1][1] - path[index][1]
        if dy1/dx1 == dy2/dx2:
            path.pop(index)
            corrections += 1
        else:
            index += 1

    if corrections == 0:
        return path

    return smooth(path)

if __name__ == '__main__':
    main()
    # robot = robot.Robot(0, 0, math.pi/2, 2)
    # obstacle = obstacle.Obstacle(5, 5, 0*math.pi/4, 5, robot)
    # print(obstacle.perimeter_points())
