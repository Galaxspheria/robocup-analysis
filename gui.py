from matplotlib import cm
import matplotlib.pyplot as plt
import matplotlib.path as pth
import matplotlib.patches as patches
import obstacle
import random

FIELD_DIM = 20
STEP_COST = 3
SPLITS = 4

obstacles = [obstacle.Obstacle(10, 2, 1, 3), obstacle.Obstacle(3, 4, 1, 1), obstacle.Obstacle(5, 2, 1, 1), obstacle.Obstacle(15, 8, 1, 1), obstacle.Obstacle(5, 13, 1, 1)]
start = [1, 1]
target = [18, 18]

field = [[0 for _ in range (FIELD_DIM)] for _ in range(FIELD_DIM)]

for obs in obstacles:
    for y in range(FIELD_DIM):
        for x in range(FIELD_DIM):
            field[y][x] += obs.cost_of_point_after_time(x, y, 2)

for y in range(FIELD_DIM):
    for x in range(FIELD_DIM):
        field[y][x] /= len(obstacles)

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
    fig, ax = plt.subplots()
    # CS = ax.contour(X, Y, Z)
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
