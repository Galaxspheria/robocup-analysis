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
GOAL = (500, 600)
STEPTH = 0.2


def main():
    bot = robot.Robot(10, 10, math.pi / 2, 2)
    obstacles = [obstacle.Obstacle(200, 200, 1, 50, bot), obstacle.Obstacle(500, 100, -2, 20, bot)]
    start = [1, 1]
    target = [18, 18]

    field = [[0 for _ in range(FIELD_DIM_X)] for _ in range(FIELD_DIM_Y)]

    for y in range(FIELD_DIM_Y):
        for x in range(FIELD_DIM_X):
            field[y][x] += math.sqrt((GOAL[0] - x) ** 2 + (GOAL[1] - y) ** 2) * STEPTH

    for obs in obstacles:
        p = obs.perimeter_points()
        min_x = min(list(map(lambda pt: pt[0], p[0])))
        max_x = max(list(map(lambda pt: pt[0], p[0])))
        min_y = min(list(map(lambda pt: pt[1], p[0])))
        max_y = max(list(map(lambda pt: pt[1], p[0])))
        avg_perimeter = list(map(lambda ptA, ptB: ((ptA[0] + ptB[0])/2, (ptA[1] + ptB[1])/2), p[0], p[1]))
        midpoint_x = (min_x + max_x)//2
        midpoint_y = (min_y + max_y)//2

        for y in range(min_y, max_y):
            # actually half of the strip length
            strip_length = round(math.sqrt(((max_y - min_y)/2)**2 - (y - midpoint_y)**2))
            for x in range(midpoint_x - (strip_length - 1), midpoint_x + (strip_length - 1)):
                if 0 <= x < FIELD_DIM_X and 0 <= y < FIELD_DIM_Y:
                    # 1. get closest point in avg_perimeter
                    closest_distance = (x - avg_perimeter[0][0])**2 + (y - avg_perimeter[0][1])**2
                    for coord in avg_perimeter:
                        distance = (x - coord[0])**2 + (y - coord[1])**2
                        if distance < closest_distance:
                            closest_distance = distance
                    # 2. get distance from center
                    center = (obs.x, obs.y)
                    dist_from_center = (x - center[0])**2 + (y - center[1])**2
                    # 3. check which one is the closest
                    closest_distance = closest_distance if closest_distance < dist_from_center else dist_from_center
                    # 4. scale the distance
                    if not closest_distance == 0:
                        field[y][x] += 30 * .97**(math.sqrt(closest_distance))

    for y in range(FIELD_DIM_Y):
        for x in range(FIELD_DIM_X):
            field[y][x] /= len(obstacles)

    fig, ax = plt.subplots()
    ax.axis('equal')

    best_path = pathfind(field, (bot.x, bot.y), GOAL)
    smooth_path = smooth(best_path)
    codes = [pth.Path.MOVETO] + [pth.Path.CURVE3 for _ in range(len(smooth_path) - 1)]
    path = pth.Path(smooth_path, codes)
    path_patch = patches.PathPatch(path, facecolor='none', edgecolor='black', alpha=0.5, lw=3)
    start_cir = patches.Circle(start, radius=0.5, color='black')
    target_cir = patches.Circle(target, radius=0.5, color='black')
    ax.add_patch(path_patch)
    ax.add_patch(start_cir)
    ax.add_patch(target_cir)

    CS = ax.pcolormesh(field, cmap=cm.Spectral_r)
    fig.colorbar(CS)
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
