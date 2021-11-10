import matplotlib.pyplot as plt
import numpy as np
import os
import time

""" You can use this script to visualize the results from test_hector_iterators.cpp
It shows the found points inside the polygon as well as the groundtruth used during testing
also the LIMITS are visualized. The function will iterate over all Testcase reports present in test/tmp

Alternativley the script can be used to build up new test cases, given an INPUT_STRING (corners)
ist should detect all points inside the polygon (neither optimized not perfect). The results can be visualized and the
detected points are printed so that they can be easily copied as 'real_points' of the new test case
"""


# corners should be in correct order (random direction)
INPUT_STRING = """hector_math::Polygon<Scalar> result(2, 8);
        result.col(0) << -3.5, 4.2;
        result.col(1) << -2, 4.2;
        result.col(2) << -2, -0.5;
        result.col(3) << 1, -0.5;
        result.col(4) << 1, 4.2;
        result.col(5) << 3.5, 4.2;
        result.col(6) << 5.5, -0.5;
        result.col(7) << 4.5, -2;
        result.col(8) << 2.4, 3.5;
        result.col(9) << 2.4, -2;
        result.col(10) << -3.5, -2;"""

# LIMITS (row_min, row_max, column_min, column_max)
LIMITS = [-6, 6, -6, 6]





def get_corners(string):
    corners = []
    start = 0
    temp = 0
    end = 0
    while True:
        start = string.find("<<", end)
        end = string.find(",", start)
        if start == -1 or end == -1:
            return corners
        print(f"Loop {temp}: potential first number #{string[start + 3:end].strip()}# with start {start} and end{end}")
        x = float(string[start + 3:end])
        start = end + 1
        end = string.find(";", start)
        if start == -1 or end == -1:
            return corners
        print(f"Loop {temp}: potential second number #{string[start:end].strip()}# with start {start} and end{end}")
        y = float(string[start:end])
        corners.append((x, y))
        temp += 1
        if end + 3 > len(string):
            return corners


def draw_polygon(corners):
    x = np.zeros(len(corners) + 1)
    y = np.zeros(len(corners) + 1)
    for i in range(len(corners)):
        x[i] = corners[i][0]
        y[i] = corners[i][1]
    x[-1] = x[0]
    y[-1] = y[0]
    plt.plot(x, y, linewidth=4)
    points_x = []
    points_y = []
    for i in range(int(np.floor(np.min(x))), int(np.ceil(np.max(x))) + 1):
        for j in range(int(np.floor(np.min(y))), int(np.ceil(np.max(y))) + 1):
            points_x.append(i)
            points_y.append(j)
            plt.plot([i, i + 0.5], [j, j + 0.5], c="blue")
    # plt.scatter(points_x,points_y,c="red")
    real_points = get_real_points(corners)
    plt.scatter([x[0] for x in real_points], [x[1] for x in real_points], c="red")
    plt.show()
    get_real_positions_lazy(real_points)
    draw_nice_visualisation(corners, real_points)


def draw_nice_visualisation(corners, real_points, iterated_points=[],name="Visualisation"):
    x = np.zeros(len(corners) + 1)
    y = np.zeros(len(corners) + 1)
    for i in range(len(corners)):
        x[i] = corners[i][0]
        y[i] = corners[i][1]
    x[-1] = x[0]
    y[-1] = y[0]
    # draw polygon
    plt.plot(x, y, linewidth=4)
    plt.grid(True)
    miny = int(np.floor(min([x[1] for x in corners]))) - 1
    maxy = int(np.ceil(max([x[1] for x in corners]))) + 1
    minx = int(np.floor(min([x[0] for x in corners]))) - 1
    maxx = int(np.ceil(max([x[0] for x in corners]))) + 1
    # draw centers
    centers = np.array(np.meshgrid(np.arange(minx, maxx)+0.5, np.arange(miny, maxy)+0.5)).reshape(2, (maxx-minx)*(maxy-miny))
    plt.scatter(centers[0, :], centers[1, :])
    # draw iterated points (found during testing)
    plt.scatter([x[0] + 0.5 for x in iterated_points], [x[1] + 0.5 for x in iterated_points], s=100,
                facecolors='none', edgecolors='red', linewidths=2, label="iterated Positions")
    # draw 'groundtruth' data
    plt.scatter([x[0] + 0.5 for x in real_points], [x[1] + 0.5 for x in real_points], c="green", label="groundtruth")
    # draw Limits
    plt.plot([LIMITS[0], LIMITS[0], LIMITS[1]-0.25, LIMITS[1]-0.25,LIMITS[0]], [LIMITS[2], LIMITS[3]-0.25, LIMITS[3]-0.25, LIMITS[2],LIMITS[2]],
             linestyle='dashed', label="LIMITS")
    plt.legend()#bbox_to_anchor=(0.75, 1.15), ncol=2)
    plt.xlim((minx,maxx))
    plt.ylim((miny,maxy))
    plt.title(name)
    plt.show()


def get_real_points(corners):
    lines = []
    real_points = []
    for i in range(len(corners) - 1):
        lines.append((corners[i][0], corners[i][1], corners[i + 1][0], corners[i + 1][1]))
    lines.append((corners[-1][0], corners[-1][1], corners[0][0], corners[0][1]))
    # sort lines by min y
    lines.sort(key=lambda x: min(x[1], x[3]))
    miny = min([min(x[1], x[3]) for x in lines])
    maxy = max([max(x[1], x[3]) for x in lines])
    y = np.floor(miny) - 0.5
    while y < maxy + 1.5:
        relevant_lines = [x for x in lines if min(x[1], x[3]) <= y < max(x[1], x[3])]
        if len(relevant_lines) == 0:
            y += 1
            continue
        minx = min([min(x[0], x[2]) for x in relevant_lines])
        maxx = max([max(x[0], x[2]) for x in relevant_lines])
        crossings = []
        for line in relevant_lines:
            temp = (y - line[3]) / (line[1] - line[3])
            if 0 <= temp <= 1:
                crossings.append(line[2] + temp * (line[0] - line[2]))
        crossings.sort()
        x = np.floor(minx) - 0.5
        crossing_index = 0
        while x < maxx and crossing_index < len(crossings):
            # count crossing until point
            while crossings[crossing_index] < x:
                crossing_index += 1
                if crossing_index == len(crossings):
                    break
            # crossing_index = max(0,crossing_index-1)
            if crossing_index % 2 == 1:
                real_points.append((x - 0.5, y - 0.5))
            x += 1
        y += 1
    # filter by index limitations
    valid_real_points = []
    for point in real_points:
        if LIMITS[2] <= point[1] < LIMITS[3] and LIMITS[0] <= point[0] < LIMITS[1]:
            valid_real_points.append(point)
    print(valid_real_points)
    return valid_real_points


def get_real_positions_lazy(real_points):
    if len(real_points) == 0:
        print("{};")
    s = "{"
    y_old = real_points[0][1]
    for point in real_points:
        if point[1] > y_old:
            s = s + "\n"
            y_old = point[1]
        s = s + "{" + str(point[0]) + "," + str(point[1]) + "},"
    s = s[0:-1] + "};"
    print(s)


def read_from_file(path):
    global LIMITS
    file = open(path, 'r')
    iterated_points = []
    real_points = []
    corners = []
    LIMITS = []
    mode = -2
    while True:
        line = file.readline()
        if not line:
            break
        position = line.find(",")
        if position < 0:
            mode += 1
        else:
            num1 = float(line[0: position])
            num2 = float(line[position + 1:-1])
            if mode == -1:
                LIMITS.append(num1)
                LIMITS.append(num2)
            elif mode == 0:
                iterated_points.append((num1, num2))
            elif mode == 1:
                real_points.append((num1, num2))
            else:
                corners.append((num1, num2))
    return iterated_points, real_points, corners


if __name__ == "__main__":
    all_files = os.listdir("tmp")
    files = ["tmp/" +file for file in all_files]
    print(files)
    while len(files) > 0:
        file = max(files, key=os.path.getctime)
        modificationTime = time.strftime('%d/%m/%Y %H:%M', time.localtime(os.path.getmtime(file)))
        iterated_points, real_points, corners = read_from_file(file)
        draw_nice_visualisation(corners, real_points, iterated_points,file[4:]+" \n changed last at "+modificationTime)
        files.remove(file)
    exit()
    if True:
        # Visualising results from Tests
        file = "TestCaseCircleShape.txt"# "TestCasePolygonZShape.txt"# "TestCaseCircleShapeLimitedIndexes.txt"  # "TestCasePolygonRandom.txt"
        iterated_points, real_points, corners = read_from_file(file)
        draw_nice_visualisation(corners, real_points, iterated_points)
        print(LIMITS)
    else:
        ## for calculating "groundtruth" corners (needs to be verified visually!)
        ## if wrong check LIMITS variable
        corners = get_corners(INPUT_STRING)
        real_position = get_real_points(corners)
        draw_nice_visualisation(corners, real_position)
        get_real_positions_lazy(real_position)
