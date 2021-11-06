import matplotlib.pyplot as plt
import numpy as np

# corners should be in correct order (random direction)
INPUT_STRING = """hector_math::Polygon<Scalar> result(2, 8);
        result.col(0) << 0, 5;
        result.col(1) << -3.6,3.5;
        result.col(2) << -5,0;
        result.col(3) << -3.6,-3.5;
        result.col(4) << 0, -5;
        result.col(5) << 3.6, -3.5;
        result.col(6) << 5,0;
        result.col(7) << 3.6,3.5;"""

# LIMITS (row_min, row_max, column_min, column_max)
#LIMITS = (0, 6, -6, 6)
LIMITS = (-4, 2, -3, 1)


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
    if len(real_points)==0:
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


if __name__ == "__main__":
    corners = get_corners(INPUT_STRING)
    draw_polygon(corners)
    # get_real_points(corners)
