import matplotlib.pyplot as plt
import numpy as np
import os
import time
import json

""" You can use this script to visualize the results from test_hector_iterators.cpp
It shows the found points inside the polygon as well as the groundtruth used during testing
also the limits are visualized. The function will iterate over all Testcase reports present in 'test/tmp' """


def draw_nice_visualisation(corners, real_points, iterated_points, limits, name="Visualization"):
    x, y = np.zeros(len(corners) + 1), np.zeros(len(corners) + 1)
    for i in range(len(corners)):
        x[i] = corners[i][0]
        y[i] = corners[i][1]
    x[-1], y[-1] = x[0], y[0]
    # draw polygon
    plt.plot(x, y, linewidth=4)
    plt.grid(True)
    miny = int(np.floor(min([x[1] for x in corners]))) - 1
    maxy = int(np.ceil(max([x[1] for x in corners]))) + 1
    minx = int(np.floor(min([x[0] for x in corners]))) - 1
    maxx = int(np.ceil(max([x[0] for x in corners]))) + 1
    # draw centers
    centers = np.array(np.meshgrid(np.arange(minx, maxx) + 0.5,
                                   np.arange(miny, maxy) + 0.5)).reshape(2, (maxx - minx) * (maxy - miny))
    plt.scatter(centers[0, :], centers[1, :])
    # draw iterated points (found during testing)
    plt.scatter([x[0] + 0.5 for x in iterated_points], [x[1] + 0.5 for x in iterated_points], s=100,
                facecolors='none', edgecolors='red', linewidths=2, label="iterated Positions")
    # draw 'groundtruth' data
    plt.scatter([x[0] + 0.5 for x in real_points], [x[1] + 0.5 for x in real_points], c="green", label="groundtruth")
    # draw limits
    plt.plot([limits[0], limits[0], limits[1] - 0.25, limits[1] - 0.25, limits[0]],
             [limits[2], limits[3] - 0.25, limits[3] - 0.25, limits[2], limits[2]],
             linestyle='dashed', label="limits")
    plt.legend()  # bbox_to_anchor=(0.75, 1.15), ncol=2)
    plt.xlim((minx, maxx))
    plt.ylim((miny, maxy))
    plt.title(name)
    plt.show()


def read_from_file(path):
    with open(path) as f:
        data = json.load(f)
    return data["iterated positions"], data["real positions"], data["corners"], data["limits"]


# iterates over all files in folder and visualizes the failure reports
def show_all_failure_cases():
    all_files = os.listdir("tmp")
    for file_name in all_files:
        file = "tmp/" + file_name
        modification_time = time.strftime('%d/%m/%Y %H:%M', time.localtime(os.path.getmtime(file)))
        iterated_points, real_points, corners, limits = read_from_file(file)
        draw_nice_visualisation(corners, real_points, iterated_points, limits,
                            file[4:] + " \n changed last at " + modification_time)


if __name__ == "__main__":
    # visualize all failure cases
    show_all_failure_cases()
