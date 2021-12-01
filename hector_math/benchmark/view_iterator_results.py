#!/usr/bin/env python
import numpy as np
from matplotlib.patches import Polygon
import matplotlib.pyplot as plt

if __name__ == '__main__':
    polygon_map = np.loadtxt("polygon.txt", delimiter=',')
    quadrangle_map = np.loadtxt("quadrangle_polygon.txt", delimiter=',')
    polygon = Polygon(np.flip(np.array([
        [0.480, 0.000],
        [0.164, 0.155],
        [0.116, 0.500],
        [-0.133, 0.250],
        [-0.480, 0.399],
        [-0.316, 0.000],
        [-0.480, -0.399],
        [-0.133, -0.250],
        [0.116, -0.500],
        [0.164, -0.155],
        [0.480, 0.000],
    ]) + np.array([0.5, 0.5]), axis=1) / 0.05 - np.array([0.5, 0.5]), fill=None, color='red')
    quadrangle = Polygon(np.flip(np.array([
        [0.035, 0.035],
        [0, 0.96],
        [1, 1],
        [0.95, 0.0],
        [0.035, 0.035]
    ]), axis=1) / 0.05 - np.array([0.5, 0.5]), fill=None, color='red')
    plt.figure()
    ax = plt.subplot(121)
    ax.imshow(polygon_map[:, :])
    ax.set_xticks(np.arange(0, 20) - 0.5)
    ax.set_yticks(np.arange(0, 20) - 0.5)
    ax.grid(True)
    centers = np.array(np.meshgrid(np.arange(0, 20), np.arange(0, 20))).reshape(2, 20 * 20)
    ax.scatter(centers[0, :], centers[1, :])
    ax.add_patch(polygon)

    ax = plt.subplot(122)
    ax.imshow(quadrangle_map[:, :])
    ax.set_xticks(np.arange(0, 20) - 0.5)
    ax.set_yticks(np.arange(0, 20) - 0.5)
    ax.grid(True)
    centers = np.array(np.meshgrid(np.arange(0, 20), np.arange(0, 20))).reshape(2, 20 * 20)
    ax.scatter(centers[0, :], centers[1, :])
    ax.add_patch(quadrangle)
    plt.show()
