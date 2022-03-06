#!/usr/bin/env python
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

AXIS_BINS = 32
SAMPLED_ANGLE_RESOLUTION = 1  # In degrees
SHOW_INPUT = False

AXIS_BINS_2 = AXIS_BINS / 2
DIM_BITS = int(np.log2(AXIS_BINS)) + 1


def compute_bin(x, y, z):
    norm = np.sqrt(x ** 2 + y ** 2 + z ** 2)
    x /= norm
    y /= norm
    z /= norm
    if abs(x) > abs(y) and abs(x) > abs(z):
        result = int(0b01)
        if x < 0:
            result |= 0b100
        result |= int((1 + y * np.sqrt(2)) * AXIS_BINS_2 + 0.5) << 3
        result |= int((1 + z * np.sqrt(2)) * AXIS_BINS_2 + 0.5) << (3 + DIM_BITS)
    elif abs(y) > abs(z):
        result = int(0b10)
        if y < 0:
            result |= 0b100
        result |= int((1 + x * np.sqrt(2)) * AXIS_BINS_2 + 0.5) << 3
        result |= int((1 + z * np.sqrt(2)) * AXIS_BINS_2 + 0.5) << (3 + DIM_BITS)
    else:
        result = int(0b11)
        if z < 0:
            result |= 0b100
        result |= int((1 + y * np.sqrt(2)) * AXIS_BINS_2 + 0.5) << 3
        result |= int((1 + x * np.sqrt(2)) * AXIS_BINS_2 + 0.5) << (3 + DIM_BITS)
    return result


def inverse_bin(val):
    if (val & 0b11) == 0b01:
        y = ((((val >> 3) & (2 ** DIM_BITS - 1)) - 0.5) / AXIS_BINS_2 - 1) / np.sqrt(2)
        z = ((((val >> (3 + DIM_BITS)) & (2 ** DIM_BITS - 1)) - 0.5) / AXIS_BINS_2 - 1) / np.sqrt(2)
        x = np.sqrt(1 - y ** 2 - z ** 2)
        if (val & 0b100) == 0b100:
            x = -x
    elif (val & 0b11) == 0b10:
        x = ((((val >> 3) & (2 ** DIM_BITS - 1)) - 0.5) / AXIS_BINS_2 - 1) / np.sqrt(2)
        z = ((((val >> (3 + DIM_BITS)) & (2 ** DIM_BITS - 1)) - 0.5) / AXIS_BINS_2 - 1) / np.sqrt(2)
        y = np.sqrt(1 - x ** 2 - z ** 2)
        if (val & 0b100) == 0b100:
            y = -y
    else:
        y = ((((val >> 3) & (2 ** DIM_BITS - 1)) - 0.5) / AXIS_BINS_2 - 1) / np.sqrt(2)
        x = ((((val >> (3 + DIM_BITS)) & (2 ** DIM_BITS - 1)) - 0.5) / AXIS_BINS_2 - 1) / np.sqrt(2)
        z = np.sqrt(1 - y ** 2 - x ** 2)
        if (val & 0b100) == 0b100:
            z = -z
    return x, y, z


if __name__ == '__main__':
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    input_vectors = np.array([(np.cos(phi) * np.sin(theta), np.sin(phi) * np.sin(theta), np.cos(theta))
                              for theta in np.arange(0, np.pi, SAMPLED_ANGLE_RESOLUTION * np.pi / 180)
                              for phi in np.arange(0, 2 * np.pi, SAMPLED_ANGLE_RESOLUTION * np.pi / 180)])
    bins = set([compute_bin(x, y, z) for x, y, z in input_vectors])
    vectors = np.array([inverse_bin(x) for x in bins]) if not SHOW_INPUT else input_vectors
    ax.scatter(vectors[:, 0], vectors[:, 1], vectors[:, 2])
    plt.show()