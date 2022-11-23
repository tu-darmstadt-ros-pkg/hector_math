#!/usr/bin/env python
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

AXIS_BINS = 32
SAMPLED_ANGLE_RESOLUTION = 1  # In degrees
SHOW_INPUT = False


def compute_required_bits(x):
    return x if x < 2 else 1 + compute_required_bits(x >> 1)


def compute_bitmask(n):
    return 0 if n == 0 else (compute_bitmask(n - 1) << 1) | 1


AXIS_BINS_2 = AXIS_BINS / 2
DIM_BITS = compute_required_bits(AXIS_BINS - 1)
DIM_MASK = compute_bitmask(DIM_BITS)


def compute_bin(x, y, z):
    norm = np.sqrt(x ** 2 + y ** 2 + z ** 2)
    x /= norm
    y /= norm
    z /= norm
    multiplier = np.sqrt(2) * AXIS_BINS_2
    offset = AXIS_BINS_2
    if abs(x) > abs(y) and abs(x) > abs(z):
        result = int(0b01)
        if x < 0:
            result |= 0b100
        result |= (int(y * multiplier + offset) & DIM_MASK) << 3
        result |= (int(z * multiplier + offset) & DIM_MASK) << (3 + DIM_BITS)
    elif abs(y) > abs(z):
        result = int(0b10)
        if y < 0:
            result |= 0b100
        result |= (int(x * multiplier + offset) & DIM_MASK) << 3
        result |= (int(z * multiplier + offset) & DIM_MASK) << (3 + DIM_BITS)
    else:
        result = int(0b11)
        if z < 0:
            result |= 0b100
        result |= (int(y * multiplier + offset) & DIM_MASK) << 3
        result |= (int(x * multiplier + offset) & DIM_MASK) << (3 + DIM_BITS)
    return result


def inverse_bin(val):
    multiplier = np.sqrt(2) * AXIS_BINS_2
    offset = AXIS_BINS_2
    a = (((val >> 3) & (2 ** DIM_BITS - 1)) - offset) / multiplier
    b = (((val >> (3 + DIM_BITS)) & (2 ** DIM_BITS - 1)) - offset) / multiplier
    c = np.sqrt(1 - a ** 2 - b ** 2)
    if (val & 0b100) == 0b100:
        c = -c
    if (val & 0b11) == 0b01:
        x, y, z = c, a, b
    elif (val & 0b11) == 0b10:
        x, y, z = a, c, b
    else:
        x, y, z = b, a, c
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
