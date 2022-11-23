#!/usr/bin/env python
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

AXIS_BINS = 32
SAMPLED_ANGLE_RESOLUTION = 10  # In degrees
SHOW_INPUT = False

AXIS_BINS_2 = AXIS_BINS / 2
DIM_BITS = int(np.log2(AXIS_BINS)) + 1

LARGEST_DIM = 0
SPHERICAL = 1
SPHERICAL_FIBONACCI = 2

fibonacci_n = 2 ** 12
PHI = (np.sqrt(5) + 1) / 2
# Set the binning mode here:
MODE = SPHERICAL_FIBONACCI
names = ["Largest dimension", "Spherical", "Spherical Fibonacci"]


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


def compute_bin_spherical(x, y, z):
    norm = np.sqrt(x ** 2 + y ** 2 + z ** 2)
    x /= norm
    y /= norm
    z /= norm
    theta = np.arccos(z)
    result = int(theta * AXIS_BINS / np.pi + 0.5)
    phi = np.arctan2(y, x)
    result |= int(phi * AXIS_BINS / np.pi + 0.5) << DIM_BITS
    return result


def inverse_bin_spherical(val):
    theta = (val & (2 ** DIM_BITS - 1)) * np.pi / AXIS_BINS
    phi = (val >> DIM_BITS) * np.pi / AXIS_BINS
    return np.cos(phi) * np.sin(theta), np.sin(phi) * np.sin(theta), np.cos(theta)


def inverse_sf(x, y, z):
    norm = np.sqrt(x ** 2 + y ** 2 + z ** 2)
    x /= norm
    y /= norm
    z /= norm
    p = np.array([x, y, z])

    def madfrac(a, b):
        return np.modf(a * b)[0]  # a * b - np.floor(a * b)

    phi = min(np.arctan2(y, x), np.pi)
    cos_theta = z
    k = max(2.0, np.floor(np.log(fibonacci_n * np.pi * np.sqrt(5) * (1.0 - cos_theta ** 2)) / np.log(PHI + 1)))
    fk = np.power(PHI, k) / np.sqrt(5)
    f0, f1 = round(fk), round(fk * PHI)
    b = np.array([[2 * np.pi * madfrac(f0 + 1, PHI - 1) - 2 * np.pi * (PHI - 1),
                   2 * np.pi * madfrac(f1 + 1, PHI - 1) - 2 * np.pi * (PHI - 1)],
                  [-2 * f0 / fibonacci_n, -2 * f1 / fibonacci_n]])
    inv_b = np.linalg.inv(b)
    c = np.floor(inv_b @ np.array([phi, cos_theta - (1 - 1.0 / fibonacci_n)]))
    d, j = np.inf, 0
    for s in range(4):
        cos_theta = np.dot(b[1, :], np.array([s % 2, s // 2]) + c) + (1 - 1 / fibonacci_n)
        cos_theta = np.clip(cos_theta, -1, 1) * 2 - cos_theta

        i = np.floor(fibonacci_n * 0.5 * (1 - cos_theta))
        phi = 2 * np.pi * madfrac(i, PHI - 1)
        cos_theta = 1.0 - (2.0 * i + 1.0) / fibonacci_n
        sin_theta = np.sqrt(1 - min(cos_theta ** 2, 1.0))

        q = np.array([np.cos(phi) * sin_theta, np.sin(phi) * sin_theta, cos_theta])
        squared_distance = np.dot(q - p, q - p)
        if squared_distance < d:
            d = squared_distance
            j = i
    return j


def forward_sf(i):
    phi = 2 * np.pi * (i / PHI - np.floor(i / PHI))
    cos_theta = 1 - (1 + 2 * i) / fibonacci_n
    sin_theta = np.sqrt(max(0, 1 - cos_theta ** 2))
    return np.cos(phi) * sin_theta, np.sin(phi) * sin_theta, cos_theta


if __name__ == '__main__':
    fig = plt.figure(figsize=(8, 8))  # quadratic figure
    ax = fig.add_subplot(111, projection='3d')
    input_vectors = np.array([(np.cos(phi) * np.sin(theta), np.sin(phi) * np.sin(theta), np.cos(theta))
                              for theta in np.arange(0, np.pi, SAMPLED_ANGLE_RESOLUTION * np.pi / 180)
                              for phi in np.arange(0, 2 * np.pi, SAMPLED_ANGLE_RESOLUTION * np.pi / 180)])
    bin_fn = compute_bin if MODE == LARGEST_DIM else inverse_sf if MODE == SPHERICAL_FIBONACCI else compute_bin_spherical
    inv_bin_fn = inverse_bin if MODE == LARGEST_DIM else forward_sf if MODE == SPHERICAL_FIBONACCI else inverse_bin_spherical
    bins = set([bin_fn(x, y, z) for x, y, z in input_vectors])
    vectors = np.array([inv_bin_fn(x) for x in bins]) if not SHOW_INPUT else input_vectors
    ax.scatter(vectors[:, 0], vectors[:, 1], vectors[:, 2])

    ax.axes.set_xlim3d(left=-1, right=1)
    ax.axes.set_ylim3d(bottom=-1, top=1)
    ax.axes.set_zlim3d(bottom=-1, top=1)
    ax.view_init(elev=20, azim=270)
    plt.title(names[MODE])
    plt.tight_layout()
plt.show()
