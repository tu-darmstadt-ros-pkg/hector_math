
![workflow](https://github.com/tu-darmstadt-ros-pkg/hector_math/actions/workflows/ci.yml/badge.svg)
![workflow](https://github.com/tu-darmstadt-ros-pkg/hector_math/actions/workflows/documentation.yml/badge.svg)
# Hector Math
The hector_math ROS package provides a set of implementations for common math-related problems encountered in robotic
applications.

### Robot Model

Hector_math includes a set of convenience functions for constructing a robot model, continuously updating its joint
positions, and computing the center of mass and estimated footprint of the robot.
More details can be found [here](https://tu-darmstadt-ros-pkg.github.io/hector_math/en/master/pages/Robot.html).

### 2D Map Operations

Hector_math also includes a set of tools for performing operations on 2D maps.

* finding minima/maxima on the map
* iterating over all positions inside a given polygon

More details can be found [here](https://tu-darmstadt-ros-pkg.github.io/hector_math/en/master/pages/MapOperations.html).

### Direction Discretization

The package includes direction discretization tools for quickly converting continuous angles into discrete
directions. More details can be found
[here](https://tu-darmstadt-ros-pkg.github.io/hector_math/en/master/pages/QuaternionBinning.html).

### Containers

Hector_math also includes several containers that are useful for robotics applications.

* BoundedVector: vector-like structure but with limited size
* QuaternionCache: stores direction-dependent data
* RingBuffer: as a FIFO structure with a limited size (overwriting the oldest elements if the buffer is full)

More details can be found [here](https://tu-darmstadt-ros-pkg.github.io/hector_math/en/master/pages/Containers.html).

### Bounding Boxes

Functions for computing axis-aligned bounding boxes for various shapes.
More details can be found [here](https://tu-darmstadt-ros-pkg.github.io/hector_math/en/master/pages/Shapes.html).

# Installation

Clone this repo and its dependencies into the src directory of your ROS workspace.

```bash
cd <WORKSPACE>/src
git clone https://github.com/tu-darmstadt-ros-pkg/hector_math.git
catkin build
```

# Documentation

You can find the documentation [here](https://tu-darmstadt-ros-pkg.github.io/hector_math/en/master/).

Alternatively, you can follow the steps below to build it yourself.

#### Dependencies

* Doxygen
* Sphinx
* sphinx_rtd_theme
* Breathe

**Example for Ubuntu**  
Install dependencies

```bash
sudo apt install doxygen
pip3 install sphinx sphinx_rtd_theme breathe
```

#### Build documentation

```bash
cd hector_math/hector_math/docs
make html
sensible-browser _build/html/index.html
```

# License

This package is released under the MIT License. See the LICENSE file for more information.

# Contributing

Contributions to hector_math are welcome! If you find a bug or have a suggestion for a new feature, please open an issue
or submit a pull request on our GitHub repository.
