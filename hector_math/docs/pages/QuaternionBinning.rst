==================
Quaternion Binning
==================

Quaternion Binning can be used to discretize directions. This can be interpreted as discretizing the surface of a sphere.
In general, it is impossible to discretize the sphere with `n` points such that all points are evenly
spaced for an arbitrary 'n'. There must be a mapping between directions
and unique indices to store and retrieve direction-dependent data.

Hector_math implements three modes: `Spherical`, `Largest dim` and .`Spherical Fibonacci`.
They differ in how evenly they discretize the sphere surface and how fast they compute the mapping from a direction to its index.

.. list-table:: Discretization Mode Comparison
   :widths: 15 15 15
   :header-rows: 1
   :align: center

   * - Mode
     - Speed
     - Evenly Discretization
   * - Spherical
     - 🟡
     - ❌
   * - Largest Dimension
     - ✅
     - 🟡
   * - Spherical Fibonacci
     - ❌
     - ✅

Spherical
*********
The Spherical mode uses polar coordinates to compute :math:`\theta` and :math:`\phi`.

.. math::

   x& = \cos(\phi) \sin(\theta)\\
   y& = \sin(\phi) \sin(\theta)\\
   z& = \cos(\theta)

Both angles are evenly discretized.
The discretization can be computed medium fast however it is not evenly. There are more bins at the poles and less at the equator.

.. figure:: img/spherical.svg
   :width: 75%
   :alt: Spherical Discretization.
   :align: center

Largest Dim
***********

The Largest Dim Mode starts by evaluating which of the :math:`x`-, :math:`y`- or :math:`z`-component of the quaternion
is the largest. Then it stores discretized representations of the two remaining components. Lastly,
it discretices the :math:`w` component of the quaternion.

.. figure:: img/largest_dim.svg
   :width: 75%
   :alt: Largest dimension Discretization.
   :align: center


The largest dimension mode is somewhat faster and doesn't have a reduced resolution at the poles
due to the equidistant angles used in spherical but at the cost of overlapping regions with
increased bin resolution. Therefore the bin distribution is much higher in some areas of the
sphere surface than in others.

Spherical Fibonacci
*******************

.. figure:: img/spherical_fibonacci.svg
   :width: 75%
   :alt: Spherical Fibonacci dimension Discretization.
   :align: center

Spherical Fibonacci point sets yield nearly uniform point distributions on the unit sphere.
The forward generation of these point sets has been widely researched and is easy to implement,
such that they have been used in various applications. The Fibonacci lattice's points are
arranged in a tightly wound generative spiral, each fitting into the smallest gap between
the previous points.
Because the consecutive points are so far apart, the spiral is normally not visible.
The points are evenly spaced in a very isotropic way. Compared to the other modes implemented
here, it is the slowest but achieves the best discretization accuracy. For more details on how
to compute the spherical fibonacci mapping see `Spherical Fibonacci Mapping <1_>`_.

.. _1: https://dl.acm.org/doi/10.1145/2816795.2818131

.. figure:: img/spherical_fibonacci_spiral.svg
   :width: 75%
   :alt: Spherical Fibonacci dimension Discretization.
   :align: center

   A visualization of the spherical fibonacci mapping,
   also showing the generative spiral.




