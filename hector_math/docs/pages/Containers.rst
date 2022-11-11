===========
Containers
===========

Bounded Vector
--------------
The bounded vector is a container structure that tries to combine the advantages of arrays and vectors.
It can be used if the maximum size is known at compile time. It combines the efficiency of arrays with the syntax of vectors.

Quaternion Cache
----------------
This container can be used to store direction-dependent data. To store the data efficiently, the directions are discretized.
Hector_math implements three modes: `Spherical`, `Largest dim` and `Spherical Fibonacci`.
The level of discretization, how many bins there, are can selected as well as the discretization mode.
The modes have different advantages and disadvantages:

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

It is impossible to achieve perfect evenly discretization for arbitrary many bins.
The spherical fibonacci mode achieves the best discretization. The spherical mode
is differently distributed near the poles and the equator while the largest dim
mode doesn't have a reduced resolution at the poles but at the cost of overlapping
regions with increased bin resolution.

For more details, see :doc:`QuaternionBinning`.

API
---
.. doxygenclass:: hector_math::BoundedVector
   :members:
   :private-members:
   :undoc-members:

.. doxygenclass:: hector_math::QuaternionCache
   :members:
   :private-members:
   :undoc-members:
