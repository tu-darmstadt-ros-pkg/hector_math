===========
Containers
===========

Bounded Vector
--------------
The bounded vector is a container structure that tries to combine the advantages of arrays and vectors.
It can be used if the maximum size is known at compile time.
It combines the efficiency of arrays (e.g. locality) with the syntax of vectors (e.g. push_back).

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

Ring Buffer
-----------

The Ring Buffer is a container with a limited size which is controlled with the template argument MaxSize.
If the Ring Buffer is full, adding new elements will overwrite the oldest elements! It can be used as
a FIFO data structure. The oldest element can be easily retrieved while adding new elements will
append them to the container. The container keeps track of which element is the oldest and the newest
without ever moving any elements. Additionally, the container support any algorithm that work with
forward iterators, like std::for_each() and of course range loops.

API
---

Bounded Vector
**************
.. doxygenclass:: hector_math::BoundedVector
   :members:
   :private-members:
   :undoc-members:

Quaternion Cache
****************
.. doxygenclass:: hector_math::QuaternionCache
   :members:
   :private-members:
   :undoc-members:

Ring Buffer
***********
.. doxygenclass:: hector_math::RingBuffer
   :members:
   :private-members:
   :undoc-members:

