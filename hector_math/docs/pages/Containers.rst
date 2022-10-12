===========
Containers
===========

Bounded Vector
--------------
The bounded vector is an array like container as it has a maximum size.
It does, however, use the same syntax as std::vector.
As a result, if the maximum size is known, it can be used as a drop-in replacement for std::vectors.

Quaternion Cache
----------------
The Quaternion Cache allows to store direction-dependent data. Therefore, the continuous space of
possible directions is discretized.
There are 2 different modes for discretizing the sphere surface (all possible directions lie
on the unit sphere).
In general, it is impossible to discretize the sphere with n points such that all points are evenly
spaced. To store and retrieve direction-dependent data, there must be a mapping between directions
and unique indices.
The Quaternion cache can be used in two modes: `Spherical` and `Largest dim`.
The Spherical mode uses polar coordinates to compute :math:`\theta` and :math:`\phi`.

.. math::

   x& = \cos(\phi) \sin(\theta)\\
   y& = \sin(\phi) \sin(\theta)\\
   z& = \cos(\theta)

Both angles are evenly discretized.

The Largest Dim Mode starts by evaluating which of the :math:`x`-, :math:`y`- or :math:`z`-component of the quaternion
is the largest. Then it stores discretized representations of the two remaining components. Lastly,
it discretices the :math:`w` component of the quaternion.

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
