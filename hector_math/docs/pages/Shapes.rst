=======================
Shapes - Bounding Boxes
=======================

For given shapes, these functions compute axis-aligned bounding boxes.
Shapes include spheres, cylinders, and cuboids.
Furthermore, they can have any orientation.
The functions find the smallest box in which the specified shape fits completely.

Sphere
------

.. doxygenfunction:: hector_math::computeBoundingBoxForSphere( Scalar radius, const Isometry3<Scalar> &transform )

Box
---

.. doxygenfunction:: hector_math::computeBoundingBoxForBox

Cylinder
--------

.. doxygenfunction:: hector_math::computeBoundingBoxForCylinder

Transform
---------

.. doxygenfunction:: hector_math::transformBoundingBox