=======================
Shapes - Bounding Boxes
=======================

For given shapes, these functions compute axis-aligned bounding boxes.
Shapes include spheres, cylinders, and cuboids.
Furthermore, they can have any orientation.
The functions find the smallest box in which the specified shape fits completely.

API
---

.. doxygenfunction:: hector_math::computeBoundingBoxForSphere

.. doxygenfunction:: hector_math::computeBoundingBoxForBox

.. doxygenfunction:: hector_math::computeBoundingBoxForCylinder

.. doxygenfunction:: hector_math::transformBoundingBox