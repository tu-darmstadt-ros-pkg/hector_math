=====
Robot
=====

The Robot model class is a base class for storing and calculating the center of mass of a moving robot.
Whenever the robot moves, the representation of the joint angles is updated. However, this invalidates
the last computation of the center of mass. Hence, it has to be recomputed given the new joint positions.

API
---

.. doxygenclass:: hector_math::RobotModel
   :members:
   :private-members:
   :undoc-members:

.. doxygenclass:: hector_math::RobotFootprint
   :members:
   :private-members:
   :undoc-members:
