=====
Robot
=====

The Robot model class is a base class for storing the current state of the roboter.
For example,it keeps track of the joint positions and recomputes the center of mass every time the robot moves.
It also stores the footprint and mass of the robot.

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
