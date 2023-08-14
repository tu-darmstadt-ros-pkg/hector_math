=====
Robot
=====

The Robot model class is a base class for storing the current state of the roboter.
For example,it keeps track of the joint positions and recomputes the center of mass every time the robot moves.
It also stores the footprint and mass of the robot.

The :cpp:class:`JointStateSubscriber <hector_math::JointStateSubscriber>` in `hector_math_ros` can be used to keep the model in sync with an actual or simulated robot.

**Example**

.. code-block:: cpp

  auto robot_model = std::make_shared<hector_math::UrdfRobotModel<double>>( urdf_model );
  hector_math::JointStateSubscriber sub( robot_model );
  if ( !sub.waitForFullState( ros::Duration( 3 )) {
    // Didn't get the state for all joints within 3 seconds
  }

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

.. doxygenclass:: hector_math::JointStateSubscriber
   :members:
