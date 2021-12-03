//
// Created by stefan on 01.12.21.
//

#ifndef HECTOR_MATH_ROS_ROBOT_MODEL_H
#define HECTOR_MATH_ROS_ROBOT_MODEL_H

// You will need to depend on moveit and moveit_msgs if you use this header.

#include "hector_math/robot/robot_model.h"
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayRobotState.h>

namespace hector_math
{

template<typename Scalar>
inline moveit::core::RobotState
robotModelToRobotState( const RobotModel<Scalar> &model, const urdf::ModelSharedPtr &urdf,
                        const srdf::ModelSharedPtr &srdf = std::make_shared<srdf::Model>())
{
  moveit::core::RobotModelPtr robot_model = std::make_shared<moveit::core::RobotModel>( urdf, srdf );
  moveit::core::RobotState state( robot_model );
  const auto &names = model.getJointNames();
  const auto &positions = model.getJointPositions();
  const auto &variable_names = state.getVariableNames();
  for ( int i = 0; i < names.size(); ++i )
  {
    if ( std::find( variable_names.begin(), variable_names.end(), names[i] ) == variable_names.end()) continue;
    state.setVariablePosition( names[i], positions[i] );
  }
  state.update();
  return state;
}
}  // namespace hector_pose_prediction_interface

#endif // HECTOR_MATH_ROS_ROBOT_MODEL_H