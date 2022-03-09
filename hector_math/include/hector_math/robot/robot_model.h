//
// Created by Stefan Fabian on 17.08.21.
//

#ifndef HECTOR_MATH_ROBOT_MODEL_H
#define HECTOR_MATH_ROBOT_MODEL_H

#include "hector_math/types.h"
#include <memory>
#include <unordered_map>
#include <vector>

namespace hector_math
{

template<typename Scalar>
class RobotModel
{
public:
  using Ptr = std::shared_ptr<RobotModel<Scalar>>;
  using ConstPtr = std::shared_ptr<const RobotModel<Scalar>>;

  RobotModel( std::vector<std::string> joint_names, std::vector<Scalar> joint_positions )
      : joint_names_( std::move( joint_names ) ), joint_positions_( std::move( joint_positions ) )
  {
    // Ensure joint positions has the same length as joint names
    joint_positions.resize( joint_names_.size(), 0 );
  }

  explicit RobotModel( std::unordered_map<std::string, Scalar> joint_states )
  {
    joint_names_.reserve( joint_states.size() );
    joint_positions_.reserve( joint_states.size() );
    for ( const auto &it : joint_states ) {
      joint_names_.push_back( it.first );
      joint_positions_.push_back( it.second );
    }
  }

  //! Updates the joint positions of the robot model with the given values for the corresponding
  //! joint name.
  virtual void updateJointPositions( const std::unordered_map<std::string, Scalar> &positions )
  {
    for ( size_t i = 0; i < joint_names_.size(); ++i ) {
      auto it = positions.find( joint_names_[i] );
      if ( it == positions.end() )
        continue;
      joint_positions_[i] = it->second;
    }
    onJointStatesUpdated();
  }

  //! Updates the joint positions of the robot model with the given values.
  //! Indices have to correspond to names in jointNames().
  virtual void updateJointPositions( const std::vector<Scalar> &positions )
  {
    if ( positions.size() != joint_positions_.size() ) {
      throw std::invalid_argument( "RobotModel::updateJointPositions(): Expected array of size " +
                                   std::to_string( joint_positions_.size() ) +
                                   " but received size " + std::to_string( positions.size() ) );
    }
    std::copy( positions.begin(), positions.end(), joint_positions_.begin() );
    onJointStatesUpdated();
  }

  //! The names of the joints represented in this robot model.
  std::vector<std::string> jointNames() const { return joint_names_; }

  std::vector<Scalar> jointPositions() const { return joint_positions_; }

  Scalar getJointPosition( const std::string &name ) const
  {
    for ( size_t i = 0; i < joint_names_.size(); ++i ) {
      if ( joint_names_[i] != name )
        continue;
      return joint_positions_[i];
    }
    return std::numeric_limits<Scalar>::quiet_NaN();
  }

  //! The position of the center of mass in the robot coordinate frame.
  Vector3<Scalar> centerOfMass() const
  {
    if ( center_of_mass_valid_ )
      return center_of_mass_;
    center_of_mass_ = computeCenterOfMass();
    center_of_mass_valid_ = true;
    return center_of_mass_;
  }

  Eigen::AlignedBox<Scalar, 3> axisAlignedBoundingBox() const
  {
    if ( axis_aligned_bounding_box_valid_ )
      return axis_aligned_bounding_box_;
    axis_aligned_bounding_box_ = computeAxisAlignedBoundingBox();
    axis_aligned_bounding_box_valid_ = true;
    return axis_aligned_bounding_box_;
  }

  //! The footprint is an approximated polygon of the robots hull projected to the ground.
  Polygon<Scalar> footprint() const
  {
    if ( footprint_valid_ )
      return footprint_;
    footprint_ = computeFootprint();
    footprint_valid_ = true;
    return footprint_;
  }

protected:
  //! Use to invalidate footprint and center of mass.
  //! IMPORTANT: Call the base implementation.
  virtual void onJointStatesUpdated()
  {
    center_of_mass_valid_ = false;
    axis_aligned_bounding_box_valid_ = false;
    footprint_valid_ = false;
  }

  virtual Vector3<Scalar> computeCenterOfMass() const = 0;

  virtual Polygon<Scalar> computeFootprint() const = 0;

  virtual Eigen::AlignedBox<Scalar, 3> computeAxisAlignedBoundingBox() const = 0;

  std::vector<std::string> joint_names_;
  std::vector<Scalar> joint_positions_;
  mutable Polygon<Scalar> footprint_;
  mutable Eigen::AlignedBox<Scalar, 3> axis_aligned_bounding_box_;
  mutable Vector3<Scalar> center_of_mass_;
  mutable bool footprint_valid_ = false;
  mutable bool axis_aligned_bounding_box_valid_ = false;
  mutable bool center_of_mass_valid_ = false;
};
} // namespace hector_math

#endif // HECTOR_MATH_ROBOT_MODEL_H
