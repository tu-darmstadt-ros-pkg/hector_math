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
  using Ptr = std::shared_ptr<RobotModel<Scalar> >;
  using ConstPtr = std::shared_ptr<const RobotModel<Scalar> >;

  RobotModel( std::vector<std::string> joint_names, std::vector<Scalar> joint_positions )
    : joint_names_( std::move( joint_names )), joint_positions_( std::move( joint_positions ))
  {
    // Ensure joint positions has the same length as joint names
    joint_positions.resize( joint_names_.size(), 0 );
  }

  explicit RobotModel( std::unordered_map<std::string, Scalar> joint_states )
  {
    joint_names_.reserve( joint_states.size());
    joint_positions_.reserve( joint_states.size());
    for ( const auto &it : joint_states )
    {
      joint_names_.push_back( it.first );
      joint_positions_.push_back( it.second );
    }
  }

  //! Updates the joint positions of the robot model with the given values for the corresponding joint name.
  virtual void updateJointPositions( const std::unordered_map<std::string, Scalar> &positions )
  {
    for ( size_t i = 0; i < joint_names_.size(); ++i )
    {
      auto it = positions.find( joint_names_[i] );
      if ( it == positions.end()) continue;
      joint_positions_[i] = it->second;
    }
    center_of_mass_valid_ = false;
    onJointStatesUpdated();
  }

  //! Updates the joint positions of the robot model with the given values.
  //! Indices have to correspond to names in getJointNames().
  virtual void updateJointPositions( const std::vector<Scalar> &positions )
  {
    auto end = positions.size() > joint_positions_.size() ? positions.begin() + joint_positions_.size()
                                                          : positions.end();
    std::copy( positions.begin(), end, joint_positions_.begin());
    center_of_mass_valid_ = false;
    onJointStatesUpdated();
  }

  //! The names of the joints represented in this robot model.
  std::vector<std::string> getJointNames() const { return joint_names_; }

  std::vector<Scalar> getJointPositions() const { return joint_positions_; }

  //! The position of the center of mass in the robot coordinate frame.
  Vector3<Scalar> centerOfMass()
  {
    if ( center_of_mass_valid_ ) return center_of_mass_;
    center_of_mass_ = computeCenterOfMass();
    center_of_mass_valid_ = true;
    return center_of_mass_;
  }

  Polygon<Scalar> footprint()
  {
    if ( footprint_valid_ ) return footprint_;
    footprint_ = computeFootprint();
    footprint_valid_ = true;
    return footprint_;
  }

protected:
  //! Use to invalidate footprint and center of mass
  virtual void onJointStatesUpdated() = 0;

  virtual Vector3<Scalar> computeCenterOfMass() const = 0;

  virtual Polygon<Scalar> computeFootprint() const = 0;

  std::vector<std::string> joint_names_;
  std::vector<Scalar> joint_positions_;
  Polygon<Scalar> footprint_;
  Vector3<Scalar> center_of_mass_;
  bool footprint_valid_ = false;
  bool center_of_mass_valid_ = false;
};
}

#endif //HECTOR_MATH_ROBOT_MODEL_H
