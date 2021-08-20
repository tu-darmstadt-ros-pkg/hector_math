//
// Created by stefan on 19.08.21.
//

#ifndef HECTOR_MATH_URDF_ROBOT_MODEL_IMPL_HPP
#define HECTOR_MATH_URDF_ROBOT_MODEL_IMPL_HPP

#include <hector_math/robot/urdf_robot_model.h>

#include "hector_math/robot/urdf_conversions.hpp"

namespace hector_math
{
template<typename Scalar>
Isometry3<Scalar> UrdfRobotModel<Scalar>::transformForJoint( const Joint &joint_info,
                                                             const std::vector<Scalar> &joint_positions ) const
{
  Isometry3<Scalar> child_transform = joint_info.parent_to_joint_transform;
  Scalar joint_value = joint_positions[joint_info.joint_state_index];
  if ( joint_info.is_mimic )
    joint_value = joint_info.mimic_multiplier * joint_value + joint_info.mimic_offset;
  switch ( joint_info.type )
  {
    case urdf::Joint::CONTINUOUS:
    case urdf::Joint::REVOLUTE:
      return child_transform * Eigen::AngleAxis<Scalar>( joint_value, joint_info.axis );
    case urdf::Joint::PRISMATIC:
      child_transform.translation() += child_transform.linear() * joint_value * joint_info.axis;
      return child_transform;
    case urdf::Joint::FIXED:
      return child_transform;
    case urdf::Joint::FLOATING:
    case urdf::Joint::UNKNOWN:
    case urdf::Joint::PLANAR:
      ROS_WARN_ONCE( "Unsupported joint type in transform for joint. This message is only displayed once." );
  }
  return child_transform;
}

template<typename Scalar>
Scalar UrdfRobotModel<Scalar>::computeWeightedCenterOfMass( const LinkTree &root, const Isometry3<Scalar> &transform,
                                                            const std::vector<Scalar> &joint_positions,
                                                            Vector3<Scalar> &com ) const
{
  Scalar sum_of_mass = 0;
  if ( root.inertial_mass > 0 )
  {
    com += transform * root.inertial_origin * root.inertial_mass;
    sum_of_mass = root.inertial_mass;
  }

  for ( const Joint &joint : root.children )
  {
    Isometry3<Scalar> child_transform = transformForJoint( joint, joint_positions );
    sum_of_mass += computeWeightedCenterOfMass( joint.child_link, transform * child_transform, joint_positions,
                                                com );
  }
  return sum_of_mass;
}

template<typename Scalar>
Vector3<Scalar> UrdfRobotModel<Scalar>::computeCenterOfMass() const
{
  Vector3<Scalar> com = Vector3<Scalar>::Zero();
  Scalar sum_of_mass = computeWeightedCenterOfMass( link_tree_, Isometry3<Scalar>::Identity(),
                                                    this->joint_positions_, com );
  if ( std::abs( sum_of_mass ) < 1E-8 ) return com;
  com /= sum_of_mass;
  return com;
}

template<typename Scalar>
void UrdfRobotModel<Scalar>::onJointStatesUpdated()
{
}

template<typename Scalar>
typename UrdfRobotModel<Scalar>::LinkTree UrdfRobotModel<Scalar>::buildLinkTree( const urdf::LinkSharedPtr &root )
{
  LinkTree result;
  result.inertial_mass = root->inertial != nullptr ? root->inertial->mass : 0;
  if ( result.inertial_mass > 0 ) result.inertial_origin = urdfToEigenVector<Scalar>( root->inertial->origin.position );
  for ( const urdf::JointSharedPtr &joint : root->child_joints )
  {
    for ( const urdf::LinkSharedPtr &link : root->child_links )
    {
      if ( joint->child_link_name != link->name ) continue;
      bool mimic = joint->mimic != nullptr && !joint->mimic->joint_name.empty();
      const std::string &name = mimic ? joint->mimic->joint_name : joint->name;
      auto it = std::find( this->joint_names_.begin(), this->joint_names_.end(), name );
      if ( it == this->joint_names_.end())
      {
        this->joint_names_.push_back( name );
        this->joint_positions_.push_back( 0 );
      }
      result.children.push_back( {
                                   .child_link = buildLinkTree( link ),
                                   .parent_to_joint_transform = urdfPoseToEigenTransform<Scalar>(
                                     joint->parent_to_joint_origin_transform ),
                                   .axis = urdfToEigenVector<Scalar>( joint->axis ),
                                   .joint_state_index = static_cast<size_t>(it - this->joint_names_.begin()),
                                   .mimic_multiplier = mimic ? joint->mimic->multiplier == 0 ? 1.0
                                                                                             : joint->mimic->multiplier
                                                             : 0,
                                   .mimic_offset = mimic ? joint->mimic->offset : 0,
                                   .type = joint->type,
                                   .is_mimic = mimic
                                 } );
      break;
    }
  }
  return result;
}
}

#endif //HECTOR_MATH_URDF_ROBOT_MODEL_IMPL_HPP
