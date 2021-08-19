//
// Created by stefan on 19.08.21.
//

#ifndef HECTOR_MATH_URDF_ROBOT_MODEL_IMPL_HPP
#define HECTOR_MATH_URDF_ROBOT_MODEL_IMPL_HPP

#include "hector_math/robot/urdf_conversions.hpp"

namespace hector_math
{
namespace impl
{
template<typename Scalar>
inline Isometry3<Scalar> transformForJoint( const urdf::Joint &joint,
                                            const std::vector<std::string> &joint_names,
                                            const std::vector<Scalar> &joint_positions )
{
  Isometry3<Scalar> child_transform =
    urdfPoseToEigenTransform<Scalar>( joint.parent_to_joint_origin_transform );
  bool mimic = joint.mimic != nullptr && !joint.mimic->joint_name.empty();
  auto it = mimic ? std::find( joint_names.begin(), joint_names.end(), joint.mimic->joint_name )
                  : std::find( joint_names.begin(), joint_names.end(), joint.name );
  if ( it == joint_names.end()) return child_transform;
  Scalar joint_value = joint_positions[it - joint_names.begin()];
  if ( mimic )
    joint_value = (joint.mimic->multiplier == 0 ? 1.0 : joint.mimic->multiplier) * joint_value + joint.mimic->offset;
  switch ( joint.type )
  {
    case urdf::Joint::CONTINUOUS:
    case urdf::Joint::REVOLUTE:
      return child_transform * Eigen::AngleAxis<Scalar>( joint_value, urdfToEigenVector<Scalar>( joint.axis ));
    case urdf::Joint::PRISMATIC:
      child_transform.translation() += child_transform.linear() * joint_value * urdfToEigenVector<Scalar>( joint.axis );
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
inline Scalar computeWeightedCenterOfMass( const urdf::Link &root, const Isometry3<Scalar> &transform,
                                           const std::vector<std::string> &joint_names,
                                           const std::vector<Scalar> &joint_positions,
                                           Vector3<Scalar> &com )
{
  Scalar sum_of_mass = 0;
  if ( root.inertial != nullptr && root.inertial->mass > 0 )
  {
    const auto &pos = root.inertial->origin.position;
    com += transform * Vector3<Scalar>( pos.x, pos.y, pos.z ) * root.inertial->mass;
    sum_of_mass = root.inertial->mass;
  }

  for ( const urdf::JointSharedPtr &joint : root.child_joints )
  {
    for ( const urdf::LinkSharedPtr &link : root.child_links )
    {
      if ( joint->child_link_name != link->name ) continue;
      Isometry3<Scalar> child_transform = transformForJoint( *joint, joint_names, joint_positions );
      sum_of_mass += computeWeightedCenterOfMass( *link, transform * child_transform, joint_names, joint_positions, com );
      break;
    }
  }
  return sum_of_mass;
}
}

template<typename Scalar>
Vector3<Scalar> UrdfRobotModel<Scalar>::computeCenterOfMass() const
{
  Vector3<Scalar> com = Vector3<Scalar>::Zero();
  Scalar sum_of_mass = impl::computeWeightedCenterOfMass( *model_.root_link_, Isometry3<Scalar>::Identity(),
                                                          this->joint_names_, this->joint_positions_,
                                                          com );
  if ( std::abs( sum_of_mass ) < 1E-8 ) return com;
  com /= sum_of_mass;
  return com;
}
}

#endif //HECTOR_MATH_URDF_ROBOT_MODEL_IMPL_HPP
