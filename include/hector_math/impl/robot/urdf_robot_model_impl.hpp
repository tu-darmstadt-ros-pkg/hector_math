//
// Created by stefan on 19.08.21.
//

#ifndef HECTOR_MATH_URDF_ROBOT_MODEL_IMPL_HPP
#define HECTOR_MATH_URDF_ROBOT_MODEL_IMPL_HPP

#include "hector_math/robot/urdf_robot_model.h"
#include "hector_math/robot/urdf_conversions.hpp"
#include "hector_math/shapes/bounding_box.h"

namespace hector_math
{
template<typename Scalar>
Isometry3<Scalar> UrdfRobotModel<Scalar>::transformForJoint( const Joint &joint_info ) const
{
  Isometry3<Scalar> child_transform = joint_info.parent_to_joint_transform;
  Scalar joint_value = this->joint_positions_[joint_info.joint_state_index];
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
                                                            Vector3<Scalar> &com ) const
{
  Scalar sum_of_mass = 0;
  if ( root.inertial_mass > 0 )
  {
    com += transform * root.inertial_origin * root.inertial_mass;
    sum_of_mass = root.inertial_mass;
  }

  for ( const Joint &joint: root.children )
  {
    Isometry3<Scalar> child_transform = transformForJoint( joint );
    sum_of_mass += computeWeightedCenterOfMass( joint.child_link, transform * child_transform, com );
  }
  return sum_of_mass;
}

template<typename Scalar>
Vector3<Scalar> UrdfRobotModel<Scalar>::computeCenterOfMass() const
{
  Vector3<Scalar> com = Vector3<Scalar>::Zero();
  Scalar sum_of_mass = computeWeightedCenterOfMass( link_tree_, Isometry3<Scalar>::Identity(), com );
  if ( std::abs( sum_of_mass ) < 1E-8 ) return com;
  com /= sum_of_mass;
  return com;
}

// **************************************************
// ****************** BOUNDING BOX ******************

template<typename Scalar>
Eigen::AlignedBox<Scalar, 3>
UrdfRobotModel<Scalar>::computeAxisAlignedBoundingBox( const LinkTree &root, const Isometry3<Scalar> &transform ) const
{
  Eigen::AlignedBox<Scalar, 3> result;
  for ( const LinkGeometry &shape: root.geometries )
  {
    switch ( shape.type )
    {
      case urdf::Geometry::BOX:
        result.extend( computeBoundingBoxForBox<Scalar>( shape.dims, transform ));
        break;
      case urdf::Geometry::CYLINDER:
        result.extend( computeBoundingBoxForCylinder<Scalar>( shape.dims.x(), shape.dims.z(), transform ));
        break;
      case urdf::Geometry::SPHERE:
        result.extend( computeBoundingBoxForSphere<Scalar>( shape.dims.x(), transform ));
        break;
      case urdf::Geometry::MESH:
        // TODO Adding support for meshes would be cool I guess
        break;
    }
  }

  for ( const Joint &joint: root.children )
  {
    const Isometry3<Scalar> &child_transform = transformForJoint( joint );
    result.extend( computeAxisAlignedBoundingBox( joint.child_link, transform * child_transform ));
  }
  return result;
}

template<typename Scalar>
Eigen::AlignedBox<Scalar, 3> UrdfRobotModel<Scalar>::computeAxisAlignedBoundingBox() const
{
  return computeAxisAlignedBoundingBox( link_tree_, Isometry3<Scalar>::Identity());
}

template<typename Scalar>
void UrdfRobotModel<Scalar>::onJointStatesUpdated()
{
}

namespace impl
{
template<typename Scalar>
Vector3<Scalar> dimsFromGeometry( const urdf::Geometry *geometry )
{
  switch ( geometry->type )
  {
    case urdf::Geometry::BOX:
    {
      auto box = dynamic_cast<const urdf::Box *>(geometry);
      if ( box == nullptr ) break;
      return urdfToEigenVector<Scalar>( box->dim );
    }
    case urdf::Geometry::CYLINDER:
    {
      auto cylinder = dynamic_cast<const urdf::Cylinder *>(geometry);
      if ( cylinder == nullptr ) break;
      return Vector3<Scalar>( cylinder->radius, cylinder->radius, cylinder->length );
    }
    case urdf::Geometry::SPHERE:
    {
      auto sphere = dynamic_cast<const urdf::Sphere *>(geometry);
      if ( sphere == nullptr ) break;
      return Vector3<Scalar>( sphere->radius, sphere->radius, sphere->radius );
    }
    case urdf::Geometry::MESH:
      auto mesh = dynamic_cast<const urdf::Mesh *>(geometry);
      if ( mesh == nullptr ) break;
      // TODO Adding support for meshes would be cool I guess
  }
  return Vector3<Scalar>::Zero();
}
}

template<typename Scalar>
typename UrdfRobotModel<Scalar>::LinkTree UrdfRobotModel<Scalar>::buildLinkTree( const urdf::LinkSharedPtr &root )
{
  LinkTree result;
  result.inertial_mass = root->inertial != nullptr ? root->inertial->mass : 0;
  if ( result.inertial_mass > 0 ) result.inertial_origin = urdfToEigenVector<Scalar>( root->inertial->origin.position );

  // Collision information
  auto collision_array = root->collision_array.empty() ? std::vector<urdf::CollisionSharedPtr>( 1, root->collision )
                                                       : root->collision_array;
  for ( const urdf::CollisionSharedPtr &shape: collision_array )
  {
    if ( shape == nullptr ) continue;
    LinkGeometry geometry;
    geometry.type = shape->geometry->type;
    geometry.origin = urdfPoseToEigenTransform<Scalar>( shape->origin );
    geometry.dims = impl::dimsFromGeometry<Scalar>( shape->geometry.get());
    result.geometries.push_back( geometry );
  }
  if ( result.geometries.empty() && settings_.fallback_on_visual )
  {
    auto visual_array = root->visual_array.empty() ? std::vector<urdf::VisualSharedPtr>( 1, root->visual )
                                                   : root->visual_array;
    for ( const urdf::VisualSharedPtr &shape: visual_array )
    {
      if ( shape == nullptr ) continue;
      LinkGeometry geometry;
      geometry.type = shape->geometry->type;
      geometry.origin = urdfPoseToEigenTransform<Scalar>( shape->origin );
      geometry.dims = impl::dimsFromGeometry<Scalar>( shape->geometry.get());
      result.geometries.push_back( geometry );
    }
  }

  for ( const urdf::JointSharedPtr &joint: root->child_joints )
  {
    for ( const urdf::LinkSharedPtr &link: root->child_links )
    {
      if ( joint->child_link_name != link->name ) continue;
      bool mimic = joint->mimic != nullptr && !joint->mimic->joint_name.empty();
      const std::string &name = mimic ? joint->mimic->joint_name : joint->name;
      auto it = std::find( this->joint_names_.begin(), this->joint_names_.end(), name );
      size_t index = static_cast<size_t>(it - this->joint_names_.begin());
      if ( it == this->joint_names_.end())
      {
        index = this->joint_names_.size();
        this->joint_names_.push_back( name );
        this->joint_positions_.push_back( 0 );
      }
      Joint child;
      child.type = joint->type;
      child.joint_state_index = index;
      child.axis = urdfToEigenVector<Scalar>( joint->axis );
      child.parent_to_joint_transform = urdfPoseToEigenTransform<Scalar>( joint->parent_to_joint_origin_transform );
      child.mimic_multiplier = !mimic ? 0
                                      : joint->mimic->multiplier == 0 ? 1.0
                                                                      : joint->mimic->multiplier;
      child.mimic_offset = mimic ? joint->mimic->offset : 0;
      child.is_mimic = mimic;
      child.child_link = buildLinkTree( link );
      result.children.push_back( child );
      break;
    }
  }
  return result;
}
}

#endif //HECTOR_MATH_URDF_ROBOT_MODEL_IMPL_HPP
