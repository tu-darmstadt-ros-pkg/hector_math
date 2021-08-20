//
// Created by stefan on 19.08.21.
//

#ifndef HECTOR_MATH_URDF_CONVERSIONS_HPP
#define HECTOR_MATH_URDF_CONVERSIONS_HPP

#include "hector_math/types.h"

namespace hector_math
{
template<typename Scalar>
inline Vector3 <Scalar> urdfToEigenVector( const urdf::Vector3 &vec )
{
  return { Scalar( vec.x ), Scalar( vec.y ), Scalar( vec.z ) };
}

template<typename Scalar>
inline Eigen::Quaternion<Scalar> urdfToEigenQuaternion( const urdf::Rotation &rotation )
{
  return { Scalar( rotation.w ), Scalar( rotation.x ), Scalar( rotation.y ), Scalar( rotation.z ) };
}

template<typename Scalar>
inline Isometry3 <Scalar> urdfPoseToEigenTransform( const urdf::Pose &pose )
{
  Isometry3<Scalar> child_transform = Isometry3<Scalar>::Identity();
  child_transform.translation() = urdfToEigenVector<Scalar>( pose.position );
  child_transform.linear() = urdfToEigenQuaternion<Scalar>( pose.rotation ).toRotationMatrix();
  return child_transform;
}
}

#endif //HECTOR_MATH_URDF_CONVERSIONS_HPP
