//
// Created by stefan on 19.08.21.
//

#ifndef HECTOR_MATH_URDF_CONVERSIONS_HPP
#define HECTOR_MATH_URDF_CONVERSIONS_HPP

namespace hector_math
{
template<typename Scalar>
inline Vector3<Scalar> urdfToEigenVector( const urdf::Vector3 &vec )
{
  return { static_cast<Scalar>(vec.x), static_cast<Scalar>(vec.y), static_cast<Scalar>(vec.z) };
}

template<typename Scalar>
inline Eigen::Quaternion<Scalar> urdfToEigenQuaternion( const urdf::Rotation &rotation )
{
  return { static_cast<Scalar>(rotation.w), static_cast<Scalar>(rotation.x), static_cast<Scalar>(rotation.y),
           static_cast<Scalar>(rotation.z) };
}

template<typename Scalar>
inline Isometry3<Scalar> urdfPoseToEigenTransform( const urdf::Pose &pose )
{
  Isometry3<Scalar> child_transform = Isometry3<Scalar>::Identity();
  child_transform.translation() = urdfToEigenVector<Scalar>( pose.position );
  child_transform.linear() = urdfToEigenQuaternion<Scalar>( pose.rotation ).toRotationMatrix();
  return child_transform;
}
}

#endif //HECTOR_MATH_URDF_CONVERSIONS_HPP
