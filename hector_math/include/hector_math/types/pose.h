// Copyright (c) 2023 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_POSE_H
#define HECTOR_MATH_POSE_H

#include "hector_math/types/pose2d.h"

namespace hector_math
{

template<typename Scalar>
class Pose;
template<typename Scalar>
using PoseList = std::vector<Pose<Scalar>, Eigen::aligned_allocator<Pose<Scalar>>>;

//! Represents the pose of a robot as a combination of orientation quaternion and 3D translation
//! vector.
template<typename Scalar>
class Pose
{
public:
  static Pose<Scalar> Identity()
  {
    Pose<Scalar> result;
    result.translation_ = Vector3<Scalar>::Zero();
    result.orientation_ = Eigen::Quaternion<Scalar>::Identity();
    return result;
  }

  static Pose<Scalar> Origin() { return Identity(); }

  static Pose<Scalar> Translation( Scalar x, Scalar y, Scalar z )
  {
    return Translation( Vector3<Scalar>( x, y, z ) );
  }

  static Pose<Scalar> Translation( const Vector3<Scalar> &translation )
  {
    Pose<Scalar> result;
    result.translation_ = translation;
    result.orientation_ = Eigen::Quaternion<Scalar>::Identity();
    return result;
  }

  Pose() = default;

  Pose( Scalar tx, Scalar ty, Scalar tz, Scalar qw, Scalar qx, Scalar qy, Scalar qz )
  {
    translation_ = { tx, ty, tz };
    orientation_ = Eigen::Quaternion<Scalar>( qw, qx, qy, qz ).normalized();
  }

  explicit Pose( const Isometry3<Scalar> &isometry )
  {
    translation_ = isometry.translation();
    orientation_ = Eigen::Quaternion<Scalar>( isometry.linear() ).normalized();
  }

  explicit Pose( const Eigen::Translation<Scalar, 3> &translation )
  {
    orientation_ = Eigen::Quaternion<Scalar>::Identity();
    translation_ = translation.vector();
  }

  template<typename Derived>
  explicit Pose( const Eigen::RotationBase<Derived, 3> &rotation )
  {
    orientation_ = Eigen::Quaternion<Scalar>( rotation.derived() ).normalized();
    translation_ = Vector3<Scalar>::Zero();
  }

  Pose( const Eigen::Translation<Scalar, 3> &translation,
        const Eigen::Quaternion<Scalar> &orientation )
  {
    orientation_ = orientation;
    translation_ = translation.vector();
  }

  template<typename Derived>
  Pose( const Eigen::Translation<Scalar, 3> &translation,
        const Eigen::RotationBase<Derived, 3> &rotation )
  {
    orientation_ = Eigen::Quaternion<Scalar>( rotation.derived() ).normalized();
    translation_ = translation.vector();
  }

  template<typename Derived>
  Pose( const Eigen::Translation<Scalar, 3> &translation, const Eigen::MatrixBase<Derived> &rotation )
  {
    orientation_ = Eigen::Quaternion<Scalar>( rotation.derived() ).normalized();
    translation_ = translation.vector();
  }

  Pose( const Vector3<Scalar> &translation,
        const Eigen::Quaternion<Scalar> &orientation = Eigen::Quaternion<Scalar>::Identity() )
  {
    orientation_ = orientation;
    translation_ = translation;
  }

  template<typename Derived>
  Pose( const Vector3<Scalar> &translation, const Eigen::RotationBase<Derived, 3> &rotation )
  {
    orientation_ = Eigen::Quaternion<Scalar>( rotation.derived() ).normalized();
    translation_ = translation;
  }

  template<typename Derived>
  Pose( const Vector3<Scalar> &translation, const Eigen::MatrixBase<Derived> &rotation )
  {
    orientation_ = Eigen::Quaternion<Scalar>( rotation.derived() ).normalized();
    translation_ = translation;
  }

  Vector3<Scalar> &translation() { return translation_; }

  const Vector3<Scalar> &translation() const { return translation_; }

  Eigen::Quaternion<Scalar> &orientation() { return orientation_; }

  const Eigen::Quaternion<Scalar> &orientation() const { return orientation_; }

  Eigen::Matrix<Scalar, 3, 3> rotation() const { return orientation_.toRotationMatrix(); }

  Pose2D<Scalar> toPose2D() const
  {
    Eigen::Quaternion<Scalar> yaw( orientation_ );
    yaw.x() = 0;
    yaw.y() = 0;
    yaw.normalize();
    return Pose2D<Scalar>( translation_.x(), translation_.y(), 2 * std::acos( yaw.w() ) );
  }

  // ================= Mathematical Operations =================

  void normalize() { orientation_.normalize(); }

  Pose<Scalar> normalized() const
  {
    Pose<Scalar> result = *this;
    result.normalize();
    return result;
  }

  Isometry3<Scalar> asTransform() const
  {
    Isometry3<Scalar> result = Isometry3<Scalar>::Identity();
    result.linear() = rotation();
    result.translation() = translation_;
    return result;
  }

  Pose<Scalar> inverse() const
  {
    Pose<Scalar> result;
    // We can use the conjugate as the inverse here since the quaternion should be normalized
    result.orientation_ = orientation_.conjugate();
    result.translation_ = result.orientation_ * -translation_;
    return result;
  }

  void setIdentity()
  {
    translation_ = Vector3<Scalar>::Zero();
    orientation_ = Eigen::Quaternion<Scalar>::Identity();
  }

  Pose<Scalar> &operator*=( const Pose<Scalar> &rh ) { return *this = *this * rh; }

  Pose<Scalar> operator*( const Pose<Scalar> &rh ) const
  {
    Pose<Scalar> result;
    result.orientation_ = orientation_ * rh.orientation_;
    result.translation_ = translation_ + orientation_ * rh.translation_;
    return result;
  }

  Pose<Scalar> &operator*=( const Eigen::Translation<Scalar, 3> &translation )
  {
    return translate( translation.vector() );
  }

  Pose<Scalar> operator*( const Eigen::Translation<Scalar, 3> &translation ) const
  {
    Pose<Scalar> result = *this;
    result.translate( translation.vector() );
    return result;
  }

  Vector3<Scalar> operator*( const Vector3<Scalar> &rh ) const
  {
    return translation_ + orientation_ * rh;
  }

  template<typename Derived>
  Pose<Scalar> operator*=( const Eigen::RotationBase<Derived, 3> &rotation )
  {
    return rotate( rotation.derived() );
  }

  template<typename Derived>
  Pose<Scalar> operator*( const Eigen::RotationBase<Derived, 3> &rotation ) const
  {
    Pose<Scalar> result = *this;
    result.rotate( rotation.derived() );
    return result;
  }

  template<int N>
  Eigen::Matrix<Scalar, 3, N> operator*( const Eigen::Matrix<Scalar, 3, N> &rh ) const
  {
    return ( orientation_.toRotationMatrix() * rh ).colwise() + translation_;
  }

  template<typename OtherDerived>
  Pose &translate( const Eigen::MatrixBase<OtherDerived> &translation )
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE( OtherDerived, 3 )
    translation_ += orientation_ * translation.derived();
    return *this;
  }

  template<typename OtherDerived>
  Pose &pretranslate( const Eigen::MatrixBase<OtherDerived> &translation )
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE( OtherDerived, 3 )
    translation_ += translation.derived();
    return *this;
  }

  template<typename RotationType>
  Pose &rotate( const RotationType &rotation )
  {
    orientation_ = orientation_ * rotation;
    return *this;
  }

  template<typename RotationType>
  Pose &prerotate( const RotationType &rotation )
  {
    orientation_ = Eigen::Quaternion<Scalar>( rotation ) * orientation_;
    translation_ = rotation * translation_;
    return *this;
  }

private:
  Eigen::Quaternion<Scalar> orientation_;
  Vector3<Scalar> translation_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename Derived>
Pose<typename Eigen::RotationBase<Derived, 3>::Scalar>
operator*( const Eigen::RotationBase<Derived, 3> &rotation,
           const Pose<typename Eigen::RotationBase<Derived, 3>::Scalar> &pose )
{
  Pose<typename Eigen::RotationBase<Derived, 3>::Scalar> result = pose;
  result.prerotate( rotation.derived() );
  return result;
}

template<typename Scalar>
Pose<Scalar> operator*( const Eigen::Translation<Scalar, 3> &translation, const Pose<Scalar> &pose )
{
  Pose<Scalar> result = pose;
  result.pretranslate( translation.vector() );
  return result;
}

template<typename Scalar>
Pose<Scalar> operator*( const Isometry3<Scalar> &transform, const Pose<Scalar> &pose )
{
  return Pose<Scalar>( transform ) * pose;
}
} // namespace hector_math

#endif // HECTOR_MATH_POSE_H
