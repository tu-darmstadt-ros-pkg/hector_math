// Copyright (c) 2023 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_POSE2D_H
#define HECTOR_MATH_POSE2D_H

#include "hector_math/types/eigen.h"

namespace hector_math
{

template<typename Scalar>
class Pose2D;
template<typename Scalar>
using Pose2DList = std::vector<Pose2D<Scalar>, Eigen::aligned_allocator<Pose2D<Scalar>>>;

//! Represents the 2D pose of a robot or an object as a combination of orientation angle (around the
//! z-axis) and 2D translation vector.
template<typename Scalar>
class Pose2D
{
public:
  static Pose2D<Scalar> Identity()
  {
    Pose2D<Scalar> result;
    result.translation_ = Vector2<Scalar>::Zero();
    result.orientation_ = 0;
    return result;
  }

  static Pose2D<Scalar> Origin() { return Identity(); }

  static Pose2D<Scalar> Translation( Scalar x, Scalar y )
  {
    return Translation( Vector3<Scalar>( x, y ) );
  }

  static Pose2D<Scalar> Translation( const Vector2<Scalar> &translation )
  {
    Pose2D<Scalar> result;
    result.translation_ = translation;
    result.orientation_ = 0;
    return result;
  }

  Pose2D() = default;

  Pose2D( Scalar tx, Scalar ty, Scalar yaw )
  {
    translation_ = { tx, ty };
    orientation_ = yaw;
  }

  explicit Pose2D( const Isometry2<Scalar> &isometry )
  {
    translation_ = isometry.translation();
    orientation_ = Eigen::Rotation2D<Scalar>( isometry.linear() ).angle();
  }

  explicit Pose2D( const Eigen::Translation<Scalar, 2> &translation )
  {
    orientation_ = 0;
    translation_ = translation.vector();
  }

  template<typename Derived>
  explicit Pose2D( const Eigen::RotationBase<Derived, 2> &rotation )
  {
    orientation_ = Eigen::Rotation2D<Scalar>( rotation.derived() ).angle();
    translation_ = Vector3<Scalar>::Zero();
  }

  Pose2D( const Eigen::Translation<Scalar, 2> &translation, Scalar orientation )
  {
    orientation_ = orientation;
    translation_ = translation.vector();
  }

  template<typename Derived>
  Pose2D( const Eigen::Translation<Scalar, 2> &translation,
          const Eigen::RotationBase<Derived, 3> &rotation )
  {
    orientation_ = Eigen::Rotation2D<Scalar>( rotation.derived() ).angle();
    translation_ = translation.vector();
  }

  template<typename Derived>
  Pose2D( const Eigen::Translation<Scalar, 2> &translation,
          const Eigen::MatrixBase<Derived> &rotation )
  {
    orientation_ = Eigen::Rotation2D<Scalar>( rotation.derived() ).angle();
    translation_ = translation.vector();
  }

  Pose2D( const Vector2<Scalar> &translation, Scalar orientation )
  {
    orientation_ = orientation;
    translation_ = translation;
  }

  template<typename Derived>
  Pose2D( const Vector2<Scalar> &translation, const Eigen::RotationBase<Derived, 2> &rotation )
  {
    orientation_ = Eigen::Rotation2D<Scalar>( rotation.derived() ).angle();
    translation_ = translation;
  }

  template<typename Derived>
  Pose2D( const Vector2<Scalar> &translation, const Eigen::MatrixBase<Derived> &rotation )
  {
    orientation_ = Eigen::Rotation2D<Scalar>( rotation.derived() ).angle();
    translation_ = translation;
  }

  Vector2<Scalar> &translation() { return translation_; }

  const Vector2<Scalar> &translation() const { return translation_; }

  Scalar orientation() const { return orientation_; }

  Eigen::Rotation2D<Scalar> rotation() const { return Eigen::Rotation2D<Scalar>( orientation_ ); }

  // ================= Mathematical Operations =================

  void normalize() { orientation_.normalize(); }

  Pose2D<Scalar> normalized()
  {
    Pose2D<Scalar> result = *this;
    result.normalize();
    return result;
  }

  Isometry2<Scalar> asTransform() const
  {
    Isometry2<Scalar> result = Isometry2<Scalar>::Identity();
    result.linear() = rotation().toRotationMatrix();
    result.translation() = translation_;
    return result;
  }

  Pose2D<Scalar> inverse() const
  {
    Pose2D<Scalar> result;
    result.orientation_ = -orientation_;
    result.translation_ = Eigen::Rotation2D<Scalar>( result.orientation_ ) * -translation_;
    return result;
  }

  void setIdentity()
  {
    translation_ = Vector3<Scalar>::Zero();
    orientation_ = Eigen::Quaternion<Scalar>::Identity();
  }

  Pose2D<Scalar> &operator*=( const Pose2D<Scalar> &rh ) { return *this = *this * rh; }

  Pose2D<Scalar> operator*( const Pose2D<Scalar> &rh )
  {
    Pose2D<Scalar> result;
    result.orientation_ = orientation_ + rh.orientation_;
    result.translation_ = translation_ + Eigen::Rotation2D<Scalar>( orientation_ ) * rh.translation_;
    return result;
  }

  Pose2D<Scalar> &operator*=( const Eigen::Translation<Scalar, 2> &translation )
  {
    return translate( translation.vector() );
  }

  Pose2D<Scalar> operator*( const Eigen::Translation<Scalar, 2> &translation )
  {
    Pose2D<Scalar> result = *this;
    result.translate( translation.vector() );
    return result;
  }

  Vector3<Scalar> operator*( const Vector2<Scalar> &rh )
  {
    return translation_ + orientation_ * rh;
  }

  template<typename Derived>
  Pose2D<Scalar> operator*=( const Eigen::RotationBase<Derived, 2> &rotation )
  {
    return rotate( rotation.derived() );
  }

  template<typename Derived>
  Pose2D<Scalar> operator*( const Eigen::RotationBase<Derived, 2> &rotation )
  {
    Pose2D<Scalar> result = *this;
    result.rotate( rotation.derived() );
    return result;
  }

  template<int N>
  Eigen::Matrix<Scalar, 2, N> operator*( const Eigen::Matrix<Scalar, 2, N> &rh )
  {
    return ( Eigen::Rotation2D<Scalar>( orientation_ ) * rh ).colwise() + translation_;
  }

  template<typename OtherDerived>
  Pose2D &translate( const Eigen::MatrixBase<OtherDerived> &translation )
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE( OtherDerived, 2 )
    translation_ += Eigen::Rotation2D<Scalar>( orientation_ ) * translation.derived();
    return *this;
  }

  template<typename OtherDerived>
  Pose2D &pretranslate( const Eigen::MatrixBase<OtherDerived> &translation )
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE( OtherDerived, 2 )
    translation_ += translation.derived();
    return *this;
  }

  template<typename RotationType>
  Pose2D &rotate( const RotationType &rotation )
  {
    orientation_ += rotation;
    return *this;
  }

  template<typename RotationType>
  Pose2D &prerotate( const RotationType &rotation )
  {
    orientation_ = rotation + orientation_;
    translation_ = Eigen::Rotation2D<Scalar>( rotation ) * translation_;
    return *this;
  }

private:
  Scalar orientation_;
  Vector2<Scalar> translation_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace hector_math

#endif // HECTOR_MATH_POSE2D_H
