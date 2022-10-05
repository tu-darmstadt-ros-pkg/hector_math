//
// Created by Stefan Fabian on 17.08.21.
//

#ifndef HECTOR_MATH_TYPES_H
#define HECTOR_MATH_TYPES_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>

namespace hector_math
{

template<typename Scalar>
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
using Matrix3f = Matrix3<float>;
using Matrix3d = Matrix3<double>;

template<typename Scalar>
using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
using Vector2f = Vector2<float>;
using Vector2d = Vector2<double>;
template<typename Scalar>
using Vector2List = std::vector<Vector2<Scalar>, Eigen::aligned_allocator<Vector2<Scalar>>>;
using Vector2fList = Vector2List<float>;
using Vector2dList = Vector2List<double>;

template<typename Scalar>
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
using Vector3f = Vector3<float>;
using Vector3d = Vector3<double>;
template<typename Scalar>
using Vector3List = std::vector<Vector3<Scalar>>;
using Vector3fList = Vector3List<float>;
using Vector3dList = Vector3List<double>;

template<typename Scalar>
using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
using Vector4f = Vector4<float>;
using Vector4d = Vector4<double>;
template<typename Scalar>
using Vector4List = std::vector<Vector4<Scalar>, Eigen::aligned_allocator<Vector4<Scalar>>>;
using Vector4fList = Vector4List<float>;
using Vector4dList = Vector4List<double>;

template<typename Scalar>
using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
using VectorXf = VectorX<float>;
using VectorXd = VectorX<double>;
template<typename Scalar>
using VectorXList = std::vector<VectorX<Scalar>>;
using VectorXfList = VectorXList<float>;
using VectorXdList = VectorXList<double>;

template<typename Scalar>
using Isometry3 = Eigen::Transform<Scalar, 3, Eigen::Isometry>;
using Isometry3f = Isometry3<float>;
using Isometry3d = Isometry3<double>;

template<typename Scalar>
using Isometry2 = Eigen::Transform<Scalar, 2, Eigen::Isometry>;
using Isometry2f = Isometry2<float>;
using Isometry2d = Isometry2<double>;

template<typename Scalar>
using Point = Eigen::Array<Scalar, 2, 1>;
template<typename Scalar>
using PointList = std::vector<Point<Scalar>, Eigen::aligned_allocator<Point<Scalar>>>;

template<typename Scalar>
class Pose2D;
template<typename Scalar>
using Pose2DList = std::vector<Pose2D<Scalar>, Eigen::aligned_allocator<Pose2D<Scalar>>>;

template<typename Scalar>
class Pose;
template<typename Scalar>
using PoseList = std::vector<Pose<Scalar>, Eigen::aligned_allocator<Pose<Scalar>>>;

template<typename Scalar>
using Polygon = Eigen::Array<Scalar, 2, Eigen::Dynamic>;

template<typename Scalar>
using GridMap = Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

struct BlockIndices {
  Eigen::Index x0;
  Eigen::Index y0;
  Eigen::Index rows;
  Eigen::Index cols;
};

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

//! Represents the pose of a robot as a combination of orientation quaternion and translation
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

  Pose2D<Scalar> toPose2D()
  {
    Eigen::Quaternion<Scalar> yaw( orientation_ );
    yaw.x() = 0;
    yaw.y() = 0;
    yaw.normalize();
    return Pose2D<Scalar>( translation_.x(), translation_.y(), 2 * std::acos( yaw.w() ) );
  }

  // ================= Mathematical Operations =================

  void normalize() { orientation_.normalize(); }

  Pose<Scalar> normalized()
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

  Pose<Scalar> operator*( const Pose<Scalar> &rh )
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

  Pose<Scalar> operator*( const Eigen::Translation<Scalar, 3> &translation )
  {
    Pose<Scalar> result = *this;
    result.translate( translation.vector() );
    return result;
  }

  Vector3<Scalar> operator*( const Vector3<Scalar> &rh )
  {
    return translation_ + orientation_ * rh;
  }

  template<typename Derived>
  Pose<Scalar> operator*=( const Eigen::RotationBase<Derived, 3> &rotation )
  {
    return rotate( rotation.derived() );
  }

  template<typename Derived>
  Pose<Scalar> operator*( const Eigen::RotationBase<Derived, 3> &rotation )
  {
    Pose<Scalar> result = *this;
    result.rotate( rotation.derived() );
    return result;
  }

  template<int N>
  Eigen::Matrix<Scalar, 3, N> operator*( const Eigen::Matrix<Scalar, 3, N> &rh )
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
#endif // HECTOR_MATH_TYPES_H
