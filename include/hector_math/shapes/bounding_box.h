//
// Created by stefan on 20.08.21.
//

#ifndef HECTOR_MATH_BOUNDING_BOX_H
#define HECTOR_MATH_BOUNDING_BOX_H

#include "hector_math/types.h"

namespace hector_math
{

template<typename Scalar>
inline Eigen::AlignedBox<Scalar, 3> computeBoundingBoxForSphere( Scalar radius,
                                                                 const Isometry3<Scalar> &transform );

template<typename Scalar>
inline Eigen::AlignedBox<Scalar, 3> computeBoundingBoxForBox( const Vector3<Scalar> &dim,
                                                              const Isometry3<Scalar> &transform );

template<typename Scalar>
inline Eigen::AlignedBox<Scalar, 3> computeBoundingBoxForCylinder( Scalar radius, Scalar length,
                                                                   const Isometry3<Scalar> &transform );

template<typename Scalar>
inline Eigen::AlignedBox<Scalar, 3> transformBoundingBox( const Eigen::AlignedBox<Scalar, 3> &box,
                                                          const Isometry3<Scalar> &transform );

/// ====================================================
/// ================== IMPLEMENTATION ==================
/// ====================================================

template<typename Scalar>
inline Eigen::AlignedBox<Scalar, 3> computeBoundingBoxForSphere( Scalar radius,
                                                                 const Isometry3 <Scalar> &transform )
{
  return { transform.translation().array() - radius, transform.translation().array() + radius };
}

template<typename Scalar>
inline Eigen::AlignedBox<Scalar, 3> computeBoundingBoxForBox( const Vector3 <Scalar> &dim,
                                                              const Isometry3 <Scalar> &transform )
{
  const Vector3<Scalar> &dim_2 = dim / 2;
  Eigen::Matrix<Scalar, 3, 8> corners;
  // @formatter:off
  corners << dim_2.x(), -dim_2.x(), -dim_2.x(), -dim_2.x(), -dim_2.x(),  dim_2.x(),  dim_2.x(),  dim_2.x(),
             dim_2.y(),  dim_2.y(), -dim_2.y(), -dim_2.y(),  dim_2.y(),  dim_2.y(), -dim_2.y(), -dim_2.y(),
             dim_2.z(),  dim_2.z(),  dim_2.z(), -dim_2.z(), -dim_2.z(), -dim_2.z(), -dim_2.z(),  dim_2.z();
  // @formatter:on
  const Eigen::Matrix<Scalar, 3, 8> &transformed_corners = transform * corners;
  const auto &row_wise = transformed_corners.rowwise();
  const Vector3<Scalar> &min = row_wise.minCoeff();
  const Vector3<Scalar> &max = row_wise.maxCoeff();
  return Eigen::AlignedBox<Scalar, 3>( min, max );
}

template<typename Scalar>
inline Eigen::AlignedBox<Scalar, 3> computeBoundingBoxForCylinder( Scalar radius, Scalar length,
                                                                   const Isometry3 <Scalar> &transform )
{
  // Length is in z direction
  const Vector3<Scalar> tz = transform.linear() * Vector3<Scalar>::UnitZ();
  // Compute projection of length and radius in each axis
  Vector3<Scalar> dim_2( std::abs( tz.x()) * length / 2 + std::sqrt( 1 - tz.x() * tz.x()) * radius,
                         std::abs( tz.y()) * length / 2 + std::sqrt( 1 - tz.y() * tz.y()) * radius,
                         std::abs( tz.z()) * length / 2 + std::sqrt( 1 - tz.z() * tz.z()) * radius );

  return { transform.translation() - dim_2, transform.translation() + dim_2 };
}

template<typename Scalar>
inline Eigen::AlignedBox<Scalar, 3> transformBoundingBox( const Eigen::AlignedBox<Scalar, 3> &box,
                                                          const Isometry3 <Scalar> &transform )
{
  Vector3<Scalar> dim_2 = box.sizes() / 2;
  Eigen::Matrix<Scalar, 3, 8> corners;
  // @formatter:off
  corners << dim_2.x(), -dim_2.x(), -dim_2.x(), -dim_2.x(), -dim_2.x(),  dim_2.x(),  dim_2.x(),  dim_2.x(),
             dim_2.y(),  dim_2.y(), -dim_2.y(), -dim_2.y(),  dim_2.y(),  dim_2.y(), -dim_2.y(), -dim_2.y(),
             dim_2.z(),  dim_2.z(),  dim_2.z(), -dim_2.z(), -dim_2.z(), -dim_2.z(), -dim_2.z(),  dim_2.z();
  // @formatter:on
  const Eigen::Matrix<Scalar, 3, 8> &transformed_corners = transform * corners;
  const auto &row_wise = transformed_corners.rowwise();
  const Vector3<Scalar> &min = row_wise.minCoeff();
  const Vector3<Scalar> &max = row_wise.maxCoeff();
  return { min, max };
}
}

#endif //HECTOR_MATH_BOUNDING_BOX_H
