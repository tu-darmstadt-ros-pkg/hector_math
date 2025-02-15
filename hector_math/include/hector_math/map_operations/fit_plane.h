// Copyright (c) 2024 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_FIT_PLANE_H
#define HECTOR_MATH_FIT_PLANE_H

#include "hector_math/iterators/eigen_iterator.h"
#include "hector_math/types/aggregators.h"
#include <Eigen/Core>

namespace hector_math
{

struct PlaneEstimationResult {
  // The z value of the plane in the center of the map.
  float center_plane_z;
  float gradient_x;
  float gradient_y;
  //! The percentage of known values during estimation.
  float percentage_known;
};

/*!
 * @brief Fits a plane to the given map.
 * @param map The 2D array of height values this plane is fitted to.
 * @param resolution The resolution of the map. Used to scale the gradient.
 */
template<typename Derived, bool ( *is_valid_fn )( typename Eigen::DenseBase<Derived>::Scalar ) = std::isfinite>
PlaneEstimationResult fitPlaneXY( const Eigen::DenseBase<Derived> &map, const double resolution = 1.0 )
{
  using Scalar = typename Eigen::DenseBase<Derived>::Scalar;
  Scalar row_squared_sum = 0;
  Scalar row_sum = 0;
  Scalar col_squared_sum = 0;
  Scalar row_col_sum = 0;
  Scalar col_sum = 0;
  long count = 0;
  Vector3d z = Vector3d::Zero();
  for ( Eigen::Index col = 0; col < map.cols(); ++col ) {
    for ( Eigen::Index row = 0; row < map.rows(); ++row ) {
      const auto &value = map( row, col );
      if ( !is_valid_fn( value ) )
        continue;
      row_squared_sum += row * row;
      row_sum += row;
      col_squared_sum += col * col;
      col_sum += col;
      row_col_sum += row * col;
      ++count;
      z += Vector3d( row * value, col * value, value );
    }
  }
  Eigen::Matrix3d X;
  // clang-format off
  X << row_squared_sum, row_col_sum,     row_sum,
       row_col_sum,     col_squared_sum, col_sum,
       row_sum,         col_sum,         count;
  // clang-format on
  Vector3d abc = X.inverse() * z;
  PlaneEstimationResult result;
  result.gradient_x = static_cast<float>( abc( 0 ) / resolution );
  result.gradient_y = static_cast<float>( abc( 1 ) / resolution );
  result.center_plane_z = static_cast<float>( abc( 2 ) ) +
                          result.gradient_x * ( map.rows() - 1 ) * resolution / 2 +
                          result.gradient_y * ( map.cols() - 1 ) * resolution / 2;
  result.percentage_known = static_cast<float>( count ) / ( map.rows() * map.cols() );
  return result;
}

template<typename Derived>
__attribute_deprecated_msg__( "Use fitPlaneXY instead." ) PlaneEstimationResult
    fitPlane( const Eigen::DenseBase<Derived> &map, const double resolution = 1.0 )
{
  return fitPlaneXY( map, resolution );
}

} // namespace hector_math

#endif // HECTOR_MATH_FIT_PLANE_H
