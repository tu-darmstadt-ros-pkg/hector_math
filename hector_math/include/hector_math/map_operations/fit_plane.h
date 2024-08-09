// Copyright (c) 2024 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_FIT_PLANE_H
#define HECTOR_MATH_FIT_PLANE_H

#include "hector_math/types/aggregators.h"
#include <Eigen/Core>

namespace hector_math
{

struct PlaneEstimationResult {
  // The z value of the plane in the center of the map.
  float center_plane_z;
  float gradient_x;
  float gradient_y;
  //! The quality of the estimation in x and y direction from 0 (no data) to 1 (good data).
  float quality_x;
  float quality_y;
};

template<typename Derived>
PlaneEstimationResult fitPlane( const Eigen::DenseBase<Derived> &map )
{
  MeanAggregator<double> mean_x, mean_y, mean_z;
  MeanAggregator<double> central_row, central_col;
  for ( Eigen::Index row = 1; row < map.rows(); ++row ) {
    const double dx = map( row, 0 ) - map( row - 1, 0 );
    if ( !std::isfinite( dx ) )
      continue;
    mean_x.add( dx );
    mean_z.add( map( row, 0 ) );
    // This should be -0.5 as we compute the gradient between cells but we compensate later
    central_row.add( row );
    central_col.add( 0 );
  }
  for ( Eigen::Index col = 1; col < map.cols(); ++col ) {
    const double dy = map( 0, col ) - map( 0, col - 1 );
    if ( std::isfinite( dy ) ) {
      mean_y.add( dy );
      mean_z.add( map( 0, col ) );
      central_row.add( 0 );
      central_col.add( col );
    }
    for ( Eigen::Index row = 1; row < map.rows(); ++row ) {
      const double value = map( row, col );
      if ( !std::isfinite( value ) )
        continue;
      central_col.add( col );
      central_row.add( row );
      mean_z.add( value );

      const double dx = value - map( row - 1, col );
      const double dy = value - map( row, col - 1 );
      if ( std::isfinite( dx ) ) {
        mean_x.add( dx );
      }
      if ( std::isfinite( dy ) ) {
        mean_y.add( dy );
      }
    }
  }
  PlaneEstimationResult result;
  result.gradient_x = static_cast<float>( mean_x.mean() );
  result.gradient_y = static_cast<float>( mean_y.mean() );
  result.center_plane_z = mean_z.mean() -
                          ( central_row.mean() - ( map.rows() - 1 ) / 2.0 ) * result.gradient_x -
                          ( central_col.mean() - ( map.cols() - 1 ) / 2.0 ) * result.gradient_y;
  result.quality_x = static_cast<float>( mean_x.count() ) / ( map.rows() - 1 ) / map.cols();
  result.quality_y = static_cast<float>( mean_y.count() ) / ( map.cols() - 1 ) / map.rows();
  return result;
}

} // namespace hector_math

#endif // HECTOR_MATH_FIT_PLANE_H
