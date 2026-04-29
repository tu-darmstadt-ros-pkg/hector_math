// Copyright (c) 2024 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_FIT_PLANE_H
#define HECTOR_MATH_FIT_PLANE_H

#include "hector_math/iterators/eigen_iterator.h"
#include "hector_math/math/search.h"
#include "hector_math/types/aggregators.h"
#include <Eigen/Core>
#include <Eigen/LU>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iterator>
#include <type_traits>
#include <vector>

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
 * @brief Fits a plane to the given map using a least-squares fit.
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
  for ( const auto &[row, col] : EigenIndexIterator( map ) ) {
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
  Eigen::Matrix3d X;
  // clang-format off
  X << row_squared_sum, row_col_sum,     row_sum,
       row_col_sum,     col_squared_sum, col_sum,
       row_sum,         col_sum,         count;
  // clang-format on
  Vector3d abc = X.inverse() * z;
  PlaneEstimationResult result = {};
  result.gradient_x = static_cast<float>( abc( 0 ) / resolution );
  result.gradient_y = static_cast<float>( abc( 1 ) / resolution );
  result.center_plane_z = static_cast<float>( abc( 2 ) ) +
                          result.gradient_x * ( map.rows() - 1 ) * resolution / 2 +
                          result.gradient_y * ( map.cols() - 1 ) * resolution / 2;
  result.percentage_known = static_cast<float>( count ) / ( map.rows() * map.cols() );
  return result;
}

/*!
 * @brief Fast robust plane fit using coarse-grid finite differences and
 *        component-wise medians. Approximately an order of magnitude faster
 *        than a full RANSAC fit at comparable accuracy on planes with moderate
 *        noise. Walks a coarse grid twice (default stride 4): once to collect gradient
 *        samples, once to collect intercept samples; no per-sample scratch
 *        storage between the passes. Uses a stack-allocated scratch buffer for
 *        small maps and falls back to the heap for large ones.
 *
 *        For maps with very high NaN fractions (>90 %), use
 *        fitPlaneXYRobustBlockMedian instead — it is slower but does not
 *        degrade at extreme densities.
 *
 * @param map The 2D array of height values this plane is fitted to.
 * @param result Output. Updated only on success.
 * @param resolution The resolution of the map. Used to scale the gradient.
 * @param sample_stride Coarse-grid step in cells. Larger values are faster but
 *        sample fewer points; must be >= 1. Defaults to 4.
 *
 * Note: percentage_known is estimated from the coarse-grid validity ratio
 * (unbiased under non-pathological NaN distributions). Use fitPlaneXY if an
 * exact count is required.
 */
template<typename Derived, bool ( *is_valid_fn )( typename Eigen::DenseBase<Derived>::Scalar ) = std::isfinite>
bool fitPlaneXYRobust( const Eigen::DenseBase<Derived> &map, PlaneEstimationResult &result,
                       const double resolution = 1.0, const int sample_stride = 4 )
{
  using Scalar = typename Eigen::DenseBase<Derived>::Scalar;
  using Scratch = std::conditional_t<std::is_same_v<Scalar, float>, float, double>;

  assert( sample_stride >= 1 );
  if ( map.rows() < 2 || map.cols() < 2 ) {
    result = {};
    return false;
  }

  const Eigen::Index rows = map.rows();
  const Eigen::Index cols = map.cols();
  const Eigen::Index step_r = std::min<Eigen::Index>( sample_stride, rows - 1 );
  const Eigen::Index step_c = std::min<Eigen::Index>( sample_stride, cols - 1 );
  const Eigen::Index half_r = std::max<Eigen::Index>( 1, step_r / 2 );
  const Eigen::Index half_c = std::max<Eigen::Index>( 1, step_c / 2 );
  const double inv_full_r = 1.0 / ( static_cast<double>( step_r ) * resolution );
  const double inv_full_c = 1.0 / ( static_cast<double>( step_c ) * resolution );
  const double inv_half_r = 1.0 / ( static_cast<double>( half_r ) * resolution );
  const double inv_half_c = 1.0 / ( static_cast<double>( half_c ) * resolution );

  const size_t reserve_n = static_cast<size_t>( ( rows + step_r - 1 ) / step_r ) *
                           static_cast<size_t>( ( cols + step_c - 1 ) / step_c );
  constexpr size_t kStackGradientCapacity = 2048;

  long coarse_total = 0;
  long coarse_valid = 0;

  // ---- Pass 1: collect gradient samples (no per-sample storage).
  auto run_with_buffers = [&]( Scratch *g_x, Scratch *g_y, size_t *g_x_count_out,
                               size_t *g_y_count_out ) {
    size_t g_x_count = 0;
    size_t g_y_count = 0;
    const auto collect = [&]( const Eigen::Index row, const Eigen::Index col ) {
      ++coarse_total;
      const auto &v = map( row, col );
      if ( !is_valid_fn( v ) )
        return;
      ++coarse_valid;
      const double dv = static_cast<double>( v );
      if ( row + step_r < rows ) {
        const auto &v1 = map( row + step_r, col );
        if ( is_valid_fn( v1 ) )
          g_x[g_x_count++] = static_cast<Scratch>( ( static_cast<double>( v1 ) - dv ) * inv_full_r );
      }
      if ( half_r != step_r && row + half_r < rows ) {
        const auto &v1 = map( row + half_r, col );
        if ( is_valid_fn( v1 ) )
          g_x[g_x_count++] = static_cast<Scratch>( ( static_cast<double>( v1 ) - dv ) * inv_half_r );
      }
      if ( col + step_c < cols ) {
        const auto &v1 = map( row, col + step_c );
        if ( is_valid_fn( v1 ) )
          g_y[g_y_count++] = static_cast<Scratch>( ( static_cast<double>( v1 ) - dv ) * inv_full_c );
      }
      if ( half_c != step_c && col + half_c < cols ) {
        const auto &v1 = map( row, col + half_c );
        if ( is_valid_fn( v1 ) )
          g_y[g_y_count++] = static_cast<Scratch>( ( static_cast<double>( v1 ) - dv ) * inv_half_c );
      }
    };
    if constexpr ( Eigen::DenseBase<Derived>::IsRowMajor ) {
      for ( Eigen::Index row = 0; row < rows; row += step_r ) {
        for ( Eigen::Index col = 0; col < cols; col += step_c ) collect( row, col );
      }
    } else {
      for ( Eigen::Index col = 0; col < cols; col += step_c ) {
        for ( Eigen::Index row = 0; row < rows; row += step_r ) collect( row, col );
      }
    }
    *g_x_count_out = g_x_count;
    *g_y_count_out = g_y_count;
  };

  double gradient_x;
  double gradient_y;
  size_t g_x_count = 0;
  size_t g_y_count = 0;

  if ( reserve_n * 2 <= kStackGradientCapacity ) {
    std::array<Scratch, kStackGradientCapacity> g_x_buf;
    std::array<Scratch, kStackGradientCapacity> g_y_buf;
    run_with_buffers( g_x_buf.data(), g_y_buf.data(), &g_x_count, &g_y_count );
    if ( coarse_valid < 3 || g_x_count == 0 || g_y_count == 0 ) {
      result = {};
      return false;
    }
    gradient_x =
        static_cast<double>( findMedianUpper( g_x_buf.begin(), g_x_buf.begin() + g_x_count ) );
    gradient_y =
        static_cast<double>( findMedianUpper( g_y_buf.begin(), g_y_buf.begin() + g_y_count ) );
  } else {
    std::vector<Scratch> g_x_buf( reserve_n * 2 );
    std::vector<Scratch> g_y_buf( reserve_n * 2 );
    run_with_buffers( g_x_buf.data(), g_y_buf.data(), &g_x_count, &g_y_count );
    if ( coarse_valid < 3 || g_x_count == 0 || g_y_count == 0 ) {
      result = {};
      return false;
    }
    gradient_x =
        static_cast<double>( findMedianUpper( g_x_buf.begin(), g_x_buf.begin() + g_x_count ) );
    gradient_y =
        static_cast<double>( findMedianUpper( g_y_buf.begin(), g_y_buf.begin() + g_y_count ) );
  }

  // ---- Pass 2: re-walk coarse grid, compute intercept residuals on the fly.
  const double center_row = static_cast<double>( rows - 1 ) / 2.0;
  const double center_col = static_cast<double>( cols - 1 ) / 2.0;
  const double slope_r = gradient_x * resolution;
  const double slope_c = gradient_y * resolution;

  auto compute_intercept = [&]( Scratch *intercepts ) {
    size_t n = 0;
    auto push = [&]( const Eigen::Index row, const Eigen::Index col ) {
      const auto &v = map( row, col );
      if ( !is_valid_fn( v ) )
        return;
      intercepts[n++] =
          static_cast<Scratch>( static_cast<double>( v ) - slope_r * ( row - center_row ) -
                                slope_c * ( col - center_col ) );
    };
    if constexpr ( Eigen::DenseBase<Derived>::IsRowMajor ) {
      for ( Eigen::Index row = 0; row < rows; row += step_r ) {
        for ( Eigen::Index col = 0; col < cols; col += step_c ) push( row, col );
      }
    } else {
      for ( Eigen::Index col = 0; col < cols; col += step_c ) {
        for ( Eigen::Index row = 0; row < rows; row += step_r ) push( row, col );
      }
    }
    return n;
  };

  double intercept;
  if ( reserve_n <= kStackGradientCapacity ) {
    std::array<Scratch, kStackGradientCapacity> buf;
    const size_t n = compute_intercept( buf.data() );
    intercept = static_cast<double>( findMedianUpper( buf.begin(), buf.begin() + n ) );
  } else {
    std::vector<Scratch> buf( reserve_n );
    const size_t n = compute_intercept( buf.data() );
    intercept = static_cast<double>( findMedianUpper( buf.begin(), buf.begin() + n ) );
  }

  const long approx_count_valid =
      coarse_total == 0
          ? 0
          : static_cast<long>( static_cast<double>( coarse_valid ) /
                               static_cast<double>( coarse_total ) * static_cast<double>( rows ) *
                               static_cast<double>( cols ) );
  result.gradient_x = static_cast<float>( gradient_x );
  result.gradient_y = static_cast<float>( gradient_y );
  result.center_plane_z = static_cast<float>( intercept );
  result.percentage_known =
      static_cast<float>( static_cast<double>( approx_count_valid ) /
                          ( static_cast<double>( rows ) * static_cast<double>( cols ) ) );
  return true;
}

/*!
 * @brief Robust plane fit using block medians on a fixed subsampling grid.
 *        Slower than fitPlaneXYRobust but the most accurate of the robust
 *        variants and continues to produce useful results at extreme NaN
 *        fractions where fitPlaneXYRobust degrades.
 * @param map The 2D array of height values this plane is fitted to.
 * @param resolution The resolution of the map. Used to scale the gradient.
 */
template<typename Derived, bool ( *is_valid_fn )( typename Eigen::DenseBase<Derived>::Scalar ) = std::isfinite>
bool fitPlaneXYRobustBlockMedian( const Eigen::DenseBase<Derived> &map,
                                  PlaneEstimationResult &result, const double resolution = 1.0 )
{
  constexpr int kSubsamplingFactor = 4;
  if ( map.rows() == 0 || map.cols() == 0 ) {
    result = {};
    return false;
  }

  double row_squared_sum = 0.0;
  double row_sum = 0.0;
  double col_squared_sum = 0.0;
  double row_col_sum = 0.0;
  double col_sum = 0.0;
  long count_valid = 0;
  long sample_count = 0;
  Vector3d z = Vector3d::Zero();
  std::array<double, kSubsamplingFactor * kSubsamplingFactor> block_values{};

  for ( Eigen::Index row_start = 0; row_start < map.rows(); row_start += kSubsamplingFactor ) {
    const Eigen::Index row_end = std::min<Eigen::Index>( row_start + kSubsamplingFactor, map.rows() );
    for ( Eigen::Index col_start = 0; col_start < map.cols(); col_start += kSubsamplingFactor ) {
      const Eigen::Index col_end =
          std::min<Eigen::Index>( col_start + kSubsamplingFactor, map.cols() );
      size_t block_count = 0;
      for ( Eigen::Index row = row_start; row < row_end; ++row ) {
        for ( Eigen::Index col = col_start; col < col_end; ++col ) {
          const auto &value = map( row, col );
          if ( !is_valid_fn( value ) ) {
            continue;
          }
          ++count_valid;
          block_values[block_count++] = static_cast<double>( value );
        }
      }
      if ( block_count == 0 ) {
        continue;
      }

      const double row =
          ( static_cast<double>( row_start ) + static_cast<double>( row_end ) - 1.0 ) / 2.0;
      const double col =
          ( static_cast<double>( col_start ) + static_cast<double>( col_end ) - 1.0 ) / 2.0;
      const double median = findMedian( block_values.begin(), block_values.begin() + block_count );
      row_squared_sum += row * row;
      row_sum += row;
      col_squared_sum += col * col;
      col_sum += col;
      row_col_sum += row * col;
      ++sample_count;
      z += Vector3d( row * median, col * median, median );
    }
  }

  if ( sample_count < 3 || count_valid < 3 ) {
    result = {};
    return false;
  }

  Eigen::Matrix3d X;
  // clang-format off
  X << row_squared_sum, row_col_sum,     row_sum,
       row_col_sum,     col_squared_sum, col_sum,
       row_sum,         col_sum,         sample_count;
  // clang-format on

  Eigen::FullPivLU<Eigen::Matrix3d> decomposition( X );
  if ( !decomposition.isInvertible() ) {
    result = {};
    return false;
  }

  const Vector3d abc = decomposition.solve( z );
  if ( !abc.allFinite() ) {
    result = {};
    return false;
  }

  result.gradient_x = static_cast<float>( abc( 0 ) / resolution );
  result.gradient_y = static_cast<float>( abc( 1 ) / resolution );
  result.center_plane_z = static_cast<float>( abc( 2 ) + abc( 0 ) * ( map.rows() - 1 ) / 2.0 +
                                              abc( 1 ) * ( map.cols() - 1 ) / 2.0 );
  result.percentage_known =
      static_cast<float>( static_cast<double>( count_valid ) /
                          ( static_cast<double>( map.rows() ) * static_cast<double>( map.cols() ) ) );
  return true;
}

template<typename Derived>
__attribute_deprecated_msg__( "Use fitPlaneXY instead." ) PlaneEstimationResult
    fitPlane( const Eigen::DenseBase<Derived> &map, const double resolution = 1.0 )
{
  return fitPlaneXY( map, resolution );
}

} // namespace hector_math

#endif // HECTOR_MATH_FIT_PLANE_H
