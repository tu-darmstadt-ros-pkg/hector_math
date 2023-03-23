// Copyright (c) 2021 Stefan Fabian, Aljoscha Schmidt. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_QUATERNION_BINNING_H
#define HECTOR_MATH_QUATERNION_BINNING_H

#include "hector_math/math/operations.h"
#include <Eigen/Geometry>

namespace hector_math
{

namespace detail
{
constexpr unsigned computeRequiredBits( unsigned x )
{
  return x < 2 ? x : 1 + computeRequiredBits( x >> 1 );
}

template<typename T>
constexpr T computeBitMask( T n )
{
  return n == 0 ? 0 : ( computeBitMask<T>( n - 1 ) << 1 ) | 1;
}

template<int AXIS_BINS, int ANGLE_BINS>
struct QuaternionBinType {
  static constexpr int _required_bits =
      2 * computeRequiredBits( AXIS_BINS - 1 ) + computeRequiredBits( ANGLE_BINS ) + 3;
  using BinType =
      typename std::conditional<_required_bits >= 8 * sizeof( int ), unsigned long, unsigned int>::type;
};

template<typename Scalar>
Scalar madfrac( Scalar a, Scalar b )
{
  return a * b - std::floor( a * b );
}
} // namespace detail

namespace quaternion_binning_modes
{
//! You can visualize the different binning modes using visualize_binning.py
//! LargestDim is somewhat faster and doesn't have a reduced resolution at the poles due to the
//! equidistant angles used in spherical but at the cost of overlapping regions with increased
//! bin resolution
enum QuaternionBinningMode { LargestDim, Spherical, SphericalFibonacci };
} // namespace quaternion_binning_modes
using QuaternionBinningMode = quaternion_binning_modes::QuaternionBinningMode;

/*!
 * Computes the bin for the given quaternion.
 * Close quaternions should land in the same bin.
 *
 * @tparam AXIS_BINS
 * @tparam ANGLE_BINS
 * @tparam mode
 * @tparam Scalar
 * @tparam ReturnType
 * @param q
 * @return
 */
template<typename Scalar, int AXIS_BINS, int ANGLE_BINS, QuaternionBinningMode mode,
         typename ReturnType = typename detail::QuaternionBinType<AXIS_BINS, ANGLE_BINS>::BinType>
ReturnType computeBin( const Eigen::Quaternion<Scalar> &q )
{
  assert( std::abs( q.squaredNorm() - 1 ) < 1e-4 &&
          "The quaternion binning assumes quaternions to be normalized!" );
  static constexpr int dim_bits = detail::computeRequiredBits( AXIS_BINS - 1 );
  static constexpr ReturnType DIM_MASK = detail::computeBitMask<ReturnType>( dim_bits );
  static constexpr ReturnType ANGLE_MASK =
      detail::computeBitMask<ReturnType>( detail::computeRequiredBits( ANGLE_BINS - 1 ) );
  // First resolve conflict of -q and q representing the same orientation.
  if ( std::signbit( q.w() ) ) {
    return computeBin<Scalar, AXIS_BINS, ANGLE_BINS, mode, ReturnType>(
        Eigen::Quaternion<Scalar>( -q.w(), -q.x(), -q.y(), -q.z() ) );
  }

  if ( mode == quaternion_binning_modes::LargestDim ) {
    const Scalar x2 = q.x() * q.x();
    const Scalar y2 = q.y() * q.y();
    const Scalar z2 = q.z() * q.z();
    ReturnType bin = 0;
    // Since we find the largest dim first, the other dims are absolute smaller than sqrt(2) / 2
    // Hence we move them from (-sqrt(2) / 2, sqrt(2) / 2) to (0, AXIS_BINS)
    constexpr Scalar MULTIPLIER = M_SQRT2 * AXIS_BINS / 2;
    constexpr Scalar OFFSET = Scalar( AXIS_BINS ) / 2;
    // Find the largest component for stable binning.
    if ( x2 > y2 && x2 > z2 ) {
      bin = 0b01;
      if ( q.x() < 0 )
        bin |= 0b100;
      const Scalar norm = std::sqrt( x2 + y2 + z2 );
      // Since x is largest, y and z are absolute smaller than sqrt(2)/2
      bin |= ( ReturnType( q.y() * MULTIPLIER / norm + OFFSET ) & DIM_MASK ) << 3;
      bin |= ( ReturnType( q.z() * MULTIPLIER / norm + OFFSET ) & DIM_MASK ) << ( 3 + dim_bits );
    } else if ( y2 > z2 ) {
      bin = 0b10;
      if ( q.y() < 0 )
        bin |= 0b100;
      const Scalar norm = std::sqrt( x2 + y2 + z2 );
      bin |= ( ReturnType( q.x() * MULTIPLIER / norm + OFFSET ) & DIM_MASK ) << 3;
      bin |= ( ReturnType( q.z() * MULTIPLIER / norm + OFFSET ) & DIM_MASK ) << ( 3 + dim_bits );
    } else {
      bin = 0b11;
      if ( q.z() < 0 )
        bin |= 0b100;
      const Scalar norm = std::sqrt( x2 + y2 + z2 );
      bin |= ( ReturnType( q.y() * MULTIPLIER / norm + OFFSET ) & DIM_MASK ) << 3;
      bin |= ( ReturnType( q.x() * MULTIPLIER / norm + OFFSET ) & DIM_MASK ) << ( 3 + dim_bits );
    }
    // Angle is 2 * acos(qw) which we move from [0, 2 * pi] to [0, ANGLE_BINS)
    const Scalar angle = std::acos( q.w() );
    bin |= ( ReturnType( angle * ( ANGLE_BINS / M_PI ) ) & ANGLE_MASK ) << ( 2 * dim_bits + 3 );
    return bin;

  } else if ( mode == quaternion_binning_modes::SphericalFibonacci ) {
    constexpr Scalar PHI = ( std::sqrt( 5 ) + Scalar( 1 ) ) / Scalar( 2 );
    constexpr int fibonacci_n = ANGLE_BINS * ANGLE_BINS;
    ; // TODO: Fibonacci_n input param
    const Scalar norm = q.vec().norm();
    Scalar x = q.x() / norm;
    Scalar y = q.y() / norm;
    Scalar z = q.z() / norm;
    Scalar phi = std::min<Scalar>( std::atan2<Scalar>( y, x ), M_PI );
    Scalar sin_theta, cos_theta = z;
    Scalar k = std::max(
        2.0, std::floor( std::log( fibonacci_n * M_PI * sqrt( 5 ) * ( 1.0 - square( cos_theta ) ) ) /
                         std::log( PHI + 1.0 ) ) );
    Scalar fk = pow( PHI, k ) / std::sqrt( 5 );
    Scalar f0 = std::round( fk );
    Scalar f1 = std::round( fk * PHI );
    Eigen::Matrix<Scalar, 2, 2> b;
    b << 2.0 * M_PI * ( detail::madfrac( f0 + 1.0, PHI - 1.0 ) - ( PHI - 1.0 ) ),
        2.0 * M_PI * ( detail::madfrac( f1 + 1.0, PHI - 1.0 ) - ( PHI - 1.0 ) ),
        -2.0 * f0 / fibonacci_n, -2.0 * f1 / fibonacci_n;
    Eigen::Matrix<Scalar, 2, 1> c =
        ( b.inverse() * Eigen::Matrix<Scalar, 2, 1>{ phi, cos_theta - ( 1.0 - 1.0 / fibonacci_n ) } );
    c = c.array().floor().matrix();
    Scalar d = std::numeric_limits<Scalar>::max();
    ReturnType j = 0;
    for ( int s = 0; s < 4; s++ ) {
      cos_theta = b.row( 1 ).dot( Eigen::Matrix<Scalar, 2, 1>{ s % 2, s / 2 } + c ) +
                  ( 1.0 - 1.0 / fibonacci_n );
      cos_theta = clamp( cos_theta, Scalar( -1.0 ), Scalar( 1.0 ) ) * 2.0 - cos_theta;

      ReturnType i = std::floor( fibonacci_n * 0.5 * ( 1.0 - cos_theta ) );
      phi = 2.0 * M_PI * detail::madfrac( (Scalar)i, Scalar( PHI - 1.0 ) );
      cos_theta = 1.0 - ( 2.0 * i + 1.0 ) / fibonacci_n;
      sin_theta = std::sqrt( 1 - std::min<Scalar>( cos_theta * cos_theta, 1.0 ) );

      Scalar squared_distance = square( x - std::cos( phi ) * sin_theta ) +
                                square( y - std::sin( phi ) * sin_theta ) + square( z - cos_theta );
      if ( squared_distance < d ) {
        d = squared_distance;
        j = i;
      }
    }
    return j;

  } else { // SPHERICAL

    const Scalar norm = q.vec().norm();
    Scalar theta = std::acos( q.z() / norm );
    ReturnType bin = ReturnType( theta * AXIS_BINS / M_PI ) & DIM_MASK;
    Scalar phi = std::atan2( q.y(), q.x() ) + M_PI; // Move from [-pi, pi] to [0, 2 * pi]
    bin |= ( ReturnType( phi * AXIS_BINS / ( 2 * M_PI ) ) & DIM_MASK ) << dim_bits;

    const Scalar angle = std::acos( q.w() );
    bin |= ( ReturnType( angle * ( ANGLE_BINS / M_PI ) ) & ANGLE_MASK ) << ( 2 * dim_bits );
    return bin;
  }
}
template<typename Scalar, int AXIS_BINS, int ANGLE_BINS, QuaternionBinningMode mode,
         typename ReturnType = typename detail::QuaternionBinType<AXIS_BINS, ANGLE_BINS>::BinType>
Eigen::Matrix<Scalar, 3, 1> computeDirectionFromBin( const ReturnType &i )
{
  if ( mode == quaternion_binning_modes::SphericalFibonacci ) {
    constexpr Scalar PHI = ( std::sqrt( 5 ) + Scalar( 1 ) ) / Scalar( 2 );
    int fibonacci_n = ANGLE_BINS * ANGLE_BINS;
    Scalar phi = 2 * M_PI * ( i / PHI - std::floor( i / PHI ) );
    Scalar cos_theta = 1.0 - ( 1.0 + 2.0 * i ) / fibonacci_n;
    Scalar sin_theta = std::sqrt( std::max( 0.0, 1.0 - pow( cos_theta, 2 ) ) );
    return { std::cos( phi ) * sin_theta, std::sin( phi ) * sin_theta, cos_theta };
  } else {
    throw std::runtime_error( "Not implemented." );
  }
}
} // namespace hector_math

#endif // HECTOR_MATH_QUATERNION_BINNING_H
