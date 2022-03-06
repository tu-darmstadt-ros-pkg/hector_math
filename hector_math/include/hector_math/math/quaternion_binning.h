//
// Created by stefan on 10.12.21.
//

#ifndef HECTOR_MATH_QUATERNION_BINNING_H
#define HECTOR_MATH_QUATERNION_BINNING_H

#include <Eigen/Geometry>

namespace hector_math
{
template<int VECTOR_BINS, int ANGLE_BINS>
struct RequiredBinType {
  static constexpr int _required_bits =
      2 * std::log2( VECTOR_BINS ) + 1 + std::log2( ANGLE_BINS ) + 2;
  using BinType = typename std::conditional<_required_bits >= 8 * sizeof( int ), long, int>::type;
};

namespace quaternion_binning_modes
{
//! You can visualize the different binning modes using visualize_binning.py
//! LargestDim is somewhat faster and doesn't have a reduced resolution at the poles due to the
//! equidistant angles used in spherical but at the cost of overlapping regions with increased
//! bin resolution
enum QuaternionBinningMode { LargestDim, Spherical };
} // namespace quaternion_binning_modes
using QuaternionBinningMode = quaternion_binning_modes::QuaternionBinningMode;

/*!
 * Computes the bin for the given quaternion.
 * Close quaternions should land in the same bin.
 *
 * @tparam VECTOR_BINS
 * @tparam ANGLE_BINS
 * @tparam mode
 * @tparam Scalar
 * @tparam ReturnType
 * @param q
 * @return
 */
template<typename Scalar, int VECTOR_BINS, int ANGLE_BINS, QuaternionBinningMode mode,
         typename ReturnType = typename RequiredBinType<VECTOR_BINS, ANGLE_BINS>::BinType>
ReturnType computeBin( const Eigen::Quaternion<Scalar> &q )
{
  static constexpr int dim_bits = std::log( VECTOR_BINS ) + 1;
  // First resolve conflict of -q and q representing the same orientation.
  if ( std::signbit( q.w() ) ) {
    return computeBin<Scalar, VECTOR_BINS, ANGLE_BINS, mode, ReturnType>(
        Eigen::Quaternion<Scalar>( -q.w(), -q.x(), -q.y(), -q.z() ) );
  }

  if ( mode == quaternion_binning_modes::LargestDim ) {
    const Scalar x2 = q.x() * q.x();
    const Scalar y2 = q.y() * q.y();
    const Scalar z2 = q.z() * q.z();
    ReturnType bin = 0;
    constexpr Scalar MULTIPLIER = M_SQRT2 * VECTOR_BINS / 2;
    // +0.5 for rounding since we moved it to positive domain
    constexpr Scalar OFFSET = Scalar( VECTOR_BINS ) / 2 + 0.5;
    // Find the largest component for stable binning.
    if ( x2 > y2 && x2 > z2 ) {
      bin = 0b01;
      if ( q.x() < 0 )
        bin |= 0b100;
      const Scalar norm = std::sqrt( x2 + y2 + z2 );
      // Since x is largest, y and z are absolute smaller than sqrt(2)/2
      bin |= ReturnType( q.y() * MULTIPLIER / norm + OFFSET ) << 3;
      bin |= ReturnType( q.z() * MULTIPLIER / norm + OFFSET ) << ( 3 + dim_bits );
    } else if ( y2 > z2 ) {
      bin = 0b10;
      if ( q.y() < 0 )
        bin |= 0b100;
      const Scalar norm = std::sqrt( x2 + y2 + z2 );
      bin |= ReturnType( q.x() * MULTIPLIER / norm + OFFSET ) << 3;
      bin |= ReturnType( q.z() * MULTIPLIER / norm + OFFSET ) << ( 3 + dim_bits );
    } else {
      bin = 0b11;
      if ( q.z() < 0 )
        bin |= 0b100;
      const Scalar norm = std::sqrt( x2 + y2 + z2 );
      bin |= ReturnType( q.y() * MULTIPLIER / norm + OFFSET ) << 3;
      bin |= ReturnType( q.x() * MULTIPLIER / norm + OFFSET ) << ( 3 + dim_bits );
    }
    const Scalar angle = std::acos( q.w() );
    bin |= static_cast<ReturnType>( ( angle * ( ANGLE_BINS / M_PI ) ) + 0.5 ) << ( 2 * dim_bits + 3 );
    return bin;

  } else { // SPHERICAL

    const Scalar norm = q.vec().norm();
    Scalar theta = std::acos( q.z() / norm );
    if ( theta < 0 )
      theta += 2 * M_PI;
    ReturnType bin = ReturnType( theta * VECTOR_BINS / M_PI + 0.5 );
    Scalar phi = std::atan2( q.y(), q.x() );
    if ( phi < 0 )
      phi += 2 * M_PI;
    bin |= ReturnType( phi * VECTOR_BINS / M_PI + 0.5 ) << dim_bits;

    const Scalar angle = std::acos( q.w() );
    bin |= static_cast<ReturnType>( ( angle * ( ANGLE_BINS / M_PI ) ) + 0.5 ) << ( 2 * dim_bits );
    return bin;
  }
}
} // namespace hector_math

#endif // HECTOR_MATH_QUATERNION_BINNING_H
