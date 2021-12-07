//
// Created by stefan on 01.12.21.
//

#ifndef HECTOR_MATH_QUATERNION_CACHE_H
#define HECTOR_MATH_QUATERNION_CACHE_H

#include "hector_math/approximations/trigonometry.h"
#include <Eigen/Geometry>
#include <unordered_map>

namespace hector_math
{

template<typename Scalar, typename T, int AXIS_BINS = 128, int ANGLE_COUNT = 512>
class QuaternionCache
{
  static_assert( AXIS_BINS % 2 == 0, "AXIS_BINS should be an even number!" );
  static constexpr int _required_bits = 2 * std::log2( AXIS_BINS ) + 1 + std::log2( ANGLE_COUNT ) + 2;
  using BinType = typename std::conditional<_required_bits >= 8 * sizeof( int ), long, int>::type;
  static constexpr int dim_bits = std::log( AXIS_BINS ) + 1;

public:
  using iterator = typename std::unordered_map<BinType, T>::iterator;
  using const_iterator = typename std::unordered_map<BinType, T>::const_iterator;

  void insert( const Eigen::Quaternion<Scalar> &q, const T &&val ) noexcept
  {
    cache_.insert( { computeBin( q ), val } );
  }

  iterator find( const Eigen::Quaternion<Scalar> &q ) noexcept
  {
    return cache_.find( computeBin( q ) );
  }

  const_iterator find( const Eigen::Quaternion<Scalar> &q ) const noexcept
  {
    return cache_.find( computeBin( q ) );
  }

  iterator begin() noexcept { return cache_.begin(); }
  const_iterator begin() const noexcept { return cache_.begin(); }

  const_iterator cbegin() const noexcept { return cache_.cbegin(); }

  iterator end() noexcept { return cache_.end(); }
  const_iterator end() const noexcept { return cache_.end(); }

  const_iterator cend() const noexcept { return cache_.cend(); }

private:
  static BinType computeBin( const Eigen::Quaternion<Scalar> &q ) noexcept
  {
    // First resolve conflict of -q and q representing the same orientation.
    if ( std::signbit( q.w() ) )
      return computeBin( Eigen::Quaternion<Scalar>( -q.w(), -q.x(), -q.y(), -q.z() ) );

    const Scalar x2 = q.x() * q.x();
    const Scalar y2 = q.y() * q.y();
    const Scalar z2 = q.z() * q.z();
    BinType bin = 0;
    constexpr Scalar MULTIPLIER = M_SQRT2 * AXIS_BINS / 2;
    // +0.5 for rounding since we moved it to positive domain
    constexpr Scalar OFFSET = Scalar( AXIS_BINS ) / 2 + 0.5;
    // Find the largest component for stable binning.
    if ( x2 > y2 && x2 > z2 ) {
      bin = 0b01;
      if ( q.x() < 0 )
        bin |= 0b100;
      const Scalar norm = std::sqrt( x2 + y2 + z2 );
      // Since x is largest, y and z are absolute smaller than sqrt(2)/2
      bin |= BinType( q.y() * MULTIPLIER / norm + OFFSET ) << 3;
      bin |= BinType( q.z() * MULTIPLIER / norm + OFFSET ) << ( 3 + dim_bits );
    } else if ( y2 > z2 ) {
      bin = 0b10;
      if ( q.y() < 0 )
        bin |= 0b100;
      const Scalar norm = std::sqrt( x2 + y2 + z2 );
      bin |= BinType( q.x() * MULTIPLIER / norm + OFFSET ) << 3;
      bin |= BinType( q.z() * MULTIPLIER / norm + OFFSET ) << ( 3 + dim_bits );
    } else {
      bin = 0b11;
      if ( q.z() < 0 )
        bin |= 0b100;
      const Scalar norm = std::sqrt( x2 + y2 + z2 );
      bin |= BinType( q.y() * MULTIPLIER / norm + OFFSET ) << 3;
      bin |= BinType( q.x() * MULTIPLIER / norm + OFFSET ) << ( 3 + dim_bits );
    }
    const Scalar angle = acosApproximate( q.w() );
    bin |= static_cast<BinType>( ( angle * ( ANGLE_COUNT / M_PI ) ) + 0.5 ) << ( 2 * dim_bits + 3 );
    return bin;
  }

  std::unordered_map<BinType, T> cache_;
};
} // namespace hector_math

#endif // HECTOR_MATH_QUATERNION_CACHE_H
