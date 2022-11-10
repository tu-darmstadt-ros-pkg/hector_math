//
// Created by stefan on 01.12.21.
//

#ifndef HECTOR_MATH_QUATERNION_CACHE_H
#define HECTOR_MATH_QUATERNION_CACHE_H

#include "hector_math/math/quaternion_binning.h"
#include <unordered_map>

namespace hector_math
{

template<typename Scalar, typename T, QuaternionBinningMode MODE = quaternion_binning_modes::LargestDim,
         int AXIS_BINS = 128, int ANGLE_COUNT = 512>
class QuaternionCache
{
  static_assert( AXIS_BINS % 2 == 0, "AXIS_BINS should be an even number!" );
  using BinType = typename detail::QuaternionBinType<AXIS_BINS, ANGLE_COUNT>::BinType;

public:
  using iterator = typename std::unordered_map<BinType, T>::iterator;
  using const_iterator = typename std::unordered_map<BinType, T>::const_iterator;

  std::pair<iterator, bool> insert( const Eigen::Quaternion<Scalar> &q, const T &val ) noexcept
  {
    return cache_.insert( { computeBin( q ), val } );
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

  void clear() noexcept { cache_.clear(); }

  static BinType computeBin( const Eigen::Quaternion<Scalar> &q ) noexcept
  {
    return hector_math::computeBin<Scalar, AXIS_BINS, ANGLE_COUNT, MODE, BinType>( q );
  }

private:
  std::unordered_map<BinType, T> cache_;
};
} // namespace hector_math

#endif // HECTOR_MATH_QUATERNION_CACHE_H
