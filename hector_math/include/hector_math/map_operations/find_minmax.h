// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_FIND_MINMAX_H
#define HECTOR_MATH_FIND_MINMAX_H

#include "hector_math/iterators/polygon_iterator.h"
#include "hector_math/types.h"

namespace hector_math
{

/*!
 * Finds the minimum value in the map and the location in the form of row and col index.
 * This method is robust against NaN values in the map.
 *
 * @tparam Scalar The floating point type that is used.
 * @param map The GridMap in which we are looking for the minimum value.
 * @param row The row index of the minimum.
 * @param col The column index of the minimum.
 * @return The minimum value inside the map.
 */
template<typename EigenType>
typename EigenType::Scalar findMinimumAndIndex( const EigenType &map, Eigen::Index &row,
                                                Eigen::Index &col );

/*!
 * Finds the maximum value in the map and the location in the form of row and col index.
 * This method is robust against NaN values in the map.
 *
 * @tparam Scalar The floating point type that is used.
 * @param map The GridMap in which we are looking for the maximum value.
 * @param row The row index of the maximum.
 * @param col The column index of the maximum.
 * @return The maximum value inside the map.
 */
template<typename EigenType>
typename EigenType::Scalar findMaximumAndIndex( const EigenType &map, Eigen::Index &row,
                                                Eigen::Index &col );

/*!
 * Finds the minimum value in the map inside the given polygon.
 * This method is robust against NaN values in the map.
 *
 * @tparam Scalar The floating point type that is used.
 * @param map The GridMap in which we are looking for the minimum value.
 * @param polygon The region in which we are looking for the minimum map value. Needs to be in map coordinates.
 * @return The minimum value or NaN if the entire polygon is outside of the map or the map has no non-NaN value inside the polygon.
 */
template<typename Scalar>
Scalar findMinimum( const Eigen::Ref<const GridMap<Scalar>> &map, const Polygon<Scalar> &polygon );

/*!
 * Finds the maximum value in the map inside the given polygon.
 * This method is robust against NaN values in the map.
 *
 * @tparam Scalar The floating point type that is used.
 * @param map The GridMap in which we are looking for the maximum value.
 * @param polygon The region in which we are looking for the maximum map value. Needs to be in map coordinates.
 * @return The maximum value or NaN if the entire polygon is outside of the map or the map has no non-NaN value inside the polygon.
 */
template<typename Scalar>
Scalar findMaximum( const Eigen::Ref<const GridMap<Scalar>> &map, const Polygon<Scalar> &polygon );

namespace impl
{
template<typename Scalar, typename std::enable_if<std::numeric_limits<Scalar>::has_quiet_NaN, int>::type = 0>
constexpr Scalar initialMinimum()
{
  return std::numeric_limits<Scalar>::quiet_NaN();
}

template<typename Scalar,
         typename std::enable_if<!std::numeric_limits<Scalar>::has_quiet_NaN, int>::type = 0>
constexpr Scalar initialMinimum()
{
  return std::numeric_limits<Scalar>::max();
}

template<typename Scalar, typename std::enable_if<std::numeric_limits<Scalar>::has_quiet_NaN, int>::type = 0>
constexpr Scalar initialMaximum()
{
  return std::numeric_limits<Scalar>::quiet_NaN();
}

template<typename Scalar,
         typename std::enable_if<!std::numeric_limits<Scalar>::has_quiet_NaN, int>::type = 0>
constexpr Scalar initialMaximum()
{
  return std::numeric_limits<Scalar>::min();
}
} // namespace impl

template<typename EigenType>
typename EigenType::Scalar findMinimumAndIndex( const EigenType &map, Eigen::Index &row,
                                                Eigen::Index &col )
{
  using Scalar = typename EigenType::Scalar;
  Scalar minimum = impl::initialMinimum<Scalar>();
  for ( Eigen::Index y = 0; y < map.cols(); ++y ) {
    for ( Eigen::Index x = 0; x < map.rows(); ++x ) {
      const Scalar &val = map( x, y );
      if ( std::isnan( val ) || val >= minimum )
        continue; // Will be false for NaN
      minimum = val;
      row = x;
      col = y;
    }
  }
  return minimum;
}

template<typename EigenType>
typename EigenType::Scalar findMaximumAndIndex( const EigenType &map, Eigen::Index &row,
                                                Eigen::Index &col )
{
  using Scalar = typename EigenType::Scalar;
  Scalar maximum = impl::initialMaximum<Scalar>();
  for ( Eigen::Index y = 0; y < map.cols(); ++y ) {
    for ( Eigen::Index x = 0; x < map.rows(); ++x ) {
      const Scalar &val = map( x, y );
      if ( std::isnan( val ) || val <= maximum )
        continue; // Will be false for NaN
      maximum = val;
      row = x;
      col = y;
    }
  }
  return maximum;
}

template<typename Scalar>
Scalar findMinimum( const Eigen::Ref<const GridMap<Scalar>> &map, const Polygon<Scalar> &polygon )
{
  Scalar minimum = impl::initialMinimum<Scalar>();
  iteratePolygon( polygon, map.rows(), map.cols(), [&]( Eigen::Index x, Eigen::Index y ) {
    const Scalar &val = map( x, y );
    if ( std::isnan( val ) || val >= minimum )
      return; // Will be false for NaN
    minimum = val;
  } );
  return minimum;
}

template<typename Scalar>
Scalar findMaximum( const Eigen::Ref<const GridMap<Scalar>> &map, const Polygon<Scalar> &polygon )
{
  Scalar maximum = impl::initialMaximum<Scalar>();
  iteratePolygon( polygon, map.rows(), map.cols(), [&]( Eigen::Index x, Eigen::Index y ) {
    const Scalar &val = map( x, y );
    if ( std::isnan( val ) || val <= maximum )
      return; // Will be false for NaN
    maximum = val;
  } );
  return maximum;
}
} // namespace hector_math

#endif // HECTOR_MATH_FIND_MINMAX_H
