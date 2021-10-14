//
// Created by stefan on 07.09.21.
//

#ifndef HECTOR_MATH_FIND_MINMAX_H
#define HECTOR_MATH_FIND_MINMAX_H

#include "hector_math/iterators/polygon_iterator.h"
#include "hector_math/types.h"

namespace hector_math
{

/*!
 * Finds the minimum value in the map inside the given polygon.
 * @tparam Scalar The floating point type that is used.
 * @param map The GridMap in which we are looking for the minimum value.
 * @param polygon The region in which we are looking for the minimum map value. Needs to be in map coordinates.
 * @return The minimum value or NaN if the entire polygon is outside of the map or the map has no non-NaN value inside the polygon.
 */
template<typename Scalar>
Scalar findMinimum( const Eigen::Ref<const GridMap<Scalar>> &map, const Polygon<Scalar> &polygon )
{
  Scalar minimum = std::numeric_limits<Scalar>::has_quiet_NaN ? std::numeric_limits<Scalar>::quiet_NaN()
                                                              : std::numeric_limits<Scalar>::max();
  iteratePolygon( polygon, map.rows(), map.cols(), [ & ]( Eigen::Index x, Eigen::Index y )
  {
    const Scalar &val = map( x, y );
    if ( val >= minimum ) return; // Will be false for NaN
    minimum = val;
  } );
  return minimum;
}

/*!
 * Finds the maximum value in the map inside the given polygon.
 * @tparam Scalar The floating point type that is used.
 * @param map The GridMap in which we are looking for the maximum value.
 * @param polygon The region in which we are looking for the maximum map value. Needs to be in map coordinates.
 * @return The maximum value or NaN if the entire polygon is outside of the map or the map has no non-NaN value inside the polygon.
 */
template<typename Scalar>
Scalar findMaximum( const Eigen::Ref<const GridMap<Scalar>> &map, const Polygon<Scalar> &polygon )
{
  Scalar maximum = std::numeric_limits<Scalar>::has_quiet_NaN ? std::numeric_limits<Scalar>::quiet_NaN()
                                                              : std::numeric_limits<Scalar>::min();
  iteratePolygon( polygon, map.rows(), map.cols(), [ & ]( Eigen::Index x, Eigen::Index y )
  {
    const Scalar &val = map( x, y );
    if ( val <= maximum ) return; // Will be false for NaN
    maximum = val;
  } );
  return maximum;
}
}

#endif //HECTOR_MATH_FIND_MINMAX_H
