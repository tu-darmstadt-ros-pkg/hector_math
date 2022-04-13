//
// Created by stefan on 15.09.21.
//

#ifndef HECTOR_MATH_CIRCLE_ITERATOR_H
#define HECTOR_MATH_CIRCLE_ITERATOR_H

#include "hector_math/types.h"

namespace hector_math
{
/*!
 * Iterates over all indexes that lie in the given circle and for each index (x, y) calls the given functor.
 * This method will iterate all cells where the center of the cell (x+0.5, y+0.5) is inside the circle.
 * Note: The circle has to be in the index space, hence, if it is in map coordinates it might be necessary to divide
 *   it by the map resolution.
 *
 * The indexes can be limited using the ranges [row_min, row_max) and [col_min, col_max) where row/col_min is included
 * but row/col_max is excluded, i.e., the largest x index functor may be called with will be row_max - 1.
 * @tparam Functor A function or lambda method with the signature: void(Eigen::Index x, Eigen::Index y).
 * @param center The center of the circle that is iterated over.
 * @param radius The radius of the circle that is iterated over.
 * @param functor The function that will be called for each index (x, y) inside the polygon.
 */
template<typename Scalar, typename Functor>
void iterateCircle( const Vector2<Scalar> &center, Scalar radius, Eigen::Index row_min,
                    Eigen::Index row_max, Eigen::Index col_min, Eigen::Index col_max,
                    Functor functor );

//! Overload of iterateCircle where row_min and col_min are set to 0 to allow for bounded iteration
//! of 2D matrices and arrays.
template<typename Scalar, typename Functor>
void iterateCircle( const Vector2<Scalar> &center, Scalar radius, Eigen::Index rows,
                    Eigen::Index cols, Functor functor )
{
  iterateCircle( center, radius, 0, rows, 0, cols, functor );
}

//! Overload of iterateCircle where the indexes are not bounded.
template<typename Scalar, typename Functor>
void iterateCircle( const Vector2<Scalar> &center, Scalar radius, Functor functor )
{
  constexpr Eigen::Index min = std::numeric_limits<Eigen::Index>::min();
  constexpr Eigen::Index max = std::numeric_limits<Eigen::Index>::max();
  iterateCircle( center, radius, min, max, min, max, functor );
}

template<typename Scalar, typename Functor>
void iterateCircle( const Vector2<Scalar> &center, Scalar radius, Eigen::Index row_min,
                    Eigen::Index row_max, Eigen::Index col_min, Eigen::Index col_max, Functor functor )
{
  const Eigen::Index min_y = std::max<Eigen::Index>( col_min, std::round( center.y() - radius ) );
  const Eigen::Index max_y = std::min<Eigen::Index>( col_max, std::round( center.y() + radius ) );
  const Scalar radius_squared = radius * radius;
  for ( Eigen::Index y = min_y; y < max_y; ++y ) {
    // Formula of a circle is: r^2 = x^2 + y^2 ==> x = +/- sqrt(r^2 - y^2) which we can use to get min x and max x
    const Scalar delta_y = y + 0.5 - center.y();
    const Scalar width = std::sqrt( radius_squared - delta_y * delta_y );
    const Eigen::Index min_x = std::max<Eigen::Index>( row_min, std::round( center.x() - width ) );
    const Eigen::Index max_x = std::min<Eigen::Index>( row_max, std::round( center.x() + width ) );
    for ( Eigen::Index x = min_x; x < max_x; ++x ) { functor( x, y ); }
  }
}
} // namespace hector_math

#endif // HECTOR_MATH_CIRCLE_ITERATOR_H
