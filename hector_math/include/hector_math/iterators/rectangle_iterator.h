//
// Created by stefan on 11.11.21.
//

#ifndef HECTOR_MATH_RECTANGLE_ITERATOR_H
#define HECTOR_MATH_RECTANGLE_ITERATOR_H

#include "hector_math/containers/bounded_vector.h"
#include "hector_math/types.h"
#include <vector>

namespace hector_math
{

/*!
 * Iterates over all indexes that lie in the given rectangle and for each index (x, y) calls the
 * given functor. This method will iterate all cells where the center of the cell (x+0.5, y+0.5) is
 * inside the rectangle. Note: The polygon has to be in the index space, hence, if it is in map
 * coordinates it might be necessary to divide it by the map resolution.
 *
 * The indexes can be limited using the ranges [row_min, row_max) and [col_min, col_max) where
 * row/col_min is included but row/col_max is excluded, i.e., the largest x index functor may be
 * called with will be row_max - 1.
 * @tparam Functor A function or lambda method with the signature: void(Eigen::Index x, Eigen::Index
 * y).
 * @param polygon The polygon that is iterated over.
 * @param functor The function that will be called for each index (x, y) inside the polygon.
 */
template<typename Scalar, typename Functor>
void iterateRectangle( const Vector2<Scalar> &a, const Vector2<Scalar> &b, const Vector2<Scalar> &c,
                       Eigen::Index row_min, Eigen::Index row_max, Eigen::Index col_min,
                       Eigen::Index col_max, Functor functor );

//! Overload of iterateRectangle where row_min and col_min are set to 0 to allow for bounded
//! iteration of 2D matrices and arrays.
template<typename Scalar, typename Functor>
void iterateRectangle( const Vector2<Scalar> &a, const Vector2<Scalar> &b, const Vector2<Scalar> &c,
                       Eigen::Index rows, Eigen::Index cols, Functor functor )
{
  iteratePolygon( polygon, 0, rows, 0, cols, functor );
}

//! Overload of iterateRectangle where the indexes are not bounded.
template<typename Scalar, typename Functor>
void iterateRectangle( const Vector2<Scalar> &a, const Vector2<Scalar> &b, const Vector2<Scalar> &c,
                       Functor functor )
{
  constexpr Eigen::Index min = std::numeric_limits<Eigen::Index>::min();
  constexpr Eigen::Index max = std::numeric_limits<Eigen::Index>::max();
  iteratePolygon( polygon, min, max, min, max, functor );
}

template<typename Scalar, typename Functor>
void iterateRectangle( const Vector2<Scalar> &a, const Vector2<Scalar> &b, const Vector2<Scalar> &c,
                       Eigen::Index row_min, Eigen::Index row_max, Eigen::Index col_min,
                       Eigen::Index col_max, Functor functor )
{
  const auto &d = b + c - a;

  std::array<Vector2<Scalar>, 4> points = { a, b, d, c };
  // Find the corner with the lowest y value
  size_t smallest_index = std::min_element( points.begin(), points.end(),
                                            []( const MatrixType &a, const MatrixType &b ) {
                                              if ( std::abs( a.y() - b.y() ) < 1E-9 )
                                                return a.x() < b.x();
                                              return a.y() < b.y();
                                            } ) -
                          points.begin();
  // Given the structure we can directly determine the highest y-corner and by comparing the other
  // two corners obtain the left- and rightmost corner
  const auto &lowest = points[smallest_index];
  const auto &highest = points[smallest_index >= 2 ? smallest_index - 2 : smallest_index + 2];
  size_t index_left = smallest_index == 0 ? smallest_index + 3 : smallest_index - 1;
  size_t index_right = smallest_index == 3 ? 0 : smallest_index + 1;
  // The sign of the determinant of the vectors (lowest,highest) and (lowest,left) is negative if
  // left is on the right of the line from lowest to highest. In that case, swap the indices.
  if ( ( highest_.x() - lowest_.x() ) * ( points[index_left].y() - lowest_.y() ) <
       ( highest_.y() - lowest_.y() ) * ( points[index_left].x() - lowest_.x() ) )
    std::swap( index_left, index_right );
  const auto &left = points[index_left];
  const auto &right = points[index_right];
}

namespace detail
{
} // namespace detail
} // namespace hector_math
#endif // HECTOR_MATH_RECTANGLE_ITERATOR_H
