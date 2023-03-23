// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_RECTANGLE_ITERATOR_H
#define HECTOR_MATH_RECTANGLE_ITERATOR_H

#include "hector_math/containers/bounded_vector.h"
#include "hector_math/types.h"

namespace hector_math
{

/*!
 * Iterates over all indexes that lie in the rectangle formed by the three points a, b and c - where
 * ab, and ac form adjacent edges of the rectangle - and for each index (x, y) calls the
 * given functor. This method will iterate all cells where the center of the cell (x+0.5, y+0.5) is
 * inside the rectangle. Note: The polygon has to be in the index space, hence, if it is in map
 * coordinates it might be necessary to divide it by the map resolution.
 *
 * The indexes can be limited using the ranges [row_min, row_max) and [col_min, col_max) where
 * row/col_min is included but row/col_max is excluded, i.e., the largest x index functor may be
 * called with will be row_max - 1.
 *
 * *Note:* This method can actually iterate not only rectangles but also parallelograms.
 * Theoretically, it would work for any convex quadrilateral (alt. quadrangle).
 *
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
  iterateRectangle( a, b, c, 0, rows, 0, cols, functor );
}

//! Overload of iterateRectangle where the indexes are not bounded.
template<typename Scalar, typename Functor>
void iterateRectangle( const Vector2<Scalar> &a, const Vector2<Scalar> &b, const Vector2<Scalar> &c,
                       Functor functor )
{
  constexpr Eigen::Index min = std::numeric_limits<Eigen::Index>::min();
  constexpr Eigen::Index max = std::numeric_limits<Eigen::Index>::max();
  iterateRectangle( a, b, c, min, max, min, max, functor );
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
                                            []( const Vector2<Scalar> &a, const Vector2<Scalar> &b ) {
                                              if ( std::abs( a.y() - b.y() ) < 1E-5 )
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
  if ( ( highest.x() - lowest.x() ) * ( points[index_left].y() - lowest.y() ) <
       ( highest.y() - lowest.y() ) * ( points[index_left].x() - lowest.x() ) )
    std::swap( index_left, index_right );
  const auto &left = points[index_left];
  const auto &right = points[index_right];

  struct Line {
    Line( const Vector2<Scalar> &start, const Vector2<Scalar> &end, const Eigen::Index y,
          const bool left )
    {
      if ( std::abs( end.y() - start.y() ) < 1E-4 ) {
        x_increment = 0;
        x = left ? std::min( start.x(), end.x() ) : std::max( start.x(), end.x() );
        return;
      }
      x_increment = ( end.x() - start.x() ) / ( end.y() - start.y() );

      // Compute x value at center of y-column
      const Scalar diff_start_y =
          y - std::floor( start.y() ) + 0.5 - ( start.y() - std::floor( start.y() ) );
      x = start.x() + diff_start_y * x_increment;
    }

    Scalar x;
    Scalar x_increment;
  };

  Eigen::Index y = std::max<Eigen::Index>( col_min, std::round( lowest.y() ) );
  const Eigen::Index max_y = std::min<Eigen::Index>( col_max, std::round( highest.y() ) );
  const Eigen::Index left_switch = std::round( left.y() );
  const Eigen::Index right_switch = std::round( right.y() );
  Scalar left_x, right_x;
  Line left_line( lowest, left, y, true ), right_line( lowest, right, y, false );
  Eigen::Index next_y = std::min( left_switch, right_switch );
  // Loop until next corner
  for ( ; y < next_y; ++y ) {
    Eigen::Index x = std::max<Eigen::Index>( row_min, std::round( left_line.x ) );
    const Eigen::Index x_end = std::min<Eigen::Index>( row_max, std::round( right_line.x ) );
    left_line.x += left_line.x_increment;
    right_line.x += right_line.x_increment;
    for ( ; x < x_end; ++x ) { functor( x, y ); }
  }

  // Either left or right switched, if both, the for loop will have 0 iterations
  if ( y == left_switch ) {
    left_line = Line( left, highest, y, true );
    next_y = std::min( right_switch, max_y );
  } else if ( y == right_switch ) {
    right_line = Line( right, highest, y, false );
    next_y = std::min( left_switch, max_y );
  }

  // Loop until next corner
  for ( ; y < next_y; ++y ) {
    Eigen::Index x = std::max<Eigen::Index>( row_min, std::round( left_line.x ) );
    const Eigen::Index x_end = std::min<Eigen::Index>( row_max, std::round( right_line.x ) );
    left_line.x += left_line.x_increment;
    right_line.x += right_line.x_increment;
    for ( ; x < x_end; ++x ) { functor( x, y ); }
  }
  // Final switch, inverted order since if both switch at the same y, now right takes precedence
  if ( y == right_switch ) {
    right_line = Line( right, highest, y, false );
  } else if ( y == left_switch ) {
    left_line = Line( left, highest, y, true );
  }

  // Loop until end
  for ( ; y < max_y; ++y ) {
    Eigen::Index x = std::max<Eigen::Index>( row_min, std::round( left_line.x ) );
    const Eigen::Index x_end = std::min<Eigen::Index>( row_max, std::round( right_line.x ) );
    left_line.x += left_line.x_increment;
    right_line.x += right_line.x_increment;
    for ( ; x < x_end; ++x ) { functor( x, y ); }
  }
}
} // namespace hector_math
#endif // HECTOR_MATH_RECTANGLE_ITERATOR_H
