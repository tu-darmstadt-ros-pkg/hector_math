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
template<typename T, typename Functor>
void iterateRectangle( const Vector2<T> &a, const Vector2<T> &b, const Vector2<T> &c,
                       Eigen::Index row_min, Eigen::Index row_max, Eigen::Index col_min,
                       Eigen::Index col_max, Functor functor );

//! Overload of iterateRectangle where row_min and col_min are set to 0 to allow for bounded
//! iteration of 2D matrices and arrays.
template<typename T, typename Functor>
void iterateRectangle( const Vector2<T> &a, const Vector2<T> &b, const Vector2<T> &c,
                       Eigen::Index rows, Eigen::Index cols, Functor functor )
{
  iterateRectangle( a, b, c, 0, rows, 0, cols, functor );
}

//! Overload of iterateRectangle where the indexes are not bounded.
template<typename T, typename Functor>
void iterateRectangle( const Vector2<T> &a, const Vector2<T> &b, const Vector2<T> &c, Functor functor )
{
  constexpr Eigen::Index min = std::numeric_limits<Eigen::Index>::min();
  constexpr Eigen::Index max = std::numeric_limits<Eigen::Index>::max();
  iterateRectangle( a, b, c, min, max, min, max, functor );
}

namespace detail
{
struct RectangleData {
  Vector2d lowest;
  Vector2d highest;
  Vector2d left;
  Vector2d right;
  Eigen::Index left_switch;
  Eigen::Index right_switch;
};
} // namespace detail

template<bool COLUMN_MAJOR = true>
class RectangleIndexIterator
{
public:
  struct Line;
  //! Overload iterating the given rectangle without any index_ limits.
  //! @see RectangleIndexIterator::RectangleIndexIterator(const Vector2d &, const Vector2d &, const Vector2d, Eigen::Index, Eigen::Index, Eigen::Index, Eigen::Index)
  RectangleIndexIterator( const Vector2d &a, const Vector2d &b, const Vector2d &c )
      : RectangleIndexIterator( a, b, c, std::numeric_limits<Eigen::Index>::lowest(),
                                std::numeric_limits<Eigen::Index>::max(),
                                std::numeric_limits<Eigen::Index>::lowest(),
                                std::numeric_limits<Eigen::Index>::max() )
  {
  }

  //! Overload where the index_ limits are taken from the given Eigen dense base (array/matrix).
  //! @see RectangleIndexIterator::RectangleIndexIterator(const Vector2d &, const Vector2d &, const Vector2d &, Eigen::Index, Eigen::Index, Eigen::Index, Eigen::Index)
  template<typename Derived>
  RectangleIndexIterator( const Eigen::DenseBase<Derived> &array, const Vector2d &a,
                          const Vector2d &b, const Vector2d &c )
      : RectangleIndexIterator( a, b, c, 0, array.rows(), 0, array.cols() )
  {
  }

  //! Overload where min x and y are 0 and max x and y are the given values.
  //! @see RectangleIndexIterator::RectangleIndexIterator(const Vector2d &, const Vector2d &, const Vector2d &, Eigen::Index, Eigen::Index, Eigen::Index, Eigen::Index)
  RectangleIndexIterator( const Vector2d &a, const Vector2d &b, const Vector2d &c,
                          Eigen::Index x_max, Eigen::Index y_max )
      : RectangleIndexIterator( a, b, c, 0, x_max, 0, y_max )
  {
  }

  /*!
   * Iterates over the given Rectangle in the form of the three points a, b and c where ab and ac
   * form adjacent edges of the rectangle. The rectangle is then defined by the points a, b, d and c
   * where d is computed from a, b and c as the corner opposite to a.
   *
   * @verbatim
   * a ---- b
   * |      |
   * c ---- d
   * @endverbatim
   *
   * Using x/y_min and x/y_max, the iteration can be limited to a specific range within the rectangle.
   *
   * The limits are given as [x_min, x_max) and [y_min, y_max) where x_min and y_min are included
   * but x_max and y_max are excluded, i.e., the largest x index_ that will be iterated over is x_max - 1.
   *
   * @param a The first corner of the rectangle.
   * @param b The second corner of the rectangle which has a direct connection to a.
   * @param c The third corner of the rectangle which has a direct connection to a.
   * @param x_min The minimum x index_ that will be iterated over.
   * @param x_max The maximum x index_ that will be iterated over. This index_ is excluded.
   * @param y_min The minimum y index_ that will be iterated over.
   * @param y_max The maximum y index_ that will be iterated over. This index_ is excluded.
   */
  RectangleIndexIterator( const Vector2d &a, const Vector2d &b, const Vector2d &c, Eigen::Index x_min,
                          Eigen::Index x_max, Eigen::Index y_min, Eigen::Index y_max )
      : row_min_( x_min ), row_max_( x_max ), col_min_( y_min ), col_max_( y_max )
  {
    assert( x_min < x_max && y_min < y_max );
    const auto &d = ( b + c - a ).eval();
    std::array<Vector2d, 4> points = { a, b, d, c };
    // Find the corner with the lowest y value for column major and the lowest x value for row major
    size_t smallest_index = std::min_element( points.begin(), points.end(),
                                              []( const Vector2d &a, const Vector2d &b ) {
                                                if constexpr ( COLUMN_MAJOR ) {
                                                  if ( std::abs( a.y() - b.y() ) < 1E-5 )
                                                    return a.x() < b.x();
                                                  return a.y() < b.y();
                                                } else {
                                                  if ( std::abs( a.x() - b.x() ) < 1E-5 )
                                                    return a.y() < b.y();
                                                  return a.x() < b.x();
                                                }
                                              } ) -
                            points.begin();
    // Given the structure we can directly determine the highest y(/x)-corner and by comparing the
    // other two corners obtain the left- and rightmost corner
    data_.lowest = points[smallest_index];
    data_.highest = points[smallest_index >= 2 ? smallest_index - 2 : smallest_index + 2];
    size_t index_left = smallest_index == 0 ? smallest_index + 3 : smallest_index - 1;
    size_t index_right = smallest_index == 3 ? 0 : smallest_index + 1;
    // The sign of the determinant of the vectors (lowest,highest) and (lowest,left) is negative if
    // left is on the right of the line from lowest to highest. In that case, swap the indices.
    if ( ( data_.highest.x() - data_.lowest.x() ) * ( points[index_left].y() - data_.lowest.y() ) <
         ( data_.highest.y() - data_.lowest.y() ) * ( points[index_left].x() - data_.lowest.x() ) )
      std::swap( index_left, index_right );
    // For row major this is reversed
    if constexpr ( !COLUMN_MAJOR )
      std::swap( index_left, index_right );
    data_.left = points[index_left];
    data_.right = points[index_right];
    if constexpr ( COLUMN_MAJOR ) {
      col_min_ = std::max<Eigen::Index>( y_min, std::round( data_.lowest.y() ) );
      col_max_ = std::min<Eigen::Index>( y_max, std::round( data_.highest.y() ) );
      data_.left_switch = std::round( data_.left.y() );
      data_.right_switch = std::round( data_.right.y() );
    } else {
      row_min_ = std::max<Eigen::Index>( x_min, std::round( data_.lowest.x() ) );
      row_max_ = std::min<Eigen::Index>( x_max, std::round( data_.highest.x() ) );
      data_.left_switch = std::round( data_.left.x() );
      data_.right_switch = std::round( data_.right.x() );
    }
  }

  struct iterator;

  iterator begin();
  iterator end();

private:
  detail::RectangleData data_;
  Eigen::Index row_min_;
  Eigen::Index row_max_;
  Eigen::Index col_min_;
  Eigen::Index col_max_;
};

template<typename T, typename Functor>
void iterateRectangle( const Vector2<T> &a, const Vector2<T> &b, const Vector2<T> &c,
                       Eigen::Index row_min, Eigen::Index row_max, Eigen::Index col_min,
                       Eigen::Index col_max, Functor functor )
{
  const auto &d = ( b + c - a ).eval();

  std::array<Vector2<T>, 4> points = { a, b, d, c };
  // Find the corner with the lowest y value
  size_t smallest_index = std::min_element( points.begin(), points.end(),
                                            []( const Vector2<T> &a, const Vector2<T> &b ) {
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
    Line( const Vector2<T> &start, const Vector2<T> &end, const Eigen::Index y, const bool left )
    {
      if ( std::abs( end.y() - start.y() ) < 1E-4 ) {
        x_increment = 0;
        x = double( left ? std::min( start.x(), end.x() ) : std::max( start.x(), end.x() ) );
        return;
      }
      x_increment = double( end.x() - start.x() ) / double( end.y() - start.y() );

      // Compute x value at center of y-column
      const double diff_start_y =
          y - std::floor( start.y() ) + 0.5 - ( start.y() - std::floor( start.y() ) );
      x = start.x() + diff_start_y * x_increment;
    }

    double x;
    double x_increment;
  };

  Eigen::Index y = std::max<Eigen::Index>( col_min, std::round( lowest.y() ) );
  const Eigen::Index max_y = std::min<Eigen::Index>( col_max, std::round( highest.y() ) );
  const Eigen::Index left_switch = std::round( left.y() );
  const Eigen::Index right_switch = std::round( right.y() );
  Line left_line( lowest, left, y, true );
  Line right_line( lowest, right, y, false );
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

template<>
struct RectangleIndexIterator<true>::Line {
  Line( const Vector2d &start, const Vector2d &end, const Eigen::Index y, const bool left )
  {
    if ( std::abs( end.y() - start.y() ) < 1E-4 ) {
      increment = 0;
      x = double( left ? std::min( start.x(), end.x() ) : std::max( start.x(), end.x() ) );
      return;
    }
    increment = double( end.x() - start.x() ) / double( end.y() - start.y() );

    // Compute x value at center of y-column
    const double diff_start_y =
        double( y ) - std::floor( start.y() ) + 0.5 - ( start.y() - std::floor( start.y() ) );
    x = start.x() + diff_start_y * increment;
  }

  double x;
  double increment;
};

template<>
struct RectangleIndexIterator<false>::Line {
  Line( const Vector2d &start, const Vector2d &end, const Eigen::Index x, const bool left )
  {
    if ( std::abs( end.x() - start.x() ) < 1E-4 ) {
      increment = 0;
      y = double( left ? std::min( start.y(), end.y() ) : std::max( start.y(), end.y() ) );
      return;
    }
    increment = double( end.y() - start.y() ) / double( end.x() - start.x() );

    // Compute y value at center of x-column
    const double diff_start_x =
        double( x ) - std::floor( start.x() ) + 0.5 - ( start.x() - std::floor( start.x() ) );
    y = start.y() + diff_start_x * increment;
  }

  double y;
  double increment;
};

template<bool COLUMN_MAJOR>
struct RectangleIndexIterator<COLUMN_MAJOR>::iterator {
private:
  detail::RectangleData data;
  Line left_line;
  Line right_line;
  Index2D index;
  Eigen::Index scan_line_max;
  Eigen::Index next_switch;
  Eigen::Index row_min_;
  Eigen::Index row_max_;
  Eigen::Index col_min_;
  Eigen::Index col_max_;

public:
  iterator( const detail::RectangleData &data, Eigen::Index x, Eigen::Index y, Eigen::Index row_min,
            Eigen::Index row_max, Eigen::Index col_min, Eigen::Index col_max )
      : data( data ), left_line( data.lowest, data.left, y, true ),
        right_line( data.lowest, data.right, y, false ), index( x, y ), row_min_( row_min ),
        row_max_( row_max ), col_min_( col_min ), col_max_( col_max )
  {
    Eigen::Index scanline = COLUMN_MAJOR ? y : x;
    if ( scanline < data.left_switch ) {
      left_line = Line( data.lowest, data.left, scanline, true );
    } else {
      left_line = Line( data.left, data.highest, scanline, true );
    }
    if ( scanline < data.right_switch ) {
      right_line = Line( data.lowest, data.right, scanline, false );
    } else {
      right_line = Line( data.right, data.highest, scanline, false );
    }
    next_switch = std::min( data.left_switch, data.right_switch );
    if ( next_switch <= scanline ) {
      next_switch = std::max( data.left_switch, data.right_switch );
      if ( next_switch < scanline )
        next_switch = COLUMN_MAJOR ? col_max : row_max;
    }
    if constexpr ( COLUMN_MAJOR ) {
      scan_line_max = std::min<Eigen::Index>( row_max, std::round( right_line.x ) );
      if ( index.row >= scan_line_max ) {
        ++( *this );
      }
    } else {
      scan_line_max = std::min<Eigen::Index>( col_max, std::round( right_line.y ) );
      if ( index.col >= scan_line_max ) {
        ++( *this );
      }
    }
  }

  friend bool operator!=( const iterator &a, const iterator &b ) { return a.index != b.index; }

  const Index2D &operator*() const { return index; }

  iterator &operator++()
  {
    if constexpr ( COLUMN_MAJOR ) {
      if ( ++index.row < scan_line_max ) {
        return *this;
      }
      ++index.col;
      if ( index.col < col_max_ ) {
        left_line.x += left_line.increment;
        right_line.x += right_line.increment;
        if ( index.col >= next_switch ) {
          switchLines();
        }
        index.row = std::max<Eigen::Index>( row_min_, std::round( left_line.x ) );
        scan_line_max = std::min<Eigen::Index>( row_max_, std::round( right_line.x ) );
        if ( index.row >= scan_line_max ) {
          ++( *this );
        }
        return *this;
      }
      index.row = 0;
      index.col = col_max_;
      return *this;
    } else {
      if ( ++index.col < scan_line_max ) {
        return *this;
      }
      ++index.row;
      if ( index.row < row_max_ ) {
        left_line.y += left_line.increment;
        right_line.y += right_line.increment;
        if ( index.row >= next_switch ) {
          switchLines();
        }
        index.col = std::max<Eigen::Index>( col_min_, std::round( left_line.y ) );
        scan_line_max = std::min<Eigen::Index>( col_max_, std::round( right_line.y ) );
        if ( index.col >= scan_line_max ) {
          ++( *this );
        }
        return *this;
      }
      index.col = 0;
      index.row = row_max_;
      return *this;
    }
  }

private:
  void switchLines()
  {
    const Eigen::Index scanline = COLUMN_MAJOR ? index.col : index.row;
    const Eigen::Index scanline_end = COLUMN_MAJOR ? col_max_ : row_max_;
    if ( scanline == data.left_switch ) {
      left_line = Line( data.left, data.highest, scanline, true );
      next_switch = scanline < data.right_switch ? data.right_switch : scanline_end;
    }
    if ( scanline == data.right_switch ) {
      right_line = Line( data.right, data.highest, scanline, false );
      next_switch = scanline < data.left_switch ? data.left_switch : scanline_end;
    }
    if constexpr ( COLUMN_MAJOR ) {
      if ( index.col >= col_max_ ) {
        index.col = col_max_;
        index.row = 0;
        scan_line_max = 0;
      }
    } else {
      if ( index.row >= row_max_ ) {
        index.row = row_max_;
        index.col = 0;
        scan_line_max = 0;
      }
    }
  }
};

template<bool COLUMN_MAJOR>
typename RectangleIndexIterator<COLUMN_MAJOR>::iterator RectangleIndexIterator<COLUMN_MAJOR>::begin()
{
  if constexpr ( COLUMN_MAJOR ) {
    Eigen::Index y = std::max<Eigen::Index>( col_min_, std::round( data_.lowest.y() ) );
    Line left_line( data_.lowest, data_.left, y, true );
    return iterator( data_, std::max<Eigen::Index>( row_min_, std::round( left_line.x ) ), y,
                     row_min_, row_max_, col_min_, col_max_ );
  } else {
    Eigen::Index x = std::max<Eigen::Index>( row_min_, std::round( data_.lowest.x() ) );
    Line left_line( data_.lowest, data_.left, x, true );
    return iterator( data_, x, std::max<Eigen::Index>( col_min_, std::round( left_line.y ) ),
                     row_min_, row_max_, col_min_, col_max_ );
  }
}

template<bool COLUMN_MAJOR>
typename RectangleIndexIterator<COLUMN_MAJOR>::iterator RectangleIndexIterator<COLUMN_MAJOR>::end()
{
  if constexpr ( COLUMN_MAJOR ) {
    return iterator( data_, 0, col_max_, row_min_, row_max_, col_min_, col_max_ );
  }
  return iterator( data_, row_max_, 0, row_min_, row_max_, col_min_, col_max_ );
}

} // namespace hector_math
#endif // HECTOR_MATH_RECTANGLE_ITERATOR_H
