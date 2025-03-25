// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_POLYGON_ITERATOR_H
#define HECTOR_MATH_POLYGON_ITERATOR_H

#include "hector_math/containers/bounded_vector.h"
#include "hector_math/types.h"
#include <vector>

namespace hector_math
{

/*!
 * Iterates over all indexes that lie in the given polygon and for each index (x, y) calls the given
 * functor. This method will iterate all cells where the center of the cell (x+0.5, y+0.5) is inside
 * the polygon. Note: The polygon has to be in the index space, hence, if it is in map coordinates
 * it might be necessary to divide it by the map resolution.
 *
 * The indexes can be limited using the ranges [row_min, row_max) and [col_min, col_max) where
 * row/col_min is included but row/col_max is excluded, i.e., the largest x index functor may be
 * called with will be row_max - 1.
 * @tparam Functor A function or lambda method with the signature: void(Eigen::Index x, Eigen::Index y).
 * @param polygon The polygon that is iterated over.
 * @param functor The function that will be called for each index (x, y) inside the polygon.
 */
template<typename T, typename Functor>
void iteratePolygon( const Polygon<T> &polygon, Eigen::Index row_min, Eigen::Index row_max,
                     Eigen::Index col_min, Eigen::Index col_max, Functor functor );

//! Overload of iteratePolygon where row_min and col_min are set to 0 to allow for bounded iteration
//! of 2D matrices and arrays.
template<typename T, typename Functor>
void iteratePolygon( const Polygon<T> &polygon, Eigen::Index rows, Eigen::Index cols, Functor functor )
{
  iteratePolygon( polygon, 0, rows, 0, cols, functor );
}

//! Overload of iteratePolygon where the indexes are not bounded.
template<typename T, typename Functor>
void iteratePolygon( const Polygon<T> &polygon, Functor functor )
{
  constexpr Eigen::Index min = std::numeric_limits<Eigen::Index>::min();
  constexpr Eigen::Index max = std::numeric_limits<Eigen::Index>::max();
  iteratePolygon( polygon, min, max, min, max, functor );
}

template<bool COLUMN_MAJOR = true>
class PolygonIndexIterator
{
public:
  //! Overload iterating the given polygon where the limits are taken from the given Eigen dense base (array/matrix).
  //! @see PolygonIndexIterator::PolygonIndexIterator(const Polygond &, Eigen::Index, Eigen::Index, Eigen::Index, Eigen::Index)
  template<typename Derived>
  PolygonIndexIterator( const Eigen::DenseBase<Derived> &map, const Polygond &polygon )
      : PolygonIndexIterator( polygon, 0, map.rows(), 0, map.cols() )
  {
  }

  //! Overload where min x and y are 0 and max x and y are the given rows and cols.
  //! @see PolygonIndexIterator::PolygonIndexIterator(const Polygond &, Eigen::Index, Eigen::Index, Eigen::Index, Eigen::Index)
  PolygonIndexIterator( const Polygond &polygon, Eigen::Index rows, Eigen::Index cols )
      : PolygonIndexIterator( polygon, 0, rows, 0, cols )
  {
  }

  //! Overload where the indexes are not bounded.
  //! @see PolygonIndexIterator::PolygonIndexIterator(const Polygond &, Eigen::Index, Eigen::Index, Eigen::Index, Eigen::Index)
  PolygonIndexIterator( const Polygond &polygon )
      : PolygonIndexIterator( polygon, std::numeric_limits<Eigen::Index>::lowest(),
                              std::numeric_limits<Eigen::Index>::max(),
                              std::numeric_limits<Eigen::Index>::lowest(),
                              std::numeric_limits<Eigen::Index>::max() )
  {
  }

  /*!
   * Iterates over the points in the given polygon.
   * The iteration can be limited to a specific range within the polygon using the x/y_min and x/y_max values.
   * Note that the polygon has to be in the index_ space, hence, if it is in map coordinates it might
   * be necessary to convert it to the index_ space.
   *
   * The limits are given as [x_min, x_max) and [y_min, y_max) where x_min and y_min are included
   * but x_max and y_max are excluded, i.e., the largest x index_ that will be iterated over is x_max - 1.
   *
   * @param polygon The polygon that is iterated over.
   * @param x_min The minimum x index_ that will be iterated over.
   * @param x_max The maximum x index_ that will be iterated over. This index_ is excluded.
   * @param y_min The minimum y index_ that will be iterated over.
   * @param y_max The maximum y index_ that will be iterated over. This index_ is excluded.
   */
  PolygonIndexIterator( const Polygond &polygon, Eigen::Index row_min, Eigen::Index row_max,
                        Eigen::Index col_min, Eigen::Index col_max )
      : polygon_( polygon ), row_min_( row_min ), row_max_( row_max ), col_min_( col_min ),
        col_max_( col_max )
  {
  }

  struct Line;
  struct iterator;

  iterator begin();
  iterator end();

private:
  Polygond polygon_;
  const Eigen::Index row_min_;
  const Eigen::Index row_max_;
  const Eigen::Index col_min_;
  const Eigen::Index col_max_;
};

template<>
struct PolygonIndexIterator<true>::Line {
  Line() = default;

  Line( const Pointd &a, const Pointd &b )
  {
    Pointd start = a;
    Pointd end = b;
    if ( a.y() > b.y() )
      std::swap( start, end );
    double diff_y = b.y() - a.y();
    if ( std::abs( diff_y ) < 1E-4 )
      x_increment = 0;
    else
      x_increment = double( b.x() - a.x() ) / double( b.y() - a.y() );

    // Compute x value at center of y-column
    x = start.x() + ( 0.5 - ( start.y() - std::floor( start.y() ) ) ) * x_increment;
    start_y = start.y();
    end_y = end.y();
  }

  double start_y;
  double end_y;
  double x;
  double x_increment;
};

template<>
struct PolygonIndexIterator<false>::Line {
  Line() = default;

  Line( const Pointd &a, const Pointd &b )
  {
    Pointd start = a;
    Pointd end = b;
    if ( a.x() > b.x() )
      std::swap( start, end );
    double diff_x = b.x() - a.x();
    if ( std::abs( diff_x ) < 1E-4 )
      y_increment = 0;
    else
      y_increment = double( b.y() - a.y() ) / double( b.x() - a.x() );

    // Compute x value at center of y-column
    y = start.y() + ( 0.5 - ( start.x() - std::floor( start.x() ) ) ) * y_increment;
    start_x = start.x();
    end_x = end.x();
  }

  double start_x;
  double end_x;
  double y;
  double y_increment;
};

namespace detail
{
template<typename T, typename Functor, int LIMIT = 0>
void iteratePolygon( const Polygon<T> &polygon, Eigen::Index row_min, Eigen::Index row_max,
                     Eigen::Index col_min, Eigen::Index col_max, Functor functor );
}

template<typename T, typename Functor>
void iteratePolygon( const Polygon<T> &polygon, Eigen::Index row_min, Eigen::Index row_max,
                     Eigen::Index col_min, Eigen::Index col_max, Functor functor )
{
  if ( polygon.cols() < 3 )
    return;
  if ( polygon.cols() <= 15 ) {
    detail::iteratePolygon<T, Functor, 15>( polygon, row_min, row_max, col_min, col_max, functor );
  } else if ( polygon.cols() <= 63 ) {
    detail::iteratePolygon<T, Functor, 63>( polygon, row_min, row_max, col_min, col_max, functor );
  } else {
    detail::iteratePolygon<T, Functor>( polygon, row_min, row_max, col_min, col_max, functor );
  }
}

namespace detail
{
template<typename T, typename Functor, int LIMIT>
void iteratePolygon( const Polygon<T> &polygon, Eigen::Index row_min, Eigen::Index row_max,
                     Eigen::Index col_min, Eigen::Index col_max, Functor functor )
{
  // Build iteration lines from the polygon points that allow us to get the x value for each
  // discrete y index_ in the map
  using Line = PolygonIndexIterator<true>::Line;
  using LineContainer =
      std::conditional_t<LIMIT == 0, std::vector<Line>, BoundedVector<Line, LIMIT + 1>>;
  using RegionContainer =
      std::conditional_t<LIMIT == 0, std::vector<Eigen::Index>, BoundedVector<Eigen::Index, LIMIT>>;

  LineContainer lines;
  lines.reserve( polygon.cols() );
  Eigen::Index max_y = std::round( polygon.col( 0 ).y() );
  // Build lines from points and obtain max y for the stopping criterion during the iteration loop
  for ( Eigen::Index i = 0; i < polygon.cols() - 1; ++i ) {
    lines.emplace_back( polygon.col( i ).template cast<double>(),
                        polygon.col( i + 1 ).template cast<double>() );
    Eigen::Index y_end = std::round( lines[i].end_y );
    if ( y_end > max_y )
      max_y = y_end;
  }
  max_y = std::min( max_y, col_max );
  lines.emplace_back( polygon.col( polygon.cols() - 1 ).template cast<double>(),
                      polygon.col( 0 ).template cast<double>() );

  // Sort lines by their y start, to quickly find new active lines as we iterate over y
  std::sort( lines.begin(), lines.end(),
             []( const Line &a, const Line &b ) { return a.start_y < b.start_y; } );
  std::size_t active_line_index = 0;
  LineContainer active_lines;
  RegionContainer x_region_segments;

  Eigen::Index y = std::max<Eigen::Index>( col_min, std::round( lines[active_line_index].start_y ) );
  for ( ; y < max_y; ++y ) {
    const double y_cell_limit = double( y ) + 0.5;
    // Determine lines that ended
    for ( int i = active_lines.size() - 1; i >= 0; --i ) {
      active_lines[i].x += active_lines[i].x_increment;
      if ( active_lines[i].end_y >= y_cell_limit )
        continue;
      active_lines.erase( active_lines.begin() + i );
    }
    // Determine new lines that started
    for ( ; active_line_index < lines.size(); ++active_line_index ) {
      if ( lines[active_line_index].start_y >= y_cell_limit )
        break;
      if ( lines[active_line_index].end_y < y_cell_limit )
        continue; // Ignore lines that start and end before current column
      if ( lines[active_line_index].start_y < y )
        lines[active_line_index].x += lines[active_line_index].x_increment;
      active_lines.push_back( lines[active_line_index] );
    }

    // We obtain from each line the x for the current y and use that information to iterate between
    // each pair of x(k) -> x(k+1) where k = 2 * i and i is a natural integer
    x_region_segments.clear();
    for ( std::size_t i = 0; i < active_lines.size(); ++i ) {
      Eigen::Index x = std::round( active_lines[i].x );
      x_region_segments.push_back( x );
    }
    std::sort( x_region_segments.begin(), x_region_segments.end() );

    for ( std::size_t i = 0; i < x_region_segments.size() - 1; i += 2 ) {
      Eigen::Index x_start = std::max<Eigen::Index>( row_min, x_region_segments[i] );
      Eigen::Index x_end = std::min<Eigen::Index>( row_max, x_region_segments[i + 1] );
      for ( Eigen::Index x = x_start; x < x_end; ++x ) { functor( x, y ); }
    }
  }
}
} // namespace detail

template<bool COLUMN_MAJOR>
struct PolygonIndexIterator<COLUMN_MAJOR>::iterator {
private:
  std::vector<Line> lines_;
  std::vector<Line> active_lines_;
  std::vector<Eigen::Index> scan_line_region_segments_;
  Index2D index_;
  std::size_t active_line_index_ = 0;
  std::size_t segment_index_ = 0;
  Eigen::Index next_segment_switch_;
  const Eigen::Index row_min_;
  Eigen::Index row_max_;
  const Eigen::Index col_min_;
  Eigen::Index col_max_;
  bool end_ = false;

public:
  iterator( const Polygond &polygon, Eigen::Index row_min, Eigen::Index row_max,
            Eigen::Index col_min, Eigen::Index col_max )
      : row_min_( row_min ), row_max_( row_max ), col_min_( col_min ), col_max_( col_max )
  {
    lines_.reserve( polygon.cols() );
    // Build lines from points and obtain max y for the stopping criterion during the iteration loop
    for ( Eigen::Index i = 0; i < polygon.cols() - 1; ++i ) {
      lines_.emplace_back( polygon.col( i ), polygon.col( i + 1 ) );
    }
    lines_.emplace_back( polygon.col( polygon.cols() - 1 ), polygon.col( 0 ) );

    if constexpr ( COLUMN_MAJOR ) {
      Eigen::Index max_y = col_min;
      for ( Eigen::Index i = 0; i < polygon.cols(); ++i ) {
        max_y = std::max<Eigen::Index>( max_y, std::round( polygon.col( i ).y() ) );
      }
      col_max_ = std::min( col_max, max_y );
      // Sort lines by their y start, to quickly find new active lines as we iterate over y
      std::sort( lines_.begin(), lines_.end(),
                 []( const Line &a, const Line &b ) { return a.start_y < b.start_y; } );
      index_.col = std::max<Eigen::Index>( col_min, std::round( lines_[0].start_y ) );
    } else {
      Eigen::Index max_x = row_min;
      for ( Eigen::Index i = 0; i < polygon.cols(); ++i ) {
        max_x = std::max<Eigen::Index>( max_x, std::round( polygon.col( i ).x() ) );
      }
      row_max_ = std::min( row_max, max_x );
      // Sort lines by their x start, to quickly find new active lines as we iterate over x
      std::sort( lines_.begin(), lines_.end(),
                 []( const Line &a, const Line &b ) { return a.start_x < b.start_x; } );
      index_.row = std::max<Eigen::Index>( row_min, std::round( lines_[0].start_x ) );
    }
    updateSegments();
    if constexpr ( COLUMN_MAJOR ) {
      index_.row = std::max( row_min, scan_line_region_segments_[0] );
      next_segment_switch_ = std::min( row_max, scan_line_region_segments_[1] );
      if ( index_.row >= next_segment_switch_ )
        ++( *this );
    } else {
      index_.col = std::max( col_min, scan_line_region_segments_[0] );
      next_segment_switch_ = std::min( col_max, scan_line_region_segments_[1] );
      if ( index_.col >= next_segment_switch_ )
        ++( *this );
    }
  }

  iterator()
      : index_( 0, 0 ), next_segment_switch_( 0 ), row_min_( 0 ), row_max_( 0 ), col_min_( 0 ),
        col_max_( 0 ), end_( true )
  {
  }

  const Index2D &operator*() const { return index_; }

  friend bool operator==( const iterator &a, const iterator &b )
  {
    return a.end_ == b.end_ || ( !a.end_ && !b.end_ && a.index_ == b.index_ );
  }

  friend bool operator!=( const iterator &a, const iterator &b ) { return !( a == b ); }

  iterator &operator++()
  {
    if constexpr ( COLUMN_MAJOR ) {
      if ( ++index_.row < next_segment_switch_ ) {
        return *this;
      }
      segment_index_ += 2;
      if ( segment_index_ >= scan_line_region_segments_.size() - 1 ) {
        ++index_.col;
        if ( index_.col >= col_max_ ) {
          end_ = true;
          index_.col = col_max_;
          index_.row = 0;
          next_segment_switch_ = 0;
          scan_line_region_segments_.clear();
          return *this;
        }
        updateSegments();
        assert( scan_line_region_segments_.size() >= 2 );
      }
      index_.row = std::max( row_min_, scan_line_region_segments_[segment_index_] );
      next_segment_switch_ = std::min( row_max_, scan_line_region_segments_[segment_index_ + 1] );
      if ( index_.row >= next_segment_switch_ )
        ++( *this );
      return *this;
    } else {
      if ( ++index_.col < next_segment_switch_ ) {
        return *this;
      }
      segment_index_ += 2;
      if ( segment_index_ >= scan_line_region_segments_.size() - 1 ) {
        ++index_.row;
        if ( index_.row >= row_max_ ) {
          end_ = true;
          index_.row = row_max_;
          index_.col = 0;
          next_segment_switch_ = 0;
          scan_line_region_segments_.clear();
          return *this;
        }
        updateSegments();
        assert( scan_line_region_segments_.size() >= 2 );
      }
      index_.col = std::max( col_min_, scan_line_region_segments_[segment_index_] );
      next_segment_switch_ = std::min( col_max_, scan_line_region_segments_[segment_index_ + 1] );
      if ( index_.col >= next_segment_switch_ )
        ++( *this );
      return *this;
    }
  }

private:
  void updateSegments()
  {
    if constexpr ( COLUMN_MAJOR ) {
      const double y_cell_limit = double( index_.col ) + 0.5;
      // Determine lines that ended
      for ( int i = active_lines_.size() - 1; i >= 0; --i ) {
        active_lines_[i].x += active_lines_[i].x_increment;
        if ( active_lines_[i].end_y >= y_cell_limit )
          continue;
        active_lines_.erase( active_lines_.begin() + i );
      }
      // Determine new lines that started
      for ( ; active_line_index_ < lines_.size(); ++active_line_index_ ) {
        if ( lines_[active_line_index_].start_y >= y_cell_limit )
          break;
        if ( lines_[active_line_index_].end_y < y_cell_limit )
          continue; // Ignore lines that start and end before current column
        if ( lines_[active_line_index_].start_y < index_.col )
          lines_[active_line_index_].x += lines_[active_line_index_].x_increment;
        active_lines_.push_back( lines_[active_line_index_] );
      }

      // We obtain from each line the x for the current y and use that information to iterate between each pair of x(k) -> x(k+1) where k = 2 * i and i is a natural integer
      scan_line_region_segments_.clear();
      for ( std::size_t i = 0; i < active_lines_.size(); ++i ) {
        Eigen::Index x = std::round( active_lines_[i].x );
        scan_line_region_segments_.push_back( x );
      }
      std::sort( scan_line_region_segments_.begin(), scan_line_region_segments_.end() );
      segment_index_ = 0;
    } else {
      const double x_cell_limit = double( index_.row ) + 0.5;
      // Determine lines that ended
      for ( int i = active_lines_.size() - 1; i >= 0; --i ) {
        active_lines_[i].y += active_lines_[i].y_increment;
        if ( active_lines_[i].end_x >= x_cell_limit )
          continue;
        active_lines_.erase( active_lines_.begin() + i );
      }
      // Determine new lines that started
      for ( ; active_line_index_ < lines_.size(); ++active_line_index_ ) {
        if ( lines_[active_line_index_].start_x >= x_cell_limit )
          break;
        if ( lines_[active_line_index_].end_x < x_cell_limit )
          continue; // Ignore lines that start and end before current column
        if ( lines_[active_line_index_].start_x < index_.row )
          lines_[active_line_index_].y += lines_[active_line_index_].y_increment;
        active_lines_.push_back( lines_[active_line_index_] );
      }

      // We obtain from each line the y for the current x and use that information to iterate between each pair of x(k) -> x(k+1) where k = 2 * i and i is a natural integer
      scan_line_region_segments_.clear();
      for ( std::size_t i = 0; i < active_lines_.size(); ++i ) {
        Eigen::Index y = std::round( active_lines_[i].y );
        scan_line_region_segments_.push_back( y );
      }
      std::sort( scan_line_region_segments_.begin(), scan_line_region_segments_.end() );
      segment_index_ = 0;
    }
  }
};

template<bool COLUMN_MAJOR>
typename PolygonIndexIterator<COLUMN_MAJOR>::iterator PolygonIndexIterator<COLUMN_MAJOR>::begin()
{
  return iterator( polygon_, row_min_, row_max_, col_min_, col_max_ );
}

template<bool COLUMN_MAJOR>
typename PolygonIndexIterator<COLUMN_MAJOR>::iterator PolygonIndexIterator<COLUMN_MAJOR>::end()
{
  return iterator();
}

} // namespace hector_math

#endif // HECTOR_MATH_POLYGON_ITERATOR_H
