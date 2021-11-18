//
// Created by stefan on 20.08.21.
//

#ifndef HECTOR_MATH_POLYGON_ITERATOR_H
#define HECTOR_MATH_POLYGON_ITERATOR_H

#include "hector_math/helpers/bounded_vector.h"
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
template<typename Scalar, typename Functor>
void iteratePolygon( const Polygon<Scalar> &polygon, Eigen::Index row_min, Eigen::Index row_max,
                     Eigen::Index col_min, Eigen::Index col_max, Functor functor );

//! Overload of iteratePolygon where row_min and col_min are set to 0 to allow for bounded iteration
//! of 2D matrices and arrays.
template<typename Scalar, typename Functor>
void iteratePolygon( const Polygon<Scalar> &polygon, Eigen::Index rows, Eigen::Index cols,
                     Functor functor )
{
  iteratePolygon( polygon, 0, rows, 0, cols, functor );
}

//! Overload of iteratePolygon where the indexes are not bounded.
template<typename Scalar, typename Functor>
void iteratePolygon( const Polygon<Scalar> &polygon, Functor functor )
{
  constexpr Eigen::Index min = std::numeric_limits<Eigen::Index>::min();
  constexpr Eigen::Index max = std::numeric_limits<Eigen::Index>::max();
  iteratePolygon( polygon, min, max, min, max, functor );
}

namespace detail
{
template<typename Scalar, typename Functor, int LIMIT = 0>
void iteratePolygon( const Polygon<Scalar> &polygon, Eigen::Index row_min, Eigen::Index row_max,
                     Eigen::Index col_min, Eigen::Index col_max, Functor functor );
}

template<typename Scalar, typename Functor>
void iteratePolygon( const Polygon<Scalar> &polygon, Eigen::Index row_min, Eigen::Index row_max,
                     Eigen::Index col_min, Eigen::Index col_max, Functor functor )
{
  if ( polygon.cols() < 3 )
    return;
  if ( polygon.cols() <= 30 ) {
    detail::iteratePolygon<Scalar, Functor, 30>( polygon, row_min, row_max, col_min, col_max,
                                                 functor );
  } else {
    detail::iteratePolygon<Scalar, Functor>( polygon, row_min, row_max, col_min, col_max, functor );
  }
}

namespace detail
{
template<typename Scalar, typename Functor, int LIMIT>
void iteratePolygon( const Polygon<Scalar> &polygon, Eigen::Index row_min, Eigen::Index row_max,
                     Eigen::Index col_min, Eigen::Index col_max, Functor functor )
{
  // Build iteration lines from the polygon points that allow us to get the x value for each
  // discrete y index in the map
  struct Line {
    Line() = default;

    Line( const Point<Scalar> &a, const Point<Scalar> &b )
    {
      start = a;
      end = b;
      if ( a.y() > b.y() )
        std::swap( start, end );
      Scalar diff_y = b.y() - a.y();
      if ( std::abs( diff_y ) < 1E-4 )
        x_increment = 0;
      else
        x_increment = ( b.x() - a.x() ) / ( b.y() - a.y() );

      x = start.x() + ( 0.5 - ( start.y() - std::floor( start.y() ) ) ) * x_increment;
    }

    Point<Scalar> start, end;
    Scalar x;
    Scalar x_increment;
  };
  using LineContainer =
      typename std::conditional<LIMIT == 0, std::vector<Line>, BoundedVector<Line, LIMIT + 1>>::type;
  using LinePointerContainer =
      typename std::conditional<LIMIT == 0, std::vector<Line *>, BoundedVector<Line *, LIMIT>>::type;
  using RegionContainer =
      typename std::conditional<LIMIT == 0, std::vector<Scalar>, BoundedVector<Scalar, LIMIT>>::type;

  LineContainer lines;
  lines.reserve( polygon.cols() );
  Eigen::Index max_y = std::round( polygon.col( 0 ).y() );
  // Build lines from points and obtain max y for the stopping criterion during the iteration loop
  for ( Eigen::Index i = 0; i < polygon.cols() - 1; ++i ) {
    lines.emplace_back( polygon.col( i ), polygon.col( i + 1 ) );
    Eigen::Index y_end = std::round( lines[i].end.y() );
    if ( y_end > max_y )
      max_y = y_end;
  }
  max_y = std::min( max_y, col_max );
  lines.emplace_back( polygon.col( polygon.cols() - 1 ), polygon.col( 0 ) );

  // Sort lines by their y start, to quickly find new active lines as we iterate over y
  std::sort( lines.begin(), lines.end(),
             []( const Line &a, const Line &b ) { return a.start.y() < b.start.y(); } );
  size_t active_line_index = 0;
  LinePointerContainer active_lines;
  RegionContainer x_region_segments;

  Eigen::Index y =
      std::max<Eigen::Index>( col_min, std::round( lines[active_line_index].start.y() ) );
  for ( ; y < max_y; ++y ) {
    // Determine lines that ended
    for ( int i = active_lines.size() - 1; i >= 0; --i ) {
      active_lines[i]->x += active_lines[i]->x_increment;
      if ( active_lines[i]->end.y() >= y + 0.5 )
        continue;
      active_lines.erase( active_lines.begin() + i );
    }
    // Determine new lines that started
    for ( size_t i = active_line_index; i < lines.size(); ++i ) {
      if ( lines[i].start.y() >= y + 0.5 )
        break;
      if ( lines[i].start.y() < y )
        lines[i].x += lines[i].x_increment;
      active_lines.push_back( &lines[i] );
      ++active_line_index;
    }

    // We obtain from each line the x for the current y and use that information to iterate between
    // each pair of x(k) -> x(k+1) where k = 2 * i and i is a natural integer
    x_region_segments.clear();
    for ( size_t i = 0; i < active_lines.size(); ++i ) {
      Eigen::Index x = std::round( active_lines[i]->x );
      x_region_segments.push_back( x );
    }
    std::sort( x_region_segments.begin(), x_region_segments.end() );
    if ( x_region_segments.size() > 2 ) {
      for ( size_t i = x_region_segments.size() - 2; i > 1; --i ) {
        if ( x_region_segments[i - 1] != x_region_segments[i] )
          continue;
        x_region_segments.erase( x_region_segments.begin() + i, x_region_segments.begin() + i + 2 );
      }
    }

    for ( size_t i = 0; i < x_region_segments.size() - 1; i += 2 ) {
      Eigen::Index x_start = std::max<Eigen::Index>( row_min, std::round( x_region_segments[i] ) );
      Eigen::Index x_end = std::min<Eigen::Index>( row_max, std::round( x_region_segments[i + 1] ) );
      for ( Eigen::Index x = x_start; x < x_end; ++x ) { functor( x, y ); }
    }
  }
}
} // namespace detail
} // namespace hector_math

#endif // HECTOR_MATH_POLYGON_ITERATOR_H
