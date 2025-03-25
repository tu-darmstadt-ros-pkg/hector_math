// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

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
 * @param functor The function that will be called for each index (x, y) inside the circle.
 */
template<typename T, typename Functor>
void iterateCircle( const Vector2<T> &center, double radius, Eigen::Index row_min,
                    Eigen::Index row_max, Eigen::Index col_min, Eigen::Index col_max,
                    Functor functor );

//! Overload of iterateCircle where row_min and col_min are set to 0 to allow for bounded iteration
//! of 2D matrices and arrays.
template<typename T, typename Functor>
void iterateCircle( const Vector2<T> &center, double radius, Eigen::Index rows, Eigen::Index cols,
                    Functor functor )
{
  iterateCircle( center, radius, 0, rows, 0, cols, functor );
}

//! Overload of iterateCircle where the indexes are not bounded.
template<typename T, typename Functor>
void iterateCircle( const Vector2<T> &center, double radius, Functor functor )
{
  constexpr Eigen::Index min = std::numeric_limits<Eigen::Index>::min();
  constexpr Eigen::Index max = std::numeric_limits<Eigen::Index>::max();
  iterateCircle( center, radius, min, max, min, max, functor );
}

template<bool COLUMN_MAJOR = true>
class CircleIndexIterator
{
public:
  //! Overload iterating the given circle where the limits are taken from the given Eigen dense base (array/matrix).
  //! @see CircleIndexIterator::CircleIndexIterator(const Vector2d &, double, Eigen::Index, Eigen::Index, Eigen::Index, Eigen::Index)
  template<typename Derived>
  CircleIndexIterator( const Eigen::DenseBase<Derived> &map, const Vector2d &center, double radius )
      : CircleIndexIterator( center, radius, 0, map.rows(), 0, map.cols() )
  {
  }

  //! Overload where min x and y are 0 and max x and y are the given rows and cols.
  //! @see CircleIndexIterator::CircleIndexIterator(const Vector2d &, double, Eigen::Index, Eigen::Index, Eigen::Index, Eigen::Index)
  CircleIndexIterator( const Vector2d &center, double radius, Eigen::Index rows, Eigen::Index cols )
      : CircleIndexIterator( center, radius, 0, rows, 0, cols )
  {
  }

  //! Overload where the indexes are not bounded.
  //! @see CircleIndexIterator::CircleIndexIterator(const Vector2d &, double, Eigen::Index, Eigen::Index, Eigen::Index, Eigen::Index)
  CircleIndexIterator( const Vector2d &center, double radius )
      : CircleIndexIterator( center, radius, std::numeric_limits<Eigen::Index>::lowest(),
                             std::numeric_limits<Eigen::Index>::max(),
                             std::numeric_limits<Eigen::Index>::lowest(),
                             std::numeric_limits<Eigen::Index>::max() )
  {
  }

  /*!
   * Iterates over the given circle with the given center and radius.
   * The iteration can be limited to a specific range within the circle using the x/y_min and x/y_max values.
   * Note that the circle has to be in the index space, hence, if it is in map coordinates it might
   * be necessary to convert it to the index space.
   *
   * The limits are given as [x_min, x_max) and [y_min, y_max) where x_min and y_min are included
   * but x_max and y_max are excluded, i.e., the largest x index that will be iterated over is x_max - 1.
   *
   * @param center The center of the circle in the index space.
   * @param radius The radius of the circle in the index space.
   * @param x_min The minimum x index that will be iterated over.
   * @param x_max The maximum x index that will be iterated over. This index is excluded.
   * @param y_min The minimum y index that will be iterated over.
   * @param y_max The maximum y index that will be iterated over. This index is excluded.
   */
  CircleIndexIterator( const Vector2d &center, double radius, Eigen::Index x_min,
                       Eigen::Index x_max, Eigen::Index y_min, Eigen::Index y_max )
      : center_( center ), radius_squared_( radius * radius )
  {
    if constexpr ( COLUMN_MAJOR ) {
      x_min_ = x_min;
      x_max_ = x_max;
      y_min_ = std::max<Eigen::Index>( y_min, std::round( double( center.y() ) - radius ) );
      y_max_ = std::min<Eigen::Index>( y_max, std::round( double( center.y() ) + radius ) );
    } else {
      y_min_ = y_min;
      y_max_ = y_max;
      x_min_ = std::max<Eigen::Index>( x_min, std::round( double( center.x() ) - radius ) );
      x_max_ = std::min<Eigen::Index>( x_max, std::round( double( center.x() ) + radius ) );
    }
  }

  struct iterator;

  iterator begin();
  iterator end();

private:
  const Vector2d center_;
  double radius_squared_;
  Eigen::Index x_min_;
  Eigen::Index x_max_;
  Eigen::Index y_min_;
  Eigen::Index y_max_;
};

template<typename T, typename Functor>
void iterateCircle( const Vector2<T> &center, double radius, Eigen::Index row_min,
                    Eigen::Index row_max, Eigen::Index col_min, Eigen::Index col_max, Functor functor )
{
  const Eigen::Index min_y =
      std::max<Eigen::Index>( col_min, std::round( double( center.y() ) - radius ) );
  const Eigen::Index max_y =
      std::min<Eigen::Index>( col_max, std::round( double( center.y() ) + radius ) );
  const double radius_squared = radius * radius;
  for ( Eigen::Index y = min_y; y < max_y; ++y ) {
    // Formula of a circle is: r^2 = x^2 + y^2 ==> x = +/- sqrt(r^2 - y^2) which we can use to get min x and max x
    const double delta_y = double( y ) + 0.5 - double( center.y() );
    const double width = std::sqrt( radius_squared - delta_y * delta_y );
    const Eigen::Index min_x =
        std::max<Eigen::Index>( row_min, std::round( double( center.x() ) - width ) );
    const Eigen::Index max_x =
        std::min<Eigen::Index>( row_max, std::round( double( center.x() ) + width ) );
    for ( Eigen::Index x = min_x; x < max_x; ++x ) { functor( x, y ); }
  }
}

template<bool COLUMN_MAJOR>
struct CircleIndexIterator<COLUMN_MAJOR>::iterator {
private:
  friend class CircleIndexIterator<COLUMN_MAJOR>;
  const CircleIndexIterator<COLUMN_MAJOR> *circle_;
  Eigen::Index max;
  Index2D index;

public:
  iterator( const CircleIndexIterator<COLUMN_MAJOR> *circle, Eigen::Index x, Eigen::Index y,
            Eigen::Index max )
      : circle_( circle ), max( max ), index( x, y )
  {
  }

  friend bool operator!=( const iterator &a, const iterator &b ) { return a.index != b.index; }

  const Index2D &operator*() const { return index; }

  iterator operator++()
  {
    if constexpr ( COLUMN_MAJOR ) {
      if ( ++index.row < max ) {
        return *this;
      }
      if ( ++index.col < circle_->y_max_ ) {
        const double delta_y = double( index.col ) + 0.5 - double( circle_->center_.y() );
        const double width = std::sqrt( circle_->radius_squared_ - delta_y * delta_y );
        index.row = std::max<Eigen::Index>( circle_->x_min_,
                                            std::round( double( circle_->center_.x() ) - width ) );
        max = std::min<Eigen::Index>( circle_->x_max_,
                                      std::round( double( circle_->center_.x() ) + width ) );
        if ( index.row >= max ) {
          ++( *this );
        }
        return *this;
      }
      index.col = circle_->y_max_;
      index.row = 0;
      max = 0;
      return *this;
    } else {
      if ( ++index.col < max ) {
        return *this;
      }
      if ( ++index.row < circle_->x_max_ ) {

        const double delta_x = double( index.row ) + 0.5 - double( circle_->center_.x() );
        const double height = std::sqrt( circle_->radius_squared_ - delta_x * delta_x );
        index.col = std::max<Eigen::Index>( circle_->y_min_,
                                            std::round( double( circle_->center_.y() ) - height ) );
        max = std::min<Eigen::Index>( circle_->y_max_,
                                      std::round( double( circle_->center_.y() ) + height ) );
        if ( index.col >= max ) {
          ++( *this );
        }
        return *this;
      }
      index.row = circle_->x_max_;
      index.col = 0;
      max = 0;
      return *this;
    }
  }
};

template<bool COLUMN_MAJOR>
typename CircleIndexIterator<COLUMN_MAJOR>::iterator CircleIndexIterator<COLUMN_MAJOR>::begin()
{
  if constexpr ( COLUMN_MAJOR ) {
    // Formula of a circle is: r^2 = x^2 + y^2 ==> x = +/- sqrt(r^2 - y^2) which we can use to get min x and max x
    const double delta_y = double( y_min_ ) + 0.5 - double( center_.y() );
    const double width = std::sqrt( radius_squared_ - delta_y * delta_y );
    const Eigen::Index start_x =
        std::max<Eigen::Index>( x_min_, std::round( double( center_.x() ) - width ) );
    const Eigen::Index end_x =
        std::min<Eigen::Index>( x_max_, std::round( double( center_.x() ) + width ) );
    auto it = iterator( this, start_x, y_min_, end_x );
    if ( it.index.row >= it.max ) {
      ++it;
    }
    return it;
  } else {
    const double delta_x = double( x_min_ ) + 0.5 - double( center_.x() );
    const double height = std::sqrt( radius_squared_ - delta_x * delta_x );
    const Eigen::Index start_y =
        std::max<Eigen::Index>( y_min_, std::round( double( center_.y() ) - height ) );
    const Eigen::Index end_y =
        std::min<Eigen::Index>( y_max_, std::round( double( center_.y() ) + height ) );
    auto it = iterator( this, x_min_, start_y, end_y );
    if ( it.index.col >= it.max ) {
      ++it;
    }
    return it;
  }
}

template<bool COLUMN_MAJOR>
typename CircleIndexIterator<COLUMN_MAJOR>::iterator CircleIndexIterator<COLUMN_MAJOR>::end()
{
  if constexpr ( COLUMN_MAJOR ) {
    return iterator( this, 0, y_max_, 0 );
  }
  return iterator( this, x_max_, 0, 0 );
}
} // namespace hector_math

#endif // HECTOR_MATH_CIRCLE_ITERATOR_H
