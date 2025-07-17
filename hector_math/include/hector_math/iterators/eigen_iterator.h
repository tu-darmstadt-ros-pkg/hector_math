// Copyright (c) 2024 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_EIGEN_ITERATOR_H
#define HECTOR_MATH_EIGEN_ITERATOR_H

#include "hector_math/types.h"

namespace hector_math
{

/*!
 * Convenience function to iterate over all elements of a dense Eigen matrix or array in the most
 * efficient fashion.
 *
 * @tparam Functor A function or lambda method with the signature: void(Eigen::Index x, Eigen::Index y).
 * @param functor The function that will be called for each index (x, y) inside the polygon.
 */
template<typename Derived, typename Functor>
void iterateDenseBase( const Eigen::DenseBase<Derived> &map, Functor functor );

//! Convenience iterator to efficiently iterate over all indices in a Eigen matrix or array in a for each loop.
template<typename Derived>
class EigenIndexIterator
{
public:
  using Scalar = typename Eigen::DenseBase<Derived>::Scalar;

  explicit EigenIndexIterator( const Eigen::DenseBase<Derived> &map ) : map_( map ) { }

  struct iterator;

  iterator begin() { return iterator( map_, 0, 0 ); }
  iterator end()
  {
    if constexpr ( Eigen::DenseBase<Derived>::IsRowMajor ) {
      return iterator( map_, map_.rows(), 0 );
    }
    return iterator( map_, 0, map_.cols() );
  }

private:
  const Eigen::DenseBase<Derived> &map_;
};

//! Convenience iterator to efficiently iterate over all values in a Eigen matrix or array in a for each loop.
template<typename Derived>
class EigenValueIterator
{
public:
  using Scalar = typename Eigen::DenseBase<Derived>::Scalar;

  explicit EigenValueIterator( const Eigen::DenseBase<Derived> &map ) : map_( map ) { }

  struct iterator;

  iterator begin() { return iterator( map_, 0, 0 ); }
  iterator end()
  {
    if constexpr ( Eigen::DenseBase<Derived>::IsRowMajor ) {
      return iterator( map_, map_.rows(), 0 );
    }
    return iterator( map_, 0, map_.cols() );
  }

private:
  const Eigen::DenseBase<Derived> &map_;
};

// METHOD

template<typename Derived, typename Functor>
void iterateDenseBase( const Eigen::DenseBase<Derived> &map, Functor functor )
{
  const Eigen::Index cols = map.cols();
  const Eigen::Index rows = map.rows();
  if constexpr ( !Eigen::DenseBase<Derived>::IsRowMajor ) {
    for ( Eigen::Index y = 0; y < cols; ++y ) {
      for ( Eigen::Index x = 0; x < rows; ++x ) { functor( x, y ); }
    }
  } else {
    for ( Eigen::Index x = 0; x < rows; ++x ) {
      for ( Eigen::Index y = 0; y < cols; ++y ) { functor( x, y ); }
    }
  }
}

// ITERATOR CLASSES

template<typename Derived>
struct EigenIndexIterator<Derived>::iterator {
  iterator( const Eigen::DenseBase<Derived> &map, Eigen::Index row, Eigen::Index col )
      : index( row, col ), map_( map )
  {
  }

  Index2D index;

  const Index2D &operator*() const { return index; }

  bool operator!=( const iterator &other ) const
  {
    return index.row != other.index.row || index.col != other.index.col;
  }

  iterator operator++()
  {
    if constexpr ( Eigen::DenseBase<Derived>::IsRowMajor ) {
      if ( ++index.col >= map_.cols() ) {
        index.col = 0;
        ++index.row;
      }
    } else {
      if ( ++index.row >= map_.rows() ) {
        index.row = 0;
        ++index.col;
      }
    }
    return *this;
  }

private:
  const Eigen::DenseBase<Derived> &map_;
};

template<typename Derived>
struct EigenValueIterator<Derived>::iterator {
  iterator( const Eigen::DenseBase<Derived> &map, Eigen::Index row, Eigen::Index col )
      : row( row ), col( col ), map_( map )
  {
  }

  Eigen::Index row;
  Eigen::Index col;

  Scalar value() const { return map_( row, col ); }

  Scalar operator*() const { return map_( row, col ); }

  bool operator!=( const iterator &other ) const { return row != other.row || col != other.col; }

  iterator operator++()
  {
    if constexpr ( Eigen::DenseBase<Derived>::IsRowMajor ) {
      if ( ++col >= map_.cols() ) {
        col = 0;
        ++row;
      }
    } else {
      if ( ++row >= map_.rows() ) {
        row = 0;
        ++col;
      }
    }
    return *this;
  }

private:
  const Eigen::DenseBase<Derived> &map_;
};
} // namespace hector_math

#endif // HECTOR_MATH_EIGEN_ITERATOR_H
