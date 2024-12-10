// Copyright (c) 2024 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_EIGEN_ITERATOR_H
#define HECTOR_MATH_EIGEN_ITERATOR_H

#include "hector_math/types.h"

namespace hector_math
{
/*!
 * Convenience function to iterate over all elements of a dense Eigen matrix or array in the most efficient fashion.
 *
 *
 * @tparam Functor A function or lambda method with the signature: void(Eigen::Index x, Eigen::Index y).
 * @param functor The function that will be called for each index (x, y) inside the polygon.
 */
template<typename Derived, typename Functor>
void iterateDenseBase( const Eigen::DenseBase<Derived> &map, Functor functor )
{
  const Eigen::Index cols = map.cols();
  const Eigen::Index rows = map.rows();
  if ( !Eigen::DenseBase<Derived>::IsRowMajor ) {
    for ( Eigen::Index y = 0; y < cols; ++y ) {
      for ( Eigen::Index x = 0; x < rows; ++x ) { functor( x, y ); }
    }
  } else {
    for ( Eigen::Index x = 0; x < rows; ++x ) {
      for ( Eigen::Index y = 0; y < cols; ++y ) { functor( x, y ); }
    }
  }
}
} // namespace hector_math

#endif // HECTOR_MATH_EIGEN_ITERATOR_H
