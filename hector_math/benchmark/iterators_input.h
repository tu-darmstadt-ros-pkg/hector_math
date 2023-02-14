// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_ITERATORS_INPUT_H
#define HECTOR_MATH_ITERATORS_INPUT_H

#include "hector_math/types.h"

template<typename Scalar>
hector_math::Polygon<Scalar> createPolygon()
{
  hector_math::Polygon<Scalar> result( 2, 10 );
  result.col( 0 ) << 0.480, 0.000;
  result.col( 1 ) << 0.164, 0.155;
  result.col( 2 ) << 0.116, 0.500;
  result.col( 3 ) << -0.133, 0.250;
  result.col( 4 ) << -0.480, 0.399;
  result.col( 5 ) << -0.316, 0.000;
  result.col( 6 ) << -0.480, -0.399;
  result.col( 7 ) << -0.133, -0.250;
  result.col( 8 ) << 0.116, -0.500;
  result.col( 9 ) << 0.164, -0.155;
  result.colwise() += hector_math::Point<Scalar>( 0.5, 0.5 );
  return result;
}

template<typename Scalar>
hector_math::Polygon<Scalar> createSkewedQuadranglePolygon()
{
  hector_math::Polygon<Scalar> result( 2, 4 );
  result.col( 0 ) << 0.035, 0.035;
  result.col( 1 ) << 0, 0.96;
  result.col( 2 ) << 1, 1;
  result.col( 3 ) << 0.95, 0;
  return result;
}

#endif // HECTOR_MATH_ITERATORS_INPUT_H
