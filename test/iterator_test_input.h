#ifndef HECTOR_MATH_ITERATOR_TEST_INPUT_H
#define HECTOR_MATH_ITERATOR_TEST_INPUT_H
#include <hector_math/types.h>
#include <iostream>

enum PolygonTyp { RandomStructureNegativeIndices, RandomStructure, Z_Shape, Circle, U_Shape };
///////////////// Polygon /////////////////////
template<typename Scalar>
hector_math::Polygon<Scalar> createPolygon( PolygonTyp polygonTyp )
{
  hector_math::Polygon<Scalar> result( 2, 15 );
  switch ( polygonTyp ) {
  case PolygonTyp::RandomStructureNegativeIndices:
    result = hector_math::Polygon<Scalar>( 2, 15 );
    result.col( 0 ) << 0, 4.9;
    result.col( 1 ) << 3.5, 4.9;
    result.col( 2 ) << 4.3333, 3.3333;
    result.col( 3 ) << 4, 2.3333;
    result.col( 4 ) << 0, 2;
    result.col( 5 ) << 0, 0.95;
    result.col( 6 ) << 4, 0;
    result.col( 7 ) << 0, 0;
    result.col( 8 ) << -0.8, -1.0;
    result.col( 9 ) << -0, -2.0;
    result.col( 10 ) << -4.01, -0.99;
    result.col( 11 ) << -4, 1;
    result.col( 12 ) << -3, 4;
    result.col( 13 ) << -2, 4;
    result.col( 14 ) << -1.0, 2.5;
    return result;
  case PolygonTyp::RandomStructure:
    result = hector_math::Polygon<Scalar>( 2, 15 );
    result.col( 0 ) << 5, 9.9;
    result.col( 1 ) << 8.5, 9.9;
    result.col( 2 ) << 9.3333, 8.3333;
    result.col( 3 ) << 9, 7.3333;
    result.col( 4 ) << 5, 7;
    result.col( 5 ) << 5, 5.95;
    result.col( 6 ) << 9, 5;
    result.col( 7 ) << 5, 5;
    result.col( 8 ) << 4.2, 4.0;
    result.col( 9 ) << 5, 3.0;
    result.col( 10 ) << 0.99, 4.001;
    result.col( 11 ) << 1, 6;
    result.col( 12 ) << 1.8, 9;
    result.col( 13 ) << 3, 9;
    result.col( 14 ) << 4, 7.5;
    return result;
  case PolygonTyp::Z_Shape:
    // Z structure
    result = hector_math::Polygon<Scalar>( 2, 8 );
    result.col( 0 ) << 6.66666, 9;
    result.col( 1 ) << 0, 9;
    result.col( 2 ) << 0, 8;
    result.col( 3 ) << 5, 8;
    result.col( 4 ) << 0.333, 1;
    result.col( 5 ) << 7, 1;
    result.col( 6 ) << 7, 2;
    result.col( 7 ) << 2, 2;
    return result;
  case PolygonTyp::Circle:
    // circle structure
    result = hector_math::Polygon<Scalar>( 2, 8 );
    result.col( 0 ) << 5, 10;
    result.col( 1 ) << 1.4, 8.5;
    result.col( 2 ) << 0, 5;
    result.col( 3 ) << 1.4, 1.5;
    result.col( 4 ) << 5, 0;
    result.col( 5 ) << 8.6, 1.5;
    result.col( 6 ) << 10, 5;
    result.col( 7 ) << 8.6, 8.5;
    return result;
  case PolygonTyp::U_Shape:
    // u structure
    result = hector_math::Polygon<Scalar>( 2, 11 );
    result.col( 0 ) << 1.3, 9.2;
    result.col( 1 ) << 3, 9.2;
    result.col( 2 ) << 3, 4.5;
    result.col( 3 ) << 6, 4.5;
    result.col( 4 ) << 6, 9.2;
    result.col( 5 ) << 8.5, 9.2;
    result.col( 6 ) << 9.5, 4.5;
    result.col( 7 ) << 9.0, 3;
    result.col( 8 ) << 7.4, 8.5;
    result.col( 9 ) << 7.4, 3;
    result.col( 10 ) << 1.3, 3;
    return result;
  default:
    std::cout << "Invalid polygonTyp" << std::endl;
  }
  return hector_math::Polygon<Scalar>( 2, 0 );
}

#endif // HECTOR_MATH_ITERATOR_TEST_INPUT_H