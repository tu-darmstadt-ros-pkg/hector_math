// Copyright (c) 2022, 2024 Aljoscha Schmidt, Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
#include <hector_math/map_operations/find_minmax.h>
#include <hector_math/map_operations/fit_plane.h>

#include <gtest/gtest.h>

using namespace hector_math;

enum PolygonTyp { RectangleTopLeft, All6x6, All2x2 };
template<typename Scalar>
hector_math::Polygon<Scalar> createPolygon( PolygonTyp polygonTyp )
{
  hector_math::Polygon<Scalar> result( 2, 4 );
  switch ( polygonTyp ) {
  case PolygonTyp::RectangleTopLeft:
    result = hector_math::Polygon<Scalar>( 2, 4 );
    result.col( 0 ) << 1, 0;
    result.col( 1 ) << 1, 3;
    result.col( 2 ) << 4, 3;
    result.col( 3 ) << 4, 0;
    return result;
  case PolygonTyp::All6x6:
    result = hector_math::Polygon<Scalar>( 2, 4 );
    result.col( 0 ) << 0, 0;
    result.col( 1 ) << 0, 7;
    result.col( 2 ) << 7, 7;
    result.col( 3 ) << 7, 0;
    return result;
  case PolygonTyp::All2x2:
    result = hector_math::Polygon<Scalar>( 2, 4 );
    result.col( 0 ) << 0, 0;
    result.col( 1 ) << 0, 3;
    result.col( 2 ) << 3, 3;
    result.col( 3 ) << 3, 0;
    return result;
  }
  return result;
}

template<typename Scalar>
class MapOperations : public testing::Test
{
};

typedef testing::Types<float, double> Implementations;

TYPED_TEST_CASE( MapOperations, Implementations );

TYPED_TEST( MapOperations, find_minmax )
{
  using Scalar = TypeParam;
  const Scalar NaN = std::numeric_limits<Scalar>::quiet_NaN();
  GridMap<Scalar> map( 6, 6 );
  // @formatter:off
  // clang-format off
  map << 0, 1, 1, 0, 1, 0,
      1, 2, 3, 0, 0, 0,
      0, 1, 4, 1, 1, 0,
      1, 0, 3, 0, 2, 0,
      2, 2, NaN, 0, -1, 0,
      2, 4, 2, 0, 1, 7;
  // @formatter:on
  // clang-format on
  Polygon<Scalar> polygon = createPolygon<Scalar>( PolygonTyp::RectangleTopLeft );
  EXPECT_TRUE( findMinimum<Scalar>( map, polygon ) == 0 );
  EXPECT_TRUE( findMaximum<Scalar>( map, polygon ) == 4 );
  polygon = createPolygon<Scalar>( PolygonTyp::All6x6 );
  EXPECT_TRUE( findMinimum<Scalar>( map, polygon ) == -1 );
  EXPECT_TRUE( findMaximum<Scalar>( map, polygon ) == 7 );
  map = GridMap<Scalar>( 2, 2 );
  polygon = createPolygon<Scalar>( PolygonTyp::All2x2 );
  // @formatter:off
  // clang-format off
  map << 4.3, 4.5,
      4.2, 4.1;
  // @formatter:on
  // clang-format on
  EXPECT_NEAR( findMinimum<Scalar>( map, polygon ), 4.1, 0.001 );
  EXPECT_NEAR( findMaximum<Scalar>( map, polygon ), 4.5, 0.001 );
  map = GridMap<Scalar>( 3, 3 );
  polygon = createPolygon<Scalar>( PolygonTyp::All2x2 );
  // @formatter:off
  // clang-format off
  map << 0,std::numeric_limits<Scalar>::min(), std::numeric_limits<Scalar>::infinity(),
        -3,2,0,
      -1, -std::numeric_limits<Scalar>::infinity(), std::numeric_limits<Scalar>::max();
  // @formatter:on
  // clang-format on
  EXPECT_EQ( findMinimum<Scalar>( map, polygon ), -std::numeric_limits<Scalar>::infinity() );
  EXPECT_EQ( findMaximum<Scalar>( map, polygon ), std::numeric_limits<Scalar>::infinity() );
}

template<typename Scalar>
GridMap<Scalar> createMap( Eigen::Index rows, Eigen::Index cols, Scalar gradient_x, Scalar gradient_y )
{
  GridMap<Scalar> map( rows, cols );
  for ( Eigen::Index col = 0; col < map.cols(); ++col ) {
    for ( Eigen::Index row = 0; row < map.rows(); ++row ) {
      map( row, col ) = gradient_x * row + gradient_y * col;
    }
  }
  return map;
}

TYPED_TEST( MapOperations, fitPlane )
{
  using Scalar = TypeParam;
  constexpr Scalar NaN = std::numeric_limits<Scalar>::quiet_NaN();
  GridMap<Scalar> map;
  PlaneEstimationResult result;

  map = createMap<Scalar>( 10, 10, 0, 0 );
  result = fitPlaneXY( map );
  EXPECT_NEAR( result.gradient_x, 0, 1e-4 );
  EXPECT_NEAR( result.gradient_y, 0, 1e-4 );
  EXPECT_NEAR( result.center_plane_z, 0, 1e-4 );
  EXPECT_NEAR( result.percentage_known, 1, 1e-4 );
  if ( this->HasFailure() )
    FAIL();

  map = createMap<Scalar>( 6, 6, 1, 1 );
  result = fitPlaneXY( map );
  EXPECT_NEAR( result.gradient_x, 1, 1e-4 );
  EXPECT_NEAR( result.gradient_y, 1, 1e-4 );
  EXPECT_NEAR( result.center_plane_z, 5, 1e-4 );
  EXPECT_NEAR( result.percentage_known, 1, 1e-4 );
  if ( this->HasFailure() )
    FAIL();

  // NaN in the center are not as bad
  map.block( 2, 2, 2, 2 ) = NaN;
  result = fitPlaneXY( map );
  EXPECT_NEAR( result.gradient_x, 1, 1e-4 );
  EXPECT_NEAR( result.gradient_y, 1, 1e-4 );
  EXPECT_NEAR( result.center_plane_z, 5, 1e-4 );
  EXPECT_LT( result.percentage_known, 0.99 );
  if ( this->HasFailure() )
    FAIL();

  // NaN at the edge should reduce quality
  map = createMap<Scalar>( 6, 6, 1, 1 );
  map.block( 4, 2, 2, 2 ) = NaN;
  result = fitPlaneXY( map );
  EXPECT_NEAR( result.gradient_x, 1, 1e-4 );
  EXPECT_NEAR( result.gradient_y, 1, 1e-4 );
  EXPECT_NEAR( result.center_plane_z, 5, 1e-4 );
  EXPECT_LT( result.percentage_known, 0.99 );
  if ( this->HasFailure() )
    FAIL();

  // If half the map is NaN, quality should be lower but center z should still be correct
  map = createMap<Scalar>( 6, 6, 1, 1 );
  map.block( 3, 0, 3, 6 ) = NaN;
  result = fitPlaneXY( map );
  EXPECT_NEAR( result.gradient_x, 1, 1e-4 );
  EXPECT_NEAR( result.gradient_y, 1, 1e-4 );
  EXPECT_NEAR( result.center_plane_z, 5, 1e-4 );
  EXPECT_NEAR( result.percentage_known, 0.5, 1e-4 );
  if ( this->HasFailure() )
    FAIL();

  map = createMap<Scalar>( 10, 10, 20, -21.5 );
  map( 2, 2 ) = map( 4, 3 ) = map( 8, 1 ) = map( 9, 0 ) = map( 0, 0 ) = NaN;
  result = fitPlaneXY( map );
  EXPECT_NEAR( result.gradient_x, 20, 1e-4 );
  EXPECT_NEAR( result.gradient_y, -21.5, 1e-4 );
  EXPECT_NEAR( result.center_plane_z, 4.5 * 20 - 4.5 * 21.5, 1e-4 );
  EXPECT_LT( result.percentage_known, 1 );
  if ( this->HasFailure() )
    FAIL();

  map = GridMap<Scalar>::Constant( 10, 10, NaN );
  map.block( 0, 0, 5, 5 ) = createMap<Scalar>( 5, 5, 1, -1 );
  result = fitPlaneXY( map );
  EXPECT_NEAR( result.gradient_x, 1, 1e-4 );
  EXPECT_NEAR( result.gradient_y, -1, 1e-4 );
  EXPECT_NEAR( result.center_plane_z, 0, 1e-4 );
  EXPECT_LT( result.percentage_known, 0.9 );
  if ( this->HasFailure() )
    FAIL();

  map = createMap<Scalar>( 10, 10, 3, -1 );
  map += GridMap<Scalar>::Random( 10, 10 ) * 0.1;
  result = fitPlaneXY( map );
  EXPECT_NEAR( result.gradient_x, 3, 0.1 );
  EXPECT_NEAR( result.gradient_y, -1, 0.1 );
  EXPECT_NEAR( result.center_plane_z, 4.5 * 3 - 4.5, 0.1 );

  // Different resolution
  map = createMap<Scalar>( 6, 6, 1, 1 );
  result = fitPlaneXY( map, 0.25 );
  EXPECT_NEAR( result.gradient_x, 4, 1e-4 );
  EXPECT_NEAR( result.gradient_y, 4, 1e-4 );
  EXPECT_NEAR( result.center_plane_z, 5, 1e-4 );
  EXPECT_NEAR( result.percentage_known, 1, 1e-4 );
  if ( this->HasFailure() )
    FAIL();
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
