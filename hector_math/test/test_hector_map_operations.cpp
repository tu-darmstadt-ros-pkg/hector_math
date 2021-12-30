
#include <hector_math/map_operations/find_minmax.h>


#include <gtest/gtest.h>
#include <ros/ros.h>

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
class IteratorTest : public testing::Test
{
};

typedef testing::Types<float, double> Implementations;

TYPED_TEST_CASE( IteratorTest, Implementations );

TYPED_TEST( IteratorTest, find_minmax )
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
  EXPECT_EQ(findMinimum<Scalar>( map, polygon ), -std::numeric_limits<Scalar>::infinity());
  EXPECT_EQ(findMaximum<Scalar>( map, polygon ), std::numeric_limits<Scalar>::infinity());
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "test_hector_iterators" );
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
