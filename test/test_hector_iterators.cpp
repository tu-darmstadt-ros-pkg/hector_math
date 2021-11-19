
#include "iterator_test_input.h"
#include <hector_math/iterators/circle_iterator.h>
#include <hector_math/iterators/polygon_iterator.h>

#include "eigen_tests.h"
#include <fstream>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>

using namespace hector_math;

std::string jsonVector( const GridMap<Eigen::Index> &map, int offset )
{
  std::string s = "[";
  for ( int i = 0; i < map.rows(); i++ ) {
    for ( int j = 0; j < map.cols(); j++ ) {
      if ( map( i, j ) > 0 ) {
        s += "[" + std::to_string( i - offset ) + ", " + std::to_string( j - offset ) + "], ";
      }
    }
  }
  s = s.substr( 0, s.size() - 2 ) + "],\n";
  return s;
}
// writes JSON File for polygon Test cases, can be visualized with showPolygon.py
// visualizes the polygon, the groundtruth and the points found using the polygon iterator
template<typename Scalar>
void writeReportToFile( const GridMap<Eigen::Index> &actual_map,
                        const GridMap<Eigen::Index> &expected_map,
                        const hector_math::Polygon<Scalar> &polygon, Eigen::Index row_min,
                        Eigen::Index row_max, Eigen::Index col_min, Eigen::Index col_max,
                        int offset, const std::string &name )
{
  std::string package_path = ros::package::getPath( ROS_PACKAGE_NAME );
  std::string path = package_path + "/test/tmp/" + name;
  std::ofstream output( path, std::ios_base::out );
  if ( !output.is_open() ) {
    std::cerr << "Couldn't open file to dump report!" << std::endl;
    return;
  }
  output << "{\n \"limits\": [";
  output << row_min << ", " << row_max << ", " << col_min << ", " << col_max << "]," << std::endl;
  output << "\"iterated positions\":";
  output << jsonVector( actual_map, offset );
  output << "\"real positions\":";
  output << jsonVector( expected_map, offset );
  output << "\"corners\": [";
  for ( int i = 0; i < polygon.cols(); i++ ) {
    output << "[" << polygon( 0, i ) << ", " << polygon( 1, i );
    if ( i != polygon.cols() - 1 ) {
      output << "],";
    } else {
      output << "]]" << std::endl << "}";
    }
  }
  output.flush();
  output.close();
  std::cout << "Wrote file" << path << std::endl;
}

template<typename Scalar>
class IteratorTest : public testing::Test
{
};

typedef testing::Types<float, double> Implementations;

TYPED_TEST_CASE( IteratorTest, Implementations );

TYPED_TEST( IteratorTest, circleTest )
{
  using Scalar = TypeParam;
  using Vector2 = Vector2<Scalar>;
  // normal case in area x: -4 bis 4 and y: -4 bis 4, center (0,0) and radius 2,
  GridMap<Eigen::Index> expected_map( 5, 5 );
  // @formatter:off
  // clang-format off
  expected_map << 0, 0, 1, 0, 0,
                  0, 1, 1, 1, 0,
                  1, 1, 1, 1, 0,
                  0, 1, 1, 1, 0,
                  0, 0, 0, 0, 0;
  // @formatter:on
  // clang-format on
  GridMap<Eigen::Index> actual_map( 5, 5 );
  actual_map.setZero();
  iterateCircle<Scalar>(
      Vector2( 0.49, 0.49 ), 2, -4, 4, -4, 4,
      [&actual_map]( Eigen::Index x, Eigen::Index y ) { actual_map( x + 2, y + 2 ) += 1; } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Circle with radius 2 at (0.49, 0.49).";

  // case 2, equal to one but center at (0,0)
  // @formatter:off
  // clang-format off
  expected_map << 0, 1, 1, 0, 0,
                  1, 1, 1, 1, 0,
                  1, 1, 1, 1, 0,
                  0, 1, 1, 0, 0,
                  0, 0, 0, 0, 0;
  // @formatter:on
  // clang-format on
  actual_map.setZero();
  iterateCircle<Scalar>(
      Vector2( 0, 0 ), 2, -4, 4, -4, 4,
      [&actual_map]( Eigen::Index x, Eigen::Index y ) { actual_map( x + 2, y + 2 ) += 1; } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Circle with radius 2 at (0, 0).";

  // case 3 restrict max_row to be 1
  expected_map = GridMap<Eigen::Index>( 4, 5 );
  actual_map = GridMap<Eigen::Index>( 4, 5 );
  actual_map.setZero();
  // @formatter:off
  // clang-format off
  expected_map << 0, 0, 0, 0, 0,
                  0, 0, 1, 1, 0,
                  0, 1, 1, 1, 1,
                  0, 1, 1, 1, 1;
  // @formatter:on
  // clang-format on
  iterateCircle<Scalar>(
      Vector2( 0, 0 ), 2, -4, 1, -4, 4,
      [&actual_map]( Eigen::Index x, Eigen::Index y ) { actual_map( x + 3, y + 3 ) += 1; } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Circle with radius 2 at (0, 0) with limited row max.";

  // case 4 restrict min_row to be 1
  expected_map = GridMap<Eigen::Index>( 3, 4 );
  actual_map = GridMap<Eigen::Index>( 3, 4 );
  actual_map.setZero();
  // @formatter:off
  // clang-format off
  expected_map << 0, 0, 0, 0,
                  0, 1, 1, 0,
                  0, 0, 0, 0;
  // @formatter:on
  // clang-format on
  iterateCircle<Scalar>(
      Vector2( 0, 0 ), 2, 1, 4, -4, 4,
      [&actual_map]( Eigen::Index x, Eigen::Index y ) { actual_map( x + 0, y + 2 ) += 1; } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Circle with radius 2 at (0, ) with limited row min.";
  // case 5 restrict min_column to be -1 and max_column to be 1
  expected_map = GridMap<Eigen::Index>( 6, 4 );
  actual_map = GridMap<Eigen::Index>( 6, 4 );
  actual_map.setZero();
  // @formatter:off
  // clang-format off
  expected_map << 0, 0, 0, 0,
                  0, 1, 1, 0,
                  0, 1, 1, 0,
                  0, 1, 1, 0,
                  0, 1, 1, 0,
                  0, 0, 0, 0;
  // @formatter:on
  // clang-format on
  iterateCircle<Scalar>(
      Vector2( 0, 0 ), 2, -4, 4, -1, 1,
      [&actual_map]( Eigen::Index x, Eigen::Index y ) { actual_map( x + 3, y + 2 ) += 1; } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Circle with radius 2 at (0, ) with limited col min and max.";

  // case 6 no points due to column/index restrictions
  expected_map = GridMap<Eigen::Index>( 6, 4 );
  actual_map = GridMap<Eigen::Index>( 6, 4 );
  actual_map.setZero();
  expected_map.setZero();
  iterateCircle<Scalar>(
      Vector2( 0, 0 ), 2, 4, 8, -8, 8,
      [&actual_map]( Eigen::Index x, Eigen::Index y ) { actual_map( x + 3, y + 3 ) += 1; } );
  iterateCircle<Scalar>(
      Vector2( 0, 0 ), 2, -8, 8, 4, 8,
      [&actual_map]( Eigen::Index x, Eigen::Index y ) { actual_map( x + 3, y + 3 ) += 1; } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Circle with radius 2 at (0, 0) entirely outside of iterated area.";
  // case 7, using different function overload
  expected_map = GridMap<Eigen::Index>( 4, 4 );
  actual_map = GridMap<Eigen::Index>( 4, 4 );
  actual_map.setZero();
  // @formatter:off
  // clang-format off
  expected_map << 1, 1, 0, 0,
                  1, 0, 0, 0,
                  0, 0, 0, 0,
                  0, 0, 0, 0;
  // @formatter:on
  // clang-format on
  iterateCircle<Scalar>( Vector2( 0, 0 ), 2, 4, 4, [&actual_map]( Eigen::Index x, Eigen::Index y ) {
    actual_map( x, y ) += 1;
  } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) );
}

TYPED_TEST( IteratorTest, polygonTest )
{
  using Scalar = TypeParam;
  constexpr bool FORCE_TEST_OUTPUT = true;
  constexpr int offset = 5;
  Polygon<Scalar> polygon = createPolygon<Scalar>( PolygonTyp::RandomStructure );
  // normal case in area x: -4 bis 4 and y: -4 bis 4, center (0,0) and radius 2,
  Eigen::Index row_min = -6;
  Eigen::Index row_max = 6;
  Eigen::Index col_min = -6;
  Eigen::Index col_max = 6;
  GridMap<Eigen::Index> expected_map( 10, 10 );
  GridMap<Eigen::Index> actual_map( 10, 10 );
  actual_map.setZero();
  // @formatter:off
  // clang-format off
  expected_map << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 1, 1, 1, 1, 0, 0,
                  0, 0, 0, 0, 1, 1, 1, 1, 1, 0,
                  0, 0, 0, 1, 1, 1, 1, 1, 0, 0,
                  0, 0, 0, 1, 1, 1, 1, 1, 1, 0,
                  0, 0, 0, 0, 0, 1, 0, 1, 1, 1,
                  0, 0, 0, 0, 0, 1, 0, 1, 1, 1,
                  0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
                  0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  // @formatter:on
  // clang-format on
  iteratePolygon<Scalar>( polygon, row_min, row_max, col_min, col_max,
                          [&actual_map, offset]( Eigen::Index x, Eigen::Index y ) {
                            actual_map( x + offset, y + offset ) += 1;
                          } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) ) << "Random Structure Polygon";
  if ( FORCE_TEST_OUTPUT || !EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
    writeReportToFile( actual_map, expected_map, polygon, row_min, row_max, col_min, col_max,
                       offset, "TestCasePolygonRandom.txt" );
  // case Z
  polygon = createPolygon<Scalar>( PolygonTyp::Z_Shape );
  actual_map.setZero();
  // @formatter:off
  // clang-format off
  expected_map << 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
                  0, 1, 1, 0, 0, 0, 0, 0, 1, 0,
                  0, 1, 0, 1, 0, 0, 0, 0, 1, 0,
                  0, 1, 0, 0, 1, 1, 0, 0, 1, 0,
                  0, 1, 0, 0, 0, 0, 1, 0, 1, 0,
                  0, 1, 0, 0, 0, 0, 0, 1, 1, 0,
                  0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  // @formatter:on
  // clang-format on
  iteratePolygon<Scalar>( polygon, row_min, row_max, col_min, col_max,
                          [&actual_map, offset]( Eigen::Index x, Eigen::Index y ) {
                            actual_map( x + offset, y + offset ) += 1;
                          } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) ) << "Z-Shape.";
  if ( FORCE_TEST_OUTPUT || !EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
    writeReportToFile( actual_map, expected_map, polygon, row_min, row_max, col_min, col_max,
                       offset, "TestCasePolygonZShape.txt" );
  // circle approximation
  polygon = createPolygon<Scalar>( PolygonTyp::Circle );
  actual_map.setZero();
  // @formatter:off
  // clang-format off
  expected_map << 0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
                  0, 1, 1, 1, 1, 1, 1, 1, 1, 0,
                  0, 1, 1, 1, 1, 1, 1, 1, 1, 0,
                  0, 1, 1, 1, 1, 1, 1, 1, 1, 0,
                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                  0, 1, 1, 1, 1, 1, 1, 1, 1, 0,
                  0, 1, 1, 1, 1, 1, 1, 1, 1, 0,
                  0, 1, 1, 1, 1, 1, 1, 1, 1, 0,
                  0, 0, 0, 0, 1, 1, 0, 0, 0, 0;
  // @formatter:on
  // clang-format on
  iteratePolygon<Scalar>( polygon, row_min, row_max, col_min, col_max,
                          [&actual_map, offset]( Eigen::Index x, Eigen::Index y ) {
                            actual_map( x + offset, y + offset ) += 1;
                          } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) ) << "Circle Polygon Approx.";
  if ( FORCE_TEST_OUTPUT || !EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
    writeReportToFile( actual_map, expected_map, polygon, row_min, row_max, col_min, col_max,
                       offset, "TestCaseCircleShape.txt" );

  // u - shape
  polygon = createPolygon<Scalar>( PolygonTyp::U_Shape );
  actual_map.setZero();
  // @formatter:off
  // clang-format off
  expected_map << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 1, 1, 1, 1, 1, 1, 0,
                  0, 0, 0, 1, 1, 1, 1, 1, 1, 0,
                  0, 0, 0, 1, 1, 0, 0, 0, 0, 0,
                  0, 0, 0, 1, 1, 0, 0, 0, 0, 0,
                  0, 0, 0, 1, 1, 0, 0, 0, 0, 0,
                  0, 0, 0, 1, 1, 1, 1, 1, 1, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
                  0, 0, 0, 0, 0, 1, 1, 1, 1, 0,
                  0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
  // @formatter:on
  // clang-format on
  iteratePolygon<Scalar>( polygon, row_min, row_max, col_min, col_max,
                          [&actual_map, offset]( Eigen::Index x, Eigen::Index y ) {
                            actual_map( x + offset, y + offset ) += 1;
                          } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) );
  if ( FORCE_TEST_OUTPUT || !EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
    writeReportToFile( actual_map, expected_map, polygon, row_min, row_max, col_min, col_max,
                       offset, "TestCaseUShape.txt" );

  // circle approximation with limited indexes
  row_min = -4;
  row_max = 2;
  col_min = -3;
  col_max = 1;
  polygon = createPolygon<Scalar>( PolygonTyp::Circle );
  actual_map.setZero();
  // @formatter:off
  // clang-format off
  expected_map << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
                  0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
                  0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
                  0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
                  0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
                  0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  // @formatter:on
  // clang-format on
  iteratePolygon<Scalar>( polygon, row_min, row_max, col_min, col_max,
                          [&actual_map, offset]( Eigen::Index x, Eigen::Index y ) {
                            actual_map( x + offset, y + offset ) += 1;
                          } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Circle Approx with limited indexes";
  if ( FORCE_TEST_OUTPUT || !EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
    writeReportToFile( actual_map, expected_map, polygon, row_min, row_max, col_min, col_max,
                       offset, "TestCaseCircleShapeLimitedIndexes.txt" );

  // u - shape limited index
  polygon = createPolygon<Scalar>( PolygonTyp::U_Shape );
  actual_map.setZero();
  // @formatter:off
  // clang-format off
  expected_map << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 1, 1, 1, 0, 0, 0, 0,
                  0, 0, 0, 1, 1, 1, 0, 0, 0, 0,
                  0, 0, 0, 1, 1, 0, 0, 0, 0, 0,
                  0, 0, 0, 1, 1, 0, 0, 0, 0, 0,
                  0, 0, 0, 1, 1, 0, 0, 0, 0, 0,
                  0, 0, 0, 1, 1, 1, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  // @formatter:on
  // clang-format on
  iteratePolygon<Scalar>( polygon, row_min, row_max, col_min, col_max,
                          [&actual_map, offset]( Eigen::Index x, Eigen::Index y ) {
                            actual_map( x + offset, y + offset ) += 1;
                          } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) ) << "U Shape with limited indexes";
  if ( FORCE_TEST_OUTPUT || !EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
    writeReportToFile( actual_map, expected_map, polygon, row_min, row_max, col_min, col_max,
                       offset, "TestCaseUShapeLimitedIndexes.txt" );
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "test_hector_iterators" );
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
