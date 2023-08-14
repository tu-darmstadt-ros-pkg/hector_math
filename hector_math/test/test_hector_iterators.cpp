// Copyright (c) 2022 Aljoscha Schmidt. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "iterator_test_input.h"
#include <hector_math/iterators/circle_iterator.h>
#include <hector_math/iterators/polygon_iterator.h>
#include <hector_math/iterators/rectangle_iterator.h>

#include "eigen_tests.h"
#include <fstream>
#include <gtest/gtest.h>
#include <ros/package.h>

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

TYPED_TEST_SUITE( IteratorTest, Implementations );

TYPED_TEST( IteratorTest, rectangleTest )
{
  using Scalar = TypeParam;
  using Vector2S = Vector2<Scalar>;
  using Vector2I = Vector2<Eigen::Index>;
  GridMap<Eigen::Index> expected_map( 5, 5 );
  // @formatter:off
  // clang-format off
  expected_map << 0, 0, 0, 0, 0,
                  1, 1, 1, 1, 0,
                  1, 1, 1, 1, 0,
                  1, 1, 1, 1, 0,
                  0, 0, 0, 0, 0;
  // @formatter:on
  // clang-format on
  GridMap<Eigen::Index> actual_map( 5, 5 );
  actual_map.setZero();
  iterateRectangle<Scalar>(
      Vector2S( 1, 0 ), Vector2S( 1, 4 ), Vector2S( 4, 0 ),
      [&actual_map]( Eigen::Index x, Eigen::Index y ) { actual_map( x, y ) += 1; } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Rectangle with a (0, 1), b (4, 1) and c (0, 3).";

  // Try also with non-floating point rectangle
  actual_map.setZero();
  iterateRectangle<Eigen::Index>(
      Vector2I( 1, 0 ), Vector2I( 1, 4 ), Vector2I( 4, 0 ),
      [&actual_map]( Eigen::Index x, Eigen::Index y ) { actual_map( x, y ) += 1; } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Rectangle with a (0, 1), b (4, 1) and c (0, 3) and integers.";

  // @formatter:off
  // clang-format off
  expected_map << 1, 0, 0, 0, 0,
                  0, 1, 1, 0, 0,
                  0, 1, 1, 1, 0,
                  0, 0, 1, 1, 0,
                  0, 0, 0, 0, 1;
  // @formatter:on
  // clang-format on
  actual_map.setZero();
  iterateRectangle<Scalar>(
      Vector2S( 0.4, 0.4 ), Vector2S( 3.4, 1.6 ), Vector2S( 1.6, 3.4 ),
      [&actual_map]( Eigen::Index x, Eigen::Index y ) { actual_map( x, y ) += 1; } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Rectangle with a (0.4, 0.4), b (3.4, 1.6) and c (1.6, 3.4).";
  // swapped corners b and c -> should result in same expected map
  actual_map.setZero();
  iterateRectangle<Scalar>(
      Vector2S( 0.4, 0.4 ), Vector2S( 1.6, 3.4 ), Vector2S( 3.4, 1.6 ),
      [&actual_map]( Eigen::Index x, Eigen::Index y ) { actual_map( x, y ) += 1; } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Rectangle with a (0.4, 0.4), b (3.4, 1.6) and c (1.6, 3.4).";
  // no points in rectangle
  expected_map.setZero();
  actual_map.setZero();
  iterateRectangle<Scalar>(
      Vector2S( 1.6, 0 ), Vector2S( 2.4, 0 ), Vector2S( 1.6, 4.4 ),
      [&actual_map]( Eigen::Index x, Eigen::Index y ) { actual_map( x, y ) += 1; } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Rectangle with a (1.6, 0), b (2.4, 0) and c (1.6, 4.4).";
  // test max rows and max cols argument
  // @formatter:off
  // clang-format off
  expected_map << 1, 0, 0, 0, 0,
                  0, 1, 1, 0, 0,
                  0, 1, 1, 1, 0,
                  0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0;
  // @formatter:on
  // clang-format on
  actual_map.setZero();
  iterateRectangle<Scalar>(
      Vector2S( 0.4, 0.4 ), Vector2S( 3.4, 1.6 ), Vector2S( 1.6, 3.4 ), 3, 4,
      [&actual_map]( Eigen::Index x, Eigen::Index y ) { actual_map( x, y ) += 1; } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Rectangle with a (0.4, 0.4), b (3.4, 1.6) and c (1.6, 3.4).";

  // test min/max rows and min/max cols argument
  // @formatter:off
  // clang-format off
  expected_map << 0, 0, 0, 0, 0,
                  0, 0, 1, 0, 0,
                  0, 0, 1, 1, 0,
                  0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0;
  // @formatter:on
  // clang-format on
  actual_map.setZero();
  iterateRectangle<Scalar>(
      Vector2S( 0.4, 0.4 ), Vector2S( 3.4, 1.6 ), Vector2S( 1.6, 3.4 ), 1, 3, 2, 4,
      [&actual_map]( Eigen::Index x, Eigen::Index y ) { actual_map( x, y ) += 1; } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Rectangle with a (0.4, 0.4), b (3.4, 1.6) and c (1.6, 3.4).";

  expected_map = GridMap<Eigen::Index>( 20, 20 );
  // @formatter:off
  // clang-format off
  expected_map << 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
                  0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
                  0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
                  0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
                  0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
                  0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
                  0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
                  0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  // @formatter:on
  // clang-format on
  actual_map = GridMap<Eigen::Index>::Zero( 20, 20 );
  iterateRectangle<Scalar>(
      Vector2S( 0, 1 ), Vector2S( 1, 19 ), Vector2S( 18, 0 ),
      [&actual_map]( Eigen::Index x, Eigen::Index y ) { ++actual_map( x, y ); } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Rectangle with a (1, 2), b (0, 5) and c (9, 4).";
}

TYPED_TEST( IteratorTest, circleTest )
{
  using Scalar = TypeParam;
  using Vector2S = Vector2<Scalar>;
  using Vector2I = Vector2<Eigen::Index>;
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
      Vector2S( 0.49, 0.49 ), 2, -4, 4, -4, 4,
      [&actual_map]( Eigen::Index x, Eigen::Index y ) { actual_map( x + 2, y + 2 ) += 1; } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Circle with radius 2 at (0.49, 0.49) with negative x- and y- indices.";

  // case 2, equal to one but center at (2,2)
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
      Vector2S( 2, 2 ), 2, 0, 6, 0, 6, [&actual_map]( Eigen::Index x, Eigen::Index y ) {
        EXPECT_TRUE( x >= 0 and x < actual_map.rows() and y >= 0 and y < actual_map.cols() );
        actual_map( x, y ) += 1;
      } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Circle with radius 2 at (2, 2).";

  // Try with non-floating point values
  actual_map.setZero();
  iterateCircle<Eigen::Index>(
      Vector2I( 2, 2 ), 2, 0, 6, 0, 6, [&actual_map]( Eigen::Index x, Eigen::Index y ) {
        EXPECT_TRUE( x >= 0 and x < actual_map.rows() and y >= 0 and y < actual_map.cols() );
        actual_map( x, y ) += 1;
      } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Circle with radius 2 at (2, 2) and integer.";

  // case 3 restrict max_row to be 1
  actual_map.setZero();
  // @formatter:off
  // clang-format off
  expected_map << 0, 1, 1, 0, 0,
                  1, 1, 1, 1, 0,
                  1, 1, 1, 1, 0,
                  0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0;
  // @formatter:on
  // clang-format on
  iterateCircle<Scalar>(
      Vector2S( 2, 2 ), 2, 0, 3, 0, 6, [&actual_map]( Eigen::Index x, Eigen::Index y ) {
        EXPECT_TRUE( x >= 0 and x < actual_map.rows() and y >= 0 and y < actual_map.cols() );
        actual_map( x, y ) += 1;
      } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Circle with radius 2 at (2, 2) with limited row max.";

  // case 4 restrict min_row to be 1
  actual_map.setZero();
  // @formatter:off
  // clang-format off
  expected_map << 0, 0, 0, 0, 0,
                1, 1, 1, 1, 0,
                1, 1, 1, 1, 0,
                0, 1, 1, 0, 0,
                0, 0, 0, 0, 0;
  // @formatter:on
  // clang-format on
  iterateCircle<Scalar>(
      Vector2S( 2, 2 ), 2, 1, 6, 0, 6, [&actual_map]( Eigen::Index x, Eigen::Index y ) {
        EXPECT_TRUE( x >= 0 and x < actual_map.rows() and y >= 0 and y < actual_map.cols() );
        actual_map( x, y ) += 1;
      } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Circle with radius 2 at (2, 2) with limited row min.";
  // case 5 restrict min_column to be 1 and max_column to be 3
  actual_map.setZero();
  // @formatter:off
  // clang-format off
  expected_map << 0, 1, 1, 0, 0,
                0, 1, 1, 0, 0,
                0, 1, 1, 0, 0,
                0, 1, 1, 0, 0,
                0, 0, 0, 0, 0;
  // @formatter:on
  // clang-format on
  iterateCircle<Scalar>(
      Vector2S( 2, 2 ), 2, 0, 6, 1, 3, [&actual_map]( Eigen::Index x, Eigen::Index y ) {
        EXPECT_TRUE( x >= 0 and x < actual_map.rows() and y >= 0 and y < actual_map.cols() );
        actual_map( x, y ) += 1;
      } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Circle with radius 2 at (2, 2) with limited col min and max.";

  // case 6 no points due to column/index restrictions
  expected_map = GridMap<Eigen::Index>( 6, 4 );
  actual_map = GridMap<Eigen::Index>( 6, 4 );
  actual_map.setZero();
  expected_map.setZero();
  iterateCircle<Scalar>(
      Vector2S( 2, 2 ), 2, 5, 8, -8, 8, [&actual_map]( Eigen::Index x, Eigen::Index y ) {
        EXPECT_TRUE( x >= 0 and x < actual_map.rows() and y >= 0 and y < actual_map.cols() );
        actual_map( x + 3, y + 3 ) += 1;
      } );
  iterateCircle<Scalar>(
      Vector2S( 2, 2 ), 2, -8, 8, 5, 8, [&actual_map]( Eigen::Index x, Eigen::Index y ) {
        EXPECT_TRUE( x >= 0 and x < actual_map.rows() and y >= 0 and y < actual_map.cols() );
        actual_map( x + 3, y + 3 ) += 1;
      } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Circle with radius 2 at (2, 2) entirely outside of iterated area.";
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
  iterateCircle<Scalar>( Vector2S( 0, 0 ), 2, 4, 4, [&actual_map]( Eigen::Index x, Eigen::Index y ) {
    EXPECT_TRUE( x >= 0 and x < actual_map.rows() and y >= 0 and y < actual_map.cols() );
    actual_map( x, y ) += 1;
  } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) );
}

TYPED_TEST( IteratorTest, polygonTest )
{
  using Scalar = TypeParam;
  const bool FORCE_TEST_OUTPUT = true;
  int offset = 5;
  Polygon<Scalar> polygon = createPolygon<Scalar>( PolygonTyp::RandomStructureNegativeIndices );
  // RandomStructure case in area x: -6 bis 6 and y: -6 bis 6,
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
                            EXPECT_TRUE( x + offset >= 0 and x + offset < actual_map.rows() and
                                         y + offset >= 0 and y + offset < actual_map.cols() );
                            actual_map( x + offset, y + offset ) += 1;
                          } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
      << "Random Structure Polygon with corner with negative indices";
  if ( FORCE_TEST_OUTPUT || !EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
    writeReportToFile( actual_map, expected_map, polygon, row_min, row_max, col_min, col_max,
                       offset, "TestCasePolygonRandom_NegativeIndices.txt" );

  // RandomStructure case in area x: 0 bis 10 and y: 0 bis 10,
  polygon = createPolygon<Scalar>( PolygonTyp::RandomStructure );
  row_min = 0;
  row_max = 10;
  col_min = 0;
  col_max = 10;
  offset = 0;
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
  iteratePolygon<Scalar>(
      polygon, row_min, row_max, col_min, col_max, [&actual_map]( Eigen::Index x, Eigen::Index y ) {
        EXPECT_TRUE( x >= 0 and x < actual_map.rows() and y >= 0 and y < actual_map.cols() );
        actual_map( x, y ) += 1;
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
  iteratePolygon<Scalar>(
      polygon, row_min, row_max, col_min, col_max, [&actual_map]( Eigen::Index x, Eigen::Index y ) {
        EXPECT_TRUE( x >= 0 and x < actual_map.rows() and y >= 0 and y < actual_map.cols() );
        actual_map( x, y ) += 1;
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
  iteratePolygon<Scalar>(
      polygon, row_min, row_max, col_min, col_max, [&actual_map]( Eigen::Index x, Eigen::Index y ) {
        EXPECT_TRUE( x >= 0 and x < actual_map.rows() and y >= 0 and y < actual_map.cols() );
        actual_map( x, y ) += 1;
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
                            EXPECT_TRUE( x >= 0 and x < actual_map.rows() and y >= 0 and
                                         y < actual_map.cols() );
                            actual_map( x + offset, y + offset ) += 1;
                          } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) );
  if ( FORCE_TEST_OUTPUT || !EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
    writeReportToFile( actual_map, expected_map, polygon, row_min, row_max, col_min, col_max,
                       offset, "TestCaseUShape.txt" );

  // Line test, can not iterate over Line because it is not a polygon
  polygon = createPolygon<Scalar>( PolygonTyp::Line );
  actual_map.setZero();
  expected_map.setZero();
  iteratePolygon<Scalar>( polygon, row_min, row_max, col_min, col_max,
                          [&actual_map, offset]( Eigen::Index x, Eigen::Index y ) {
                            EXPECT_TRUE( x >= 0 and x < actual_map.rows() and y >= 0 and
                                         y < actual_map.cols() );
                            actual_map( x + offset, y + offset ) += 1;
                          } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) );
  if ( FORCE_TEST_OUTPUT || !EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
    writeReportToFile( actual_map, expected_map, polygon, row_min, row_max, col_min, col_max,
                       offset, "TestCaseLine.txt" );
  // circle approximation with limited indexes
  row_min = 1;
  row_max = 7;
  col_min = 2;
  col_max = 6;
  polygon = createPolygon<Scalar>( PolygonTyp::Circle );
  actual_map.setZero();
  // @formatter:off
  // clang-format off
  row_min = -1;
  expected_map << 0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
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
  iteratePolygon<Scalar>(
      polygon, row_min, row_max, col_min, col_max, [&actual_map]( Eigen::Index x, Eigen::Index y ) {
        EXPECT_TRUE( x >= 0 and x < actual_map.rows() and y >= 0 and y < actual_map.cols() );
        actual_map( x, y ) += 1;
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
  row_min = 1;
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
  iteratePolygon<Scalar>(
      polygon, row_min, row_max, col_min, col_max, [&actual_map]( Eigen::Index x, Eigen::Index y ) {
        EXPECT_TRUE( x >= 0 and x < actual_map.rows() and y >= 0 and y < actual_map.cols() );
        actual_map( x, y ) += 1;
      } );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) ) << "U Shape with limited indexes";
  if ( FORCE_TEST_OUTPUT || !EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
    writeReportToFile( actual_map, expected_map, polygon, row_min, row_max, col_min, col_max,
                       offset, "TestCaseUShapeLimitedIndexes.txt" );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
