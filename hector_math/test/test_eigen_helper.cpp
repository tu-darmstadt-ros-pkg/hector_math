#include <hector_math/helpers/eigen.h>

#include "eigen_tests.h"
#include <gtest/gtest.h>
#include <hector_math/types.h>

using namespace hector_math;

template<typename Scalar>
class EigenHelperTest : public testing::Test
{
};

typedef testing::Types<float, double> Implementations;

TYPED_TEST_CASE( EigenHelperTest, Implementations );

TYPED_TEST( EigenHelperTest, wrap_with_constant )
{
  using Scalar = TypeParam;
  const Scalar NaN = std::numeric_limits<Scalar>::quiet_NaN();
  const Scalar MAX = std::numeric_limits<Scalar>::max();

  GridMap<Scalar> start_map( 3, 3 );
  // clang-format off
  start_map << 0, 1, 1,
               1, 2, 3,
               0, 1, 4;
  // clang-format on
  GridMap<Scalar> expected_map( 6, 6 );
  // clang-format off
  expected_map <<0,0,0,0,0,0,
                 0,0,0,1,1,0,
                 0,0,1,2,3,0,
                 0,0,0,1,4,0,
                 0,0,0,0,0,0,
                 0,0,0,0,0,0;
  // clang-format on
  GridMap<Scalar> actual_map =
      eigen::wrapWithConstant<Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>>(
          start_map, Scalar( 0 ), 6, 6, 1, 2 );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) ) << "Wrap with constant Test Case 1";

  // Test Case 2 no shift and NaN constant value
  // clang-format off
  expected_map <<
       0,  1,  1, NaN,NaN,NaN,
       1,  2,  3, NaN,NaN,NaN,
       0,  1,  4, NaN,NaN,NaN,
      NaN,NaN,NaN,NaN,NaN,NaN,
      NaN,NaN,NaN,NaN,NaN,NaN,
      NaN,NaN,NaN,NaN,NaN,NaN;
  // clang-format on
  actual_map = eigen::wrapWithConstant<Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>>(
      start_map, NaN, 6, 6, 0, 0 );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) ) << "Wrap with constant Test Case 2";

  // Test Case 3 no shift and NaN constant value
  // clang-format off
  expected_map <<
      MAX,MAX,MAX,MAX,MAX,MAX,
      MAX,MAX,MAX,MAX,MAX,MAX,
      MAX,MAX,MAX,MAX,MAX,MAX,
      MAX,MAX,MAX, 0,  1,  1,
      MAX,MAX,MAX, 1,  2,  3,
      MAX,MAX,MAX, 0,  1,  4;
  // clang-format on
  actual_map = eigen::wrapWithConstant<Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>>(
      start_map, MAX, 6, 6, 3, 3 );
  EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) ) << "Wrap with constant Test Case 3";
}

TYPED_TEST( EigenHelperTest, shift )
{
  using Scalar = TypeParam;
  const Scalar NaN = std::numeric_limits<Scalar>::quiet_NaN();
  const Scalar MAX = std::numeric_limits<Scalar>::max();
  {
    GridMap<Scalar> start_map( 3, 3 );
    GridMap<Scalar> actual_map( 3, 3 );
    GridMap<Scalar> expected_map( 3, 3 );
    // TestCase see example in eigen.h
    // clang-format off
    start_map << 1, 2, 3,
                 4, 5, 6,
                 7, 8, 9;
    expected_map << 8, 9, 7,
                    2, 3, 1,
                    5, 6, 4;
    // clang-format on
    actual_map =
        eigen::shift<Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>>( start_map, 2, 1 );
    EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
        << "Shift Test Case 0, see example in helpers/eigen.h";
    // clang-format off
    start_map << 0,    1,  2,
                 3,    4,  5,
                 NaN, MAX,-1;
    // clang-format on

    // TestCase 1 No shift
    // clang-format off
    expected_map <<   0,   1,  2,
                      3,   4,  5,
                    NaN, MAX, -1;
    // clang-format on
    actual_map =
        eigen::shift<Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>>( start_map, 0, 0 );
    EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) ) << "Shift Test Case 1";

    // Test Case 2 row_shift = 1,
    // clang-format off
   expected_map << 3,   4,   5,
                  NaN, MAX, -1,
                   0,   1,   2;
    // clang-format on
    actual_map =
        eigen::shift<Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>>( start_map, 1, 0 );
    EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) ) << "Shift Test Case 2";

    // Test Case 3 column_shift = 1,
    // clang-format off
   expected_map << 1,  2,  0,
                   4 , 5,  3,
                  MAX, -1, NaN;
    // clang-format on
    actual_map =
        eigen::shift<Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>>( start_map, 0, 1 );
    EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) ) << "Shift Test Case 3";

    // Test Case 4 row_shift = 1, column_shift = 2
    // clang-format off
    expected_map <<  5,   3,   4,
                     -1, NaN, MAX,
                     2,   0,   1;
    // clang-format on
    actual_map =
        eigen::shift<Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>>( start_map, 1, 2 );
    EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) ) << "Shift Test Case 4";

    // Test Case 5 row_shift = -5, column_shift = -4 // should be equal to (-1,2) and (2,1)
    // clang-format off
    expected_map <<  5, 3,    4,
                    -1, NaN, MAX,
                     2,  0,   1;
    // clang-format on
    actual_map =
        eigen::shift<Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>>( start_map, -5, -4 );
    EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) ) << "Shift Test Case 5";

    // Test Case 6 non-square bigger map
    start_map = GridMap<Scalar>( 4, 5 );
    actual_map = GridMap<Scalar>( 4, 5 );
    // clang-format off
    start_map << 0, 1,   2, 3,  4,
                 5, 6,   7, 8,  9,
                10, 11, 12, 13, 14,
                NaN,MAX,-1, 0,  0;
    // clang-format on
    expected_map = GridMap<Scalar>( 4, 5 );
    // clang-format off
    expected_map << 9,  5,  6,  7,  8,
                   14, 10, 11, 12, 13,
                    0,NaN,MAX, -1,  0,
                    4,  0,  1,  2,  3;
    // clang-format on
    actual_map =
        eigen::shift<Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>>( start_map, 1, -1 );
    EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) ) << "Shift Test Case 6";
  }

  // Compile time known size 1D
  {
    Eigen::Array<Scalar, 3, 1> input;
    Eigen::Array<Scalar, 3, 1> actual;
    Eigen::Array<Scalar, 3, 1> expected;
    // clang-format off
    input << 1, 2, 3;
    expected << 2, 3, 1;
    // clang-format on
    actual = eigen::shift( input, 1, 0 );
    EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected, actual ) );
  }

  // Compile time known size 2D
  {
    Eigen::Array<Scalar, 3, 3> input;
    Eigen::Array<Scalar, 3, 3> actual;
    Eigen::Array<Scalar, 3, 3> expected;
    // clang-format off
    input << 1, 2, 3,
             4, 5, 6,
             7, 8, 9;
    expected << 2, 3, 1,
                5, 6, 4,
                8, 9, 7;
    // clang-format on
    actual = eigen::shift( input, 0, 1 );
    EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected, actual ) );
    // clang-format off
    expected << 4, 5, 6,
                7, 8, 9,
                1, 2, 3;
    // clang-format on
    actual = eigen::shift( input, 1, 0 );
    EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected, actual ) );
    // clang-format off
    expected << 5, 6, 4,
                8, 9, 7,
                2, 3, 1;
    // clang-format on
    actual = eigen::shift( input, 1, 1 );
    EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected, actual ) );
  }
}

TYPED_TEST( EigenHelperTest, flip )
{
  using Scalar = TypeParam;
  using eigen::flip_ops::FlipOp;
  const Scalar NaN = std::numeric_limits<Scalar>::quiet_NaN();
  const Scalar MAX = std::numeric_limits<Scalar>::max();
  GridMap<Scalar> start_map( 3, 3 );
  GridMap<Scalar> actual_map( 3, 3 );
  GridMap<Scalar> expected_map( 3, 3 );
  // clang-format off
  start_map << 0, 1, 2,
               3, 4, 5,
              NaN, MAX, -1;
  // clang-format on
  {
    // Test Case 1 flip_ops::Both
    // clang-format off
    expected_map <<  -1, MAX, NaN,
                      5,  4,  3,
                      2,  1,  0;
    // clang-format on
    actual_map = eigen::flip<Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>>( start_map );
    EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
        << "Flip Test Case 1: FlipOp Both";

    // Test Case 2 flip_ops:Rows
    // clang-format off
    expected_map <<  NaN, MAX, -1,
                      3,  4,  5,
                      0,  1,  2;
    // clang-format on
    actual_map =
        eigen::flip<Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>, FlipOp::Rows>( start_map );
    EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
        << "Flip Test Case 2: FlipOp Rows ";

    // Test Case 3 flip_ops:Columns
    // clang-format off
    expected_map <<  2, 1, 0,
                     5,  4, 3,
                     -1,  MAX,  NaN;
    // clang-format on
    actual_map = eigen::flip<Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>, FlipOp::Columns>(
        start_map );
    EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
        << "Flip Test Case 3: FlipOp Columns";

    // Test Case 4 flip_ops::Both
    // clang-format off
    expected_map <<  -1, MAX, NaN,
                      5,  4,  3,
                      2,  1,  0;
    // clang-format on
    actual_map =
        eigen::flip<Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>>( start_map, FlipOp::Both );
    EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
        << "Flip Test Case 4: Runtime FlipOp Both";

    // Test Case 5 flip_ops:Rows
    // clang-format off
    expected_map <<  NaN, MAX, -1,
                      3,  4,  5,
                      0,  1,  2;
    // clang-format on
    actual_map =
        eigen::flip<Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>>( start_map, FlipOp::Rows );
    EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
        << "Flip Test Case 5: Runtime FlipOp Rows ";

    // Test Case 6 flip_ops:Columns
    // clang-format off
    expected_map <<  2,  1,   0,
                     5,  4,   3,
                     -1, MAX, NaN;
    // clang-format on
    actual_map = eigen::flip<Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>>(
        start_map, FlipOp::Columns );
    EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected_map, actual_map ) )
        << "Flip Test Case 6: Runtime FlipOp Columns";
  }
  // Compile time known size 1D
  {
    Eigen::Array<Scalar, 3, 1> input;
    Eigen::Array<Scalar, 3, 1> actual;
    Eigen::Array<Scalar, 3, 1> expected;
    // clang-format off
    input << 1, 2, 3;
    expected << 3, 2, 1;
    // clang-format on
    actual = eigen::flip( input );
    EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected, actual ) );
  }

  // Compile time known size 2D
  {
    Eigen::Array<Scalar, 3, 3> input;
    Eigen::Array<Scalar, 3, 3> actual;
    Eigen::Array<Scalar, 3, 3> expected;
    // clang-format off
    input << 1, 2, 3,
             4, 5, 6,
             7, 8, 9;
    expected << 9, 8, 7,
                6, 5, 4,
                3, 2, 1;
    // clang-format on
    actual = eigen::flip( input, FlipOp::Both );
    EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected, actual ) );
    // clang-format off
    expected << 7, 8, 9,
                4, 5, 6,
                1, 2, 3;
    // clang-format on
    actual = eigen::flip( input, FlipOp::Rows );
    EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected, actual ) );
    // clang-format off
    expected << 3, 2, 1,
                6, 5, 4,
                9, 8, 7;
    // clang-format on
    actual = eigen::flip( input, FlipOp::Columns );
    EXPECT_TRUE( EIGEN_MATRIX_EQUAL( expected, actual ) );
  }
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
