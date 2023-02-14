//
// Created by stefan on 08.04.22.
//

#include "eigen_tests.h"
#include <hector_math/math/quaternion_binning.h>

using namespace hector_math;

template<typename T>
class QuaternionBinning : public ::testing::Test
{
};

typedef ::testing::Types<double, float> MyTypes;
TYPED_TEST_CASE( QuaternionBinning, MyTypes );

TYPED_TEST( QuaternionBinning, bitOperations )
{
  EXPECT_EQ( detail::computeRequiredBits( 0 ), 0 );
  EXPECT_EQ( detail::computeRequiredBits( 1 ), 1 );
  EXPECT_EQ( detail::computeRequiredBits( 2 ), 2 );
  EXPECT_EQ( detail::computeRequiredBits( 3 ), 2 );
  EXPECT_EQ( detail::computeRequiredBits( 4 ), 3 );
  EXPECT_EQ( detail::computeRequiredBits( 1023 ), 10 );
  EXPECT_EQ( detail::computeRequiredBits( 1024 ), 11 );

  EXPECT_EQ( detail::computeBitMask( 0 ), 0 );
  EXPECT_EQ( detail::computeBitMask( 1 ), 1 );
  EXPECT_EQ( detail::computeBitMask( 2 ), 0b11 );
  EXPECT_EQ( detail::computeBitMask( 3 ), 0b111 );
  EXPECT_EQ( detail::computeBitMask( 8 ), 0b11111111 );
}

TYPED_TEST( QuaternionBinning, quaternionBinning )
{
  using Scalar = TypeParam;
  Scalar theta, phi = 0;
  const int fibonacci_sqrt = 50;
  Scalar angle_increment = M_PI / 32;
  while ( phi < 2 * M_PI ) {
    theta = M_PI * ( -1 / 2.0 );
    while ( theta < M_PI / 2 ) {
      Eigen::Quaternion<Scalar> q{ 0, cos( phi ) * sin( theta ), sin( phi ) * sin( theta ),
                                   cos( theta ) };
      auto index = computeBin<Scalar, fibonacci_sqrt, fibonacci_sqrt,
                              quaternion_binning_modes::SphericalFibonacci>( q );
      auto vec = computeDirectionFromBin<Scalar, fibonacci_sqrt, fibonacci_sqrt,
                                         quaternion_binning_modes::SphericalFibonacci>( index );
      Scalar distance =
          sqrt( pow( q.x() - vec[0], 2 ) + pow( q.y() - vec[1], 2 ) + pow( q.z() - vec[2], 2 ) );
      EXPECT_TRUE( distance < 0.1 ); // expected max distance depends on fibonacci_sqrt
      theta += angle_increment;
    }
    phi += angle_increment;
  }
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
