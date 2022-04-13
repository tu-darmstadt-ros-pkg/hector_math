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

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
