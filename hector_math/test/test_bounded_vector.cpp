// Copyright (c) 2022 Aljoscha Schmidt. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <gtest/gtest.h>

#undef NDEBUG
#include <hector_math/containers/bounded_vector.h>

#include <memory>

using namespace hector_math;

template<typename Scalar>
class BoundedVectorTest : public testing::Test
{
};

typedef testing::Types<float, double, int> Implementations;

TYPED_TEST_CASE( BoundedVectorTest, Implementations );

TYPED_TEST( BoundedVectorTest, tests )
{
  using Scalar = TypeParam;
  const size_t maxSize = 25;
  BoundedVector<Scalar, maxSize> vector;
  EXPECT_EQ( vector.size(), 0 );
  EXPECT_TRUE( vector.empty() );
  EXPECT_FALSE( vector.full() );
  // add items 0,1,...,maxSize-1
  for ( size_t i = 0; i < maxSize; i++ ) {
    vector.push_back( Scalar( i ) );
    EXPECT_EQ( vector.size(), i + 1 );
  }
  EXPECT_TRUE( vector.full() );
  EXPECT_THROW( vector.push_back( 0 ), std::length_error );
  // delete last item
  vector.pop_back();
  EXPECT_EQ( vector.size(), maxSize - 1 );
  EXPECT_FALSE( vector.full() );
  // delete first item -> all other items must be moved -> 1,2,..maxSize-2
  vector.erase( vector.begin() );
  EXPECT_EQ( vector.size(), maxSize - 2 );
  for ( size_t i = 0; i < maxSize - 2; i++ ) {
    EXPECT_TRUE( vector[i] == static_cast<Scalar>( i + 1 ) );
  }
  // delete first 3 items -> all other items must be moved -> 4,5,..maxSize-5
  vector.erase( vector.begin(), vector.begin() + 3 ); // removes all elements in [first,last)
  EXPECT_EQ( vector.size(), maxSize - 2 - 3 );
  for ( size_t i = 0; i < maxSize - 6; i++ ) {
    EXPECT_TRUE( vector[i] == static_cast<Scalar>( i + 4 ) );
  }
  vector.clear();
  EXPECT_EQ( vector.size(), 0 );
  EXPECT_TRUE( vector.empty() );
  EXPECT_FALSE( vector.full() );
  // construct items 0,1,...,maxSize-1
  for ( size_t i = 0; i < maxSize; i++ ) {
    vector.emplace_back( Scalar( i ) );
    EXPECT_TRUE( vector.size() == i + 1 );
  }
  EXPECT_TRUE( vector.full() );
  EXPECT_THROW( vector.emplace_back( 0 ), std::length_error );
  EXPECT_EQ( vector.front(), 0 );
  EXPECT_EQ( vector.back(), maxSize - 1 );
  // Should not die as long as we stay within maxSize.
  vector.reserve( maxSize - 1 );
  vector.reserve( maxSize );
  EXPECT_DEATH( vector.reserve( maxSize + 1 ),
                "Bounded vector can not reserve more than max size!" );
}

TYPED_TEST( BoundedVectorTest, resize )
{
  using Scalar = TypeParam;
  const size_t maxSize = 25;
  BoundedVector<Scalar, maxSize> vector;
  for ( size_t i = 0; i < 10; i++ ) vector.push_back( Scalar( i ) );

  // Shrinking drops exactly the requested number of elements and keeps the remaining ones
  vector.resize( 6 );
  EXPECT_EQ( vector.size(), 6u );
  for ( size_t i = 0; i < 6; i++ ) EXPECT_EQ( vector[i], static_cast<Scalar>( i ) );

  // Growing value-initializes the added elements instead of exposing the previous values
  vector.resize( 9 );
  EXPECT_EQ( vector.size(), 9u );
  for ( size_t i = 0; i < 6; i++ ) EXPECT_EQ( vector[i], static_cast<Scalar>( i ) );
  for ( size_t i = 6; i < 9; i++ ) EXPECT_EQ( vector[i], Scalar( 0 ) );

  EXPECT_THROW( vector.resize( maxSize + 1 ), std::length_error );
  EXPECT_EQ( vector.size(), 9u );
}

// Elements are kept alive by the underlying array, hence removing them has to reset their value to
// release the resources they own.
TEST( BoundedVectorResourceTest, removalReleasesResources )
{
  auto resource = std::make_shared<int>( 42 );
  BoundedVector<std::shared_ptr<int>, 4> vector;
  vector.push_back( resource );
  vector.push_back( resource );
  vector.push_back( resource );
  EXPECT_EQ( resource.use_count(), 4 );

  vector.pop_back();
  EXPECT_EQ( resource.use_count(), 3 );

  vector.erase( vector.begin() );
  EXPECT_EQ( resource.use_count(), 2 );

  vector.clear();
  EXPECT_EQ( resource.use_count(), 1 );

  vector.push_back( resource );
  vector.push_back( resource );
  vector.erase( vector.begin(), vector.end() );
  EXPECT_EQ( resource.use_count(), 1 );

  vector.push_back( resource );
  vector.resize( 0 );
  EXPECT_EQ( resource.use_count(), 1 );
}

TYPED_TEST( BoundedVectorTest, slotReuse )
{
  using Scalar = TypeParam;
  BoundedVector<Scalar, 4> vector;
  vector.push_back( Scalar( 1 ) );
  vector.pop_back();
  vector.push_back( Scalar( 2 ) );
  EXPECT_EQ( vector.size(), 1u );
  EXPECT_EQ( vector.back(), Scalar( 2 ) );

  vector.push_back( Scalar( 3 ) );
  vector.clear();
  vector.emplace_back( Scalar( 4 ) );
  EXPECT_EQ( vector.size(), 1u );
  EXPECT_EQ( vector.front(), Scalar( 4 ) );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
