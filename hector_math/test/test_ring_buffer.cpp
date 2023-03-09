#include "hector_math/containers/ring_buffer.h"

#include "eigen_tests.h"
#include "hector_math/types.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

using namespace hector_math;
template<typename Scalar>
class RingBufferTest : public testing::Test
{
};

typedef testing::Types<float, double> Implementations;

bool is_distance_correct(size_t returned_distance, size_t correct_distance, int num_filled_distance){
    return returned_distance==correct_distance || num_filled_distance-returned_distance == correct_distance;
}

TYPED_TEST_CASE( RingBufferTest, Implementations );

TYPED_TEST( RingBufferTest, basic )
{
  using Scalar = TypeParam;
  constexpr size_t max_size = 50;
  RingBuffer<Scalar, max_size> ringBuffer;
  ASSERT_FALSE( ringBuffer.full() );
  ASSERT_TRUE( ringBuffer.empty() );
  ringBuffer.push_back( Scalar( 0 ) );
  ASSERT_FALSE( ringBuffer.empty() );
  ASSERT_EQ( ringBuffer.size(), 1 );
  for ( size_t i = 1; i < max_size - 1; i++ ) {
    ringBuffer.push_back( i );
    ASSERT_EQ( i + 1, ringBuffer.size() );
    ASSERT_FALSE( ringBuffer.full() );
    ASSERT_FALSE( ringBuffer.empty() );
  }
  ringBuffer.push_back( 49 );
  ASSERT_FALSE( ringBuffer.empty() );
  ASSERT_TRUE( ringBuffer.full() );

  ASSERT_EQ( ringBuffer.read_and_pop_front(), 0 );
  ASSERT_FALSE( ringBuffer.empty() );
  ASSERT_FALSE( ringBuffer.full() );
  ASSERT_EQ( 1, ringBuffer.read_and_pop_front() );
  ringBuffer.push_back( 50 );
  ASSERT_EQ( ringBuffer.size(), 49 );
  // CLEAR
  ringBuffer.clear();
  ASSERT_TRUE( ringBuffer.empty() );
  ASSERT_FALSE( ringBuffer.full() );
  ASSERT_EQ( ringBuffer.size(), 0 );
}

TYPED_TEST( RingBufferTest, iterators )
{
  using Scalar = TypeParam;
  constexpr size_t max_size = 50;
  RingBuffer<Scalar, max_size> ringBuffer;
  // FILL RING BUFFER PARTIAL
  for ( int i = 0; i < 25; i++ ) { ringBuffer.push_back( i ); }
  ASSERT_EQ( ringBuffer.size(), 25 );
  int index = 0;
  for ( auto elm : ringBuffer ) {
    ASSERT_EQ( index++, elm );
  }
  ASSERT_EQ( index, ringBuffer.size() );
  index = 0;
  for ( auto const_it = ringBuffer.cbegin(); const_it != ringBuffer.cend(); const_it++ ) {
    ASSERT_EQ( *const_it, index++ );
  }
  // FILL AGAIN AND OVERWRITE FIRST ELEMENTS
  for ( int i = 0; i < max_size; i++ ) { ringBuffer.push_back( i ); }
  ASSERT_EQ( ringBuffer.size(), max_size );
  ASSERT_TRUE( ringBuffer.full() );
  index = 0;
  for ( auto elm : ringBuffer ) { ASSERT_EQ( index++, elm ); }
  ASSERT_EQ( index, ringBuffer.size() );

  // algorithms
  //  FOR_EACH
  auto replace_with_3 = []( Scalar &n ) { n = 3; };
  std::for_each( ringBuffer.begin(), ringBuffer.end(), replace_with_3 );
  for ( auto elm : ringBuffer ) { ASSERT_EQ( 3, elm ); }
  // FILL AGAIN AND OVERWRITE FIRST ELEMENTS
  for ( int i = 0; i < max_size; i++ ) { ringBuffer.push_back( i ); }
  // FIND
  auto x = std::find( ringBuffer.begin(), ringBuffer.end(), 7 );
  ASSERT_EQ( *x, 7 );

  // Previous failure case: back failing when filling exactly to head_index_ = 0
  ringBuffer.clear();
  for ( int i = 0; i < max_size; i++ ) ringBuffer.push_back( i );
  ASSERT_EQ( ringBuffer.front(), 0 );
  ASSERT_EQ( ringBuffer.back(), max_size - 1 );
}

TYPED_TEST( RingBufferTest, iterators_random_access )
{
  using Scalar = TypeParam;
  constexpr size_t max_size = 50;
  constexpr size_t partial_filled = 25;
  for(int start_pos=0;start_pos<max_size;start_pos++) {
    std::cout<<"Start pos "<<start_pos<<std::endl;
    RingBuffer<Scalar, max_size> ringBuffer;
    for(int i=0;i<start_pos;i++){ringBuffer.push_back(-1);}
    for(int i=0;i<start_pos;i++){ringBuffer.pop_front();}
    ASSERT_EQ(&ringBuffer.front(),&ringBuffer[start_pos]);
    // FILL RING BUFFER PARTIAL
    for ( int i = 0; i < partial_filled; i++ ) { ringBuffer.push_back( i ); }

    // TEST ++
    auto begin = ringBuffer.begin();
    for ( int i = 0; i < partial_filled; i++ ) {
      ASSERT_EQ( *begin, i );
      begin++;
    }
    ASSERT_EQ( *begin, 0 );//back at front position
    begin++;
    ASSERT_EQ( *begin, 1 );

    // ++ TEST
    begin = ringBuffer.begin();
    for ( int i = 0; i < partial_filled; i++ ) {
      ASSERT_EQ( *begin, i );
      ++begin;
    }
    ASSERT_EQ( *begin, 0 );//back at front position
    ++begin;
    ASSERT_EQ( *begin, 1 );

    // TEST --
    auto iterator = ringBuffer.begin();
    iterator--; //go to last valid
    for ( int i = partial_filled - 1; i >= 0; i-- ) {
      ASSERT_EQ( *iterator, i );
      iterator--;
    }
    ASSERT_EQ( *iterator, partial_filled - 1 );//back at last position
    iterator--;
    ASSERT_EQ( *iterator, partial_filled - 2 );

    // -- TEST
    iterator = ringBuffer.begin();
    --iterator; //go to last valid
    for ( int i = partial_filled - 1; i >= 0; i-- ) {
      ASSERT_EQ( *iterator, i );
      --iterator;
    }
    ASSERT_EQ( *iterator, partial_filled - 1 );//back at last position
    --iterator;
    ASSERT_EQ( *iterator, partial_filled - 2 );

    // TEST + x
    for ( int i = 0; i < 2*partial_filled; i++ ) {
      begin = ringBuffer.begin();
      begin = begin + i;
      ASSERT_EQ(*begin,i%partial_filled );
    }
    for ( int i = 0; i < 2*partial_filled; i++ ) {
      begin = ringBuffer.begin();
      begin++;
      begin++;
      begin = begin + i;
      ASSERT_EQ(*begin,(i+2)%partial_filled );
    }

    // TEST - x
    for ( int i = 2*partial_filled; i >=0; i-- ) {
      begin = ringBuffer.begin();
      begin--;
      begin = begin - i;
      ASSERT_EQ(*begin,partial_filled-1-(i%partial_filled) );
    }

    // TEST - between two pointers
    for ( int i = 0; i < partial_filled; i++ ) {
      begin = ringBuffer.begin();
      begin = begin + 5;
      iterator = ringBuffer.begin();
      iterator = iterator + (i + 5);
      ASSERT_TRUE(is_distance_correct(iterator-begin, i,partial_filled));
    }
  }

}

TYPED_TEST( RingBufferTest, front_back )
{
  using Scalar = TypeParam;
  constexpr size_t max_size = 50;
  RingBuffer<Scalar, max_size> ringBuffer;
  // FILL RING BUFFER PARTIAL
  for ( int i = 0; i < 25; i++ ) { ringBuffer.push_back( i ); }
  ASSERT_EQ( ringBuffer.front(), 0 );
  ASSERT_EQ( ringBuffer.back(), 24 );
}

TYPED_TEST( RingBufferTest, read_front )
{
  using Scalar = TypeParam;
  constexpr size_t max_size = 50;
  RingBuffer<Scalar, max_size> ringBuffer;
  // FILL RING BUFFER PARTIAL
  for ( int i = 0; i < 25; i++ ) { ringBuffer.push_back( i ); }
  // READ ELEMENTS
  for ( int i = 0; i < 25; i++ ) { ASSERT_EQ( i, ringBuffer.read_and_pop_front() ); }
  ASSERT_TRUE( ringBuffer.empty() );
  // FILL RING BUFFER such that neither head nor tail at zero
  int overwrite_digits = 12;
  for ( int i = 0; i < max_size + overwrite_digits; i++ ) { ringBuffer.push_back( i ); }
  ASSERT_TRUE( ringBuffer.full() );
  // pop first element and check that ringBuffer.front() moves correctly
  ASSERT_EQ( ringBuffer.front(), overwrite_digits ); // first num that is not overwritten
  for ( int i = 0; i < max_size; i++ ) {
    ASSERT_EQ( ringBuffer.front(), overwrite_digits + i );
    ASSERT_EQ( ringBuffer.back(), overwrite_digits + max_size - 1 ); // does not change
    ringBuffer.pop_front();
  }
  // after removing max_size elements the buffer must be empty
  ASSERT_TRUE( ringBuffer.empty() );
  ASSERT_EQ( ringBuffer.size(), 0 );
  // is empty -> can not retrieve image
  ASSERT_ANY_THROW( ringBuffer.read_and_pop_front() );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
