#include "hector_math/containers/ring_buffer.h"

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "hector_math/types.h"
#include "eigen_tests.h"


using namespace hector_math;
template<typename Scalar>
class RingBufferTest : public testing::Test
{
};


typedef testing::Types<float, double> Implementations;

TYPED_TEST_CASE( RingBufferTest, Implementations );

TYPED_TEST( RingBufferTest, basic ) {
  using Scalar = TypeParam;
  constexpr size_t max_size = 50;
  RingBuffer<Scalar,max_size> ringBuffer;
  ASSERT_FALSE(ringBuffer.full());
  ASSERT_TRUE(ringBuffer.empty());
  ringBuffer.push_back(Scalar(0));
  ASSERT_FALSE(ringBuffer.empty());
  ASSERT_EQ(ringBuffer.size(),1);
  for(size_t i=1;i<max_size-1;i++){
    ringBuffer.push_back(i);
    ASSERT_EQ(i+1,ringBuffer.size());
    ASSERT_FALSE(ringBuffer.full());
    ASSERT_FALSE(ringBuffer.empty());
  }
  ringBuffer.push_back(49);
  ASSERT_FALSE(ringBuffer.empty());
  ASSERT_TRUE(ringBuffer.full());

  ASSERT_EQ(ringBuffer.read_and_pop_front(),0);
  ASSERT_FALSE(ringBuffer.empty());
  ASSERT_FALSE(ringBuffer.full());
  ASSERT_EQ(1,ringBuffer.read_and_pop_front());
  ringBuffer.push_back(50);
  ASSERT_EQ(ringBuffer.size(),49);
  // CLEAR
  ringBuffer.clear();
  ASSERT_TRUE(ringBuffer.empty());
  ASSERT_FALSE(ringBuffer.full());
  ASSERT_EQ(ringBuffer.size(),0);
}

TYPED_TEST(RingBufferTest, iterators){
  using Scalar = TypeParam;
  constexpr size_t max_size = 50;
  RingBuffer<Scalar,max_size> ringBuffer;
  // FILL RING BUFFER PARTIAL
  for(int i=0; i<25;i++){
    ringBuffer.push_back(i);
  }
  ASSERT_EQ(ringBuffer.size(),25);
  int index=0;
  for(auto elm:ringBuffer){
    ASSERT_EQ(index++,elm);

  }
  ASSERT_EQ(index, ringBuffer.size());
  index = 0;
  for(auto const_it=ringBuffer.cbegin();const_it!=ringBuffer.cend();const_it++){
    ASSERT_EQ(*const_it,index++);
  }
  //FILL AGAIN AND OVERWRITE FIRST ELEMENTS
  for(int i=0; i<max_size;i++){
    ringBuffer.push_back(i);
  }
  ASSERT_EQ(ringBuffer.size(),max_size);
  ASSERT_TRUE(ringBuffer.full());
  index=0;
  for(auto elm:ringBuffer){
    ASSERT_EQ(index++,elm);
  }
  ASSERT_EQ(index, ringBuffer.size());

  //algorithms
  // FOR_EACH
  auto replace_with_3 = [](Scalar& n) { n=3; };
  std::for_each(ringBuffer.begin(),ringBuffer.end(),replace_with_3);
  for(auto elm:ringBuffer){
    ASSERT_EQ(3,elm);
  }
  //FILL AGAIN AND OVERWRITE FIRST ELEMENTS
  for(int i=0; i<max_size;i++){
    ringBuffer.push_back(i);
  }
  // FIND
  auto x = std::find(ringBuffer.begin(),ringBuffer.end(),7);
  ASSERT_EQ(*x,7);
}


TYPED_TEST( RingBufferTest, front_back )
{
  using Scalar = TypeParam;
  constexpr size_t max_size = 50;
  RingBuffer<Scalar, max_size > ringBuffer;
  // FILL RING BUFFER PARTIAL
  for(int i=0; i<25;i++){
    ringBuffer.push_back(i);
  }
  ASSERT_EQ(ringBuffer.front(), 0);
  ASSERT_EQ(ringBuffer.back(), 24);
}

TYPED_TEST( RingBufferTest, read_front )
{
  using Scalar = TypeParam;
  constexpr size_t max_size = 50;
  RingBuffer<Scalar, max_size> ringBuffer;
  // FILL RING BUFFER PARTIAL
  for ( int i = 0; i < 25; i++ ) { ringBuffer.push_back( i ); }
  // READ ELEMENTS
  for ( int i = 0; i < 25; i++ ) { ASSERT_EQ(i,ringBuffer.read_and_pop_front()); }
  ASSERT_TRUE(ringBuffer.empty());
  // FILL RING BUFFER such that neither head nor tail at zero
  int overwrite_digits = 12;
  for ( int i = 0; i < max_size+overwrite_digits; i++ ) { ringBuffer.push_back( i ); }
  ASSERT_TRUE(ringBuffer.full());
  // pop first element and check that ringBuffer.front() moves correctly
  ASSERT_EQ(ringBuffer.front(),overwrite_digits); // first num that is not overwritten
  for ( int i = 0; i < max_size; i++ ) {
    ASSERT_EQ(ringBuffer.front(), overwrite_digits+i);
    ASSERT_EQ(ringBuffer.back(), overwrite_digits+max_size-1); //does not change
    ringBuffer.pop_front();
  }
  // after removing max_size elements the buffer must be empty
  ASSERT_TRUE(ringBuffer.empty());
  ASSERT_EQ(ringBuffer.size(),0);
  // is empty -> can not retrieve image
  ASSERT_ANY_THROW(ringBuffer.read_and_pop_front());
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
