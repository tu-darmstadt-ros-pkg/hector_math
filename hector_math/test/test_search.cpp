#include <hector_math/math/search.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <random>
#include <vector>

using namespace hector_math;

TEST( SearchTest, containerOverloadsUseWholeRange )
{
  std::vector<int> odd_values{ 9, 1, 4, 7, 2 };
  EXPECT_EQ( findMedianLower( odd_values ), 4 );

  odd_values = { 9, 1, 4, 7, 2 };
  EXPECT_EQ( findMedianUpper( odd_values ), 4 );

  odd_values = { 9, 1, 4, 7, 2 };
  EXPECT_EQ( findMedian( odd_values ), 4 );

  std::array<int, 4> even_values{ 8, 1, 3, 5 };
  EXPECT_EQ( findMedianLower( even_values ), 3 );

  even_values = { 8, 1, 3, 5 };
  EXPECT_EQ( findMedianUpper( even_values ), 5 );

  even_values = { 8, 1, 3, 5 };
  EXPECT_EQ( findMedian( even_values ), 4 );
}

TEST( SearchTest, arrayOverloadsWork )
{
  int lower_values[] = { 10, 4, 2, 8 };
  EXPECT_EQ( findMedianLower( lower_values ), 4 );

  int upper_values[] = { 10, 4, 2, 8 };
  EXPECT_EQ( findMedianUpper( upper_values ), 8 );

  int median_values[] = { 10, 4, 2, 8 };
  EXPECT_EQ( findMedian( median_values ), 6 );
}

TEST( SearchTest, singleElement )
{
  std::vector<int> v{ 42 };
  EXPECT_EQ( findMedianLower( v ), 42 );
  v = { 42 };
  EXPECT_EQ( findMedianUpper( v ), 42 );
  v = { 42 };
  EXPECT_EQ( findMedian( v ), 42 );
}

TEST( SearchTest, twoElements )
{
  std::vector<int> v{ 7, 3 };
  EXPECT_EQ( findMedianLower( v ), 3 );
  v = { 7, 3 };
  EXPECT_EQ( findMedianUpper( v ), 7 );
  v = { 7, 3 };
  EXPECT_EQ( findMedian( v ), 5 );

  v = { 3, 7 };
  EXPECT_EQ( findMedianLower( v ), 3 );
  v = { 3, 7 };
  EXPECT_EQ( findMedianUpper( v ), 7 );
  v = { 3, 7 };
  EXPECT_EQ( findMedian( v ), 5 );
}

TEST( SearchTest, allEqual )
{
  std::vector<int> odd{ 5, 5, 5, 5, 5 };
  EXPECT_EQ( findMedianLower( odd ), 5 );
  odd = { 5, 5, 5, 5, 5 };
  EXPECT_EQ( findMedianUpper( odd ), 5 );
  odd = { 5, 5, 5, 5, 5 };
  EXPECT_EQ( findMedian( odd ), 5 );

  std::vector<int> even{ 2, 2, 2, 2 };
  EXPECT_EQ( findMedianLower( even ), 2 );
  even = { 2, 2, 2, 2 };
  EXPECT_EQ( findMedianUpper( even ), 2 );
  even = { 2, 2, 2, 2 };
  EXPECT_EQ( findMedian( even ), 2 );
}

TEST( SearchTest, sortedAscending )
{
  std::vector<int> odd{ 1, 2, 3, 4, 5, 6, 7 };
  EXPECT_EQ( findMedianLower( odd ), 4 );
  odd = { 1, 2, 3, 4, 5, 6, 7 };
  EXPECT_EQ( findMedianUpper( odd ), 4 );
  odd = { 1, 2, 3, 4, 5, 6, 7 };
  EXPECT_EQ( findMedian( odd ), 4 );

  std::vector<int> even{ 1, 2, 3, 4, 5, 6 };
  EXPECT_EQ( findMedianLower( even ), 3 );
  even = { 1, 2, 3, 4, 5, 6 };
  EXPECT_EQ( findMedianUpper( even ), 4 );
  even = { 1, 2, 3, 4, 5, 6 };
  EXPECT_EQ( findMedian( even ), 3 ); // (3+4)/2 truncates for int
}

TEST( SearchTest, sortedDescending )
{
  std::vector<int> odd{ 7, 6, 5, 4, 3, 2, 1 };
  EXPECT_EQ( findMedianLower( odd ), 4 );
  odd = { 7, 6, 5, 4, 3, 2, 1 };
  EXPECT_EQ( findMedianUpper( odd ), 4 );
  odd = { 7, 6, 5, 4, 3, 2, 1 };
  EXPECT_EQ( findMedian( odd ), 4 );

  std::vector<int> even{ 6, 5, 4, 3, 2, 1 };
  EXPECT_EQ( findMedianLower( even ), 3 );
  even = { 6, 5, 4, 3, 2, 1 };
  EXPECT_EQ( findMedianUpper( even ), 4 );
  even = { 6, 5, 4, 3, 2, 1 };
  EXPECT_EQ( findMedian( even ), 3 );
}

TEST( SearchTest, withDuplicatesAroundMedian )
{
  std::vector<int> odd{ 1, 2, 2, 2, 3 };
  EXPECT_EQ( findMedianLower( odd ), 2 );
  odd = { 1, 2, 2, 2, 3 };
  EXPECT_EQ( findMedianUpper( odd ), 2 );
  odd = { 1, 2, 2, 2, 3 };
  EXPECT_EQ( findMedian( odd ), 2 );

  std::vector<int> even{ 1, 2, 2, 3, 3, 4 };
  EXPECT_EQ( findMedianLower( even ), 2 );
  even = { 1, 2, 2, 3, 3, 4 };
  EXPECT_EQ( findMedianUpper( even ), 3 );
  even = { 1, 2, 2, 3, 3, 4 };
  EXPECT_EQ( findMedian( even ), 2 ); // (2+3)/2 truncates for int
}

TEST( SearchTest, negativeValues )
{
  std::vector<int> odd{ -5, -1, -9, -3, -7 };
  EXPECT_EQ( findMedianLower( odd ), -5 );
  odd = { -5, -1, -9, -3, -7 };
  EXPECT_EQ( findMedianUpper( odd ), -5 );
  odd = { -5, -1, -9, -3, -7 };
  EXPECT_EQ( findMedian( odd ), -5 );

  std::vector<int> even{ -4, -2, -8, -6 };
  EXPECT_EQ( findMedianLower( even ), -6 );
  even = { -4, -2, -8, -6 };
  EXPECT_EQ( findMedianUpper( even ), -4 );
  even = { -4, -2, -8, -6 };
  EXPECT_EQ( findMedian( even ), -5 );
}

TEST( SearchTest, integerDivisionTruncates )
{
  std::vector<int> pos{ 1, 2 };
  EXPECT_EQ( findMedianLower( pos ), 1 );
  pos = { 1, 2 };
  EXPECT_EQ( findMedianUpper( pos ), 2 );
  pos = { 1, 2 };
  EXPECT_EQ( findMedian( pos ), 1 ); // (1+2)/2 = 1 (truncated)

  std::vector<int> neg{ -1, -2 };
  EXPECT_EQ( findMedianLower( neg ), -2 );
  neg = { -1, -2 };
  EXPECT_EQ( findMedianUpper( neg ), -1 );
  neg = { -1, -2 };
  EXPECT_EQ( findMedian( neg ), -1 ); // (-2 + -1)/2 = -3/2 = -1 (truncates toward zero)
}

TEST( SearchTest, doubleType )
{
  std::vector<double> even{ 1.0, 2.0, 3.0, 4.0 };
  EXPECT_DOUBLE_EQ( findMedianLower( even ), 2.0 );
  even = { 1.0, 2.0, 3.0, 4.0 };
  EXPECT_DOUBLE_EQ( findMedianUpper( even ), 3.0 );
  even = { 1.0, 2.0, 3.0, 4.0 };
  EXPECT_DOUBLE_EQ( findMedian( even ), 2.5 );

  std::vector<double> odd{ 1.0, 2.0, 3.0, 4.0, 5.0 };
  EXPECT_DOUBLE_EQ( findMedianLower( odd ), 3.0 );
  odd = { 1.0, 2.0, 3.0, 4.0, 5.0 };
  EXPECT_DOUBLE_EQ( findMedianUpper( odd ), 3.0 );
  odd = { 1.0, 2.0, 3.0, 4.0, 5.0 };
  EXPECT_DOUBLE_EQ( findMedian( odd ), 3.0 );
}

TEST( SearchTest, floatType )
{
  std::vector<float> v{ 1.5f, 2.5f, 3.5f, 4.5f };
  EXPECT_FLOAT_EQ( findMedianLower( v ), 2.5f );
  v = { 1.5f, 2.5f, 3.5f, 4.5f };
  EXPECT_FLOAT_EQ( findMedianUpper( v ), 3.5f );
  v = { 1.5f, 2.5f, 3.5f, 4.5f };
  EXPECT_FLOAT_EQ( findMedian( v ), 3.0f );
}

TEST( SearchTest, largeRandomMatchesSort )
{
  for ( size_t n : { size_t{ 1001 }, size_t{ 1000 } } ) {
    std::mt19937 rng( 42 );
    std::uniform_int_distribution<int> dist( -10000, 10000 );
    std::vector<int> data( n );
    for ( int &x : data ) x = dist( rng );

    std::vector<int> sorted = data;
    std::sort( sorted.begin(), sorted.end() );

    std::vector<int> a = data;
    EXPECT_EQ( findMedianLower( a ), sorted[( n - 1 ) / 2] );

    std::vector<int> b = data;
    EXPECT_EQ( findMedianUpper( b ), sorted[n / 2] );

    std::vector<int> c = data;
    int expected_median = ( n % 2 == 1 ) ? sorted[n / 2] : ( sorted[n / 2 - 1] + sorted[n / 2] ) / 2;
    EXPECT_EQ( findMedian( c ), expected_median );
  }
}

TEST( SearchTest, iteratorSubrangeOverload )
{
  // Surrounding 9s would dominate full-range medians; subrange picks {1,4,7,2}.
  std::vector<int> v{ 9, 9, 1, 4, 7, 2, 9, 9 };
  auto sub_begin = v.begin() + 2;
  auto sub_end = v.end() - 2;
  EXPECT_EQ( findMedianLower( sub_begin, sub_end ), 2 );

  v = { 9, 9, 1, 4, 7, 2, 9, 9 };
  sub_begin = v.begin() + 2;
  sub_end = v.end() - 2;
  EXPECT_EQ( findMedianUpper( sub_begin, sub_end ), 4 );

  v = { 9, 9, 1, 4, 7, 2, 9, 9 };
  sub_begin = v.begin() + 2;
  sub_end = v.end() - 2;
  EXPECT_EQ( findMedian( sub_begin, sub_end ), 3 );
}

TEST( SearchTest, inputIsReorderedNotCopied )
{
  const std::vector<int> original{ 9, 1, 4, 7, 2, 6, 3 }; // n=7, sorted middle index 3 -> 4
  std::vector<int> a = original;
  int lower = findMedianLower( a );
  EXPECT_EQ( lower, 4 );
  EXPECT_TRUE( std::is_permutation( a.begin(), a.end(), original.begin() ) );
  EXPECT_EQ( a[( a.size() - 1 ) / 2], lower );

  std::vector<int> b = original;
  int upper = findMedianUpper( b );
  EXPECT_EQ( upper, 4 );
  EXPECT_TRUE( std::is_permutation( b.begin(), b.end(), original.begin() ) );
  EXPECT_EQ( b[b.size() / 2], upper );

  // Even-sized check that input multiset is preserved by findMedian.
  const std::vector<int> original_even{ 8, 1, 3, 5 };
  std::vector<int> c = original_even;
  EXPECT_EQ( findMedian( c ), 4 );
  EXPECT_TRUE( std::is_permutation( c.begin(), c.end(), original_even.begin() ) );
}

TEST( SearchTest, rawArrayOddSize )
{
  int lower_values[] = { 9, 1, 4, 7, 2 };
  EXPECT_EQ( findMedianLower( lower_values ), 4 );

  int upper_values[] = { 9, 1, 4, 7, 2 };
  EXPECT_EQ( findMedianUpper( upper_values ), 4 );

  int median_values[] = { 9, 1, 4, 7, 2 };
  EXPECT_EQ( findMedian( median_values ), 4 );
}
