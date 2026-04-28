// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_SEARCH_H
#define HECTOR_MATH_SEARCH_H

#include <algorithm>
#include <cassert>
#include <iterator>

namespace hector_math
{

/*!
 * Returns the lower median of [begin, end). For an even-sized range this is the
 * smaller of the two middle elements; for an odd-sized range it is the true median.
 * Reorders the input via std::nth_element. Average O(n).
 */
template<typename RandomIt>
typename std::iterator_traits<RandomIt>::value_type findMedianLower( RandomIt begin, RandomIt end )
{
  const auto count = end - begin;
  assert( count > 0 && "findMedianLower requires at least one element" );
  RandomIt mid = begin + ( count - 1 ) / 2;
  std::nth_element( begin, mid, end );
  return *mid;
}

template<typename Container>
auto findMedianLower( Container &container )
{
  return findMedianLower( std::begin( container ), std::end( container ) );
}

/*!
 * Returns the upper median of [begin, end). For an even-sized range this is the
 * larger of the two middle elements; for an odd-sized range it is the true median.
 * Reorders the input via std::nth_element. Average O(n).
 */
template<typename RandomIt>
typename std::iterator_traits<RandomIt>::value_type findMedianUpper( RandomIt begin, RandomIt end )
{
  const auto count = end - begin;
  assert( count > 0 && "findMedianUpper requires at least one element" );
  RandomIt mid = begin + count / 2;
  std::nth_element( begin, mid, end );
  return *mid;
}

template<typename Container>
auto findMedianUpper( Container &container )
{
  return findMedianUpper( std::begin( container ), std::end( container ) );
}

/*!
 * Returns the median of [begin, end). For an even-sized range the average of the
 * lower and upper medians is returned; for an odd-sized range the true median.
 * Uses one std::nth_element call followed by std::max_element on the partitioned
 * left half — average O(n).
 */
template<typename RandomIt>
typename std::iterator_traits<RandomIt>::value_type findMedian( RandomIt begin, RandomIt end )
{
  using Value = typename std::iterator_traits<RandomIt>::value_type;
  const auto count = end - begin;
  assert( count > 0 && "findMedian requires at least one element" );
  RandomIt upper_middle = begin + count / 2;
  std::nth_element( begin, upper_middle, end );
  const Value upper_value = *upper_middle;
  if ( ( count & 1 ) == 1 ) {
    return upper_value;
  }
  const Value lower_value = *std::max_element( begin, upper_middle );
  return ( lower_value + upper_value ) / static_cast<Value>( 2 );
}

template<typename Container>
auto findMedian( Container &container )
{
  return findMedian( std::begin( container ), std::end( container ) );
}

} // namespace hector_math

#endif // HECTOR_MATH_SEARCH_H
