// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <hector_math/math/search.h>

#include <algorithm>
#include <benchmark/benchmark.h>
#include <random>
#include <vector>

namespace
{

// Holds kPoolSize independent random vectors plus a working copy of each. Each call
// to take() returns one mutable working vector; once all working copies have been
// consumed the pool refreshes them from the pristine source under PauseTiming so the
// copy cost stays out of the measurement.
class VectorPool
{
public:
  static constexpr size_t kPoolSize = 32;

  explicit VectorPool( size_t n )
  {
    std::default_random_engine generator( 42 );
    std::uniform_real_distribution<float> distribution( -1000.0f, 1000.0f );
    pristines_.resize( kPoolSize );
    working_.resize( kPoolSize );
    for ( size_t i = 0; i < kPoolSize; ++i ) {
      pristines_[i].resize( n );
      working_[i].resize( n );
      for ( float &v : pristines_[i] ) v = distribution( generator );
    }
    refresh();
  }

  std::vector<float> &take( benchmark::State &state )
  {
    if ( idx_ == kPoolSize ) {
      state.PauseTiming();
      refresh();
      state.ResumeTiming();
    }
    return working_[idx_++];
  }

private:
  void refresh()
  {
    for ( size_t i = 0; i < kPoolSize; ++i ) {
      std::copy( pristines_[i].begin(), pristines_[i].end(), working_[i].begin() );
    }
    idx_ = 0;
  }

  std::vector<std::vector<float>> pristines_;
  std::vector<std::vector<float>> working_;
  size_t idx_ = 0;
};

void findMedian( benchmark::State &state )
{
  VectorPool pool( state.range( 0 ) );
  for ( auto _ : state ) {
    auto &data = pool.take( state );
    auto m = hector_math::findMedian( data.begin(), data.end() );
    benchmark::DoNotOptimize( m );
  }
}

// Naive baseline for comparison: two independent nth_element calls instead of the
// nth_element + max_element trick used by findMedian.
void findMedianTwoCalls( benchmark::State &state )
{
  VectorPool pool( state.range( 0 ) );
  for ( auto _ : state ) {
    auto &data = pool.take( state );
    const auto n = data.end() - data.begin();
    auto upper = hector_math::findMedianUpper( data.begin(), data.end() );
    if ( ( n & 1 ) == 0 ) {
      auto lower = hector_math::findMedianLower( data.begin(), data.end() );
      upper = ( upper + lower ) / 2.0f;
    }
    benchmark::DoNotOptimize( upper );
  }
}

void findMedianUpper( benchmark::State &state )
{
  VectorPool pool( state.range( 0 ) );
  for ( auto _ : state ) {
    auto &data = pool.take( state );
    auto m = hector_math::findMedianUpper( data.begin(), data.end() );
    benchmark::DoNotOptimize( m );
  }
}

void findMedianLower( benchmark::State &state )
{
  VectorPool pool( state.range( 0 ) );
  for ( auto _ : state ) {
    auto &data = pool.take( state );
    auto m = hector_math::findMedianLower( data.begin(), data.end() );
    benchmark::DoNotOptimize( m );
  }
}

void stdNthElement( benchmark::State &state )
{
  VectorPool pool( state.range( 0 ) );
  for ( auto _ : state ) {
    auto &data = pool.take( state );
    auto mid = data.begin() + data.size() / 2;
    std::nth_element( data.begin(), mid, data.end() );
    benchmark::DoNotOptimize( *mid );
  }
}

void stdSort( benchmark::State &state )
{
  VectorPool pool( state.range( 0 ) );
  for ( auto _ : state ) {
    auto &data = pool.take( state );
    std::sort( data.begin(), data.end() );
    benchmark::DoNotOptimize( data[data.size() / 2] );
  }
}

} // namespace

BENCHMARK( findMedian )->Arg( 100 )->Arg( 1000 )->Arg( 100000 )->Arg( 1000000 )->Unit( benchmark::kMicrosecond );
BENCHMARK( findMedianTwoCalls )
    ->Arg( 100 )
    ->Arg( 1000 )
    ->Arg( 100000 )
    ->Arg( 1000000 )
    ->Unit( benchmark::kMicrosecond );
BENCHMARK( findMedianUpper )
    ->Arg( 100 )
    ->Arg( 1000 )
    ->Arg( 100000 )
    ->Arg( 1000000 )
    ->Unit( benchmark::kMicrosecond );
BENCHMARK( findMedianLower )
    ->Arg( 100 )
    ->Arg( 1000 )
    ->Arg( 100000 )
    ->Arg( 1000000 )
    ->Unit( benchmark::kMicrosecond );
BENCHMARK( stdNthElement )
    ->Arg( 100 )
    ->Arg( 1000 )
    ->Arg( 100000 )
    ->Arg( 1000000 )
    ->Unit( benchmark::kMicrosecond );
BENCHMARK( stdSort )->Arg( 100 )->Arg( 1000 )->Arg( 100000 )->Arg( 1000000 )->Unit(
    benchmark::kMicrosecond );

BENCHMARK_MAIN();
