// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <hector_math/map_operations/fit_plane.h>
#include <hector_math/types.h>

#include <benchmark/benchmark.h>
#include <random>

using namespace hector_math;

static void fitPlane( benchmark::State &state )
{
  GridMap<float> map = GridMapf::Random( state.range(), state.range() );
  for ( auto _ : state ) {
    PlaneEstimationResult result = hector_math::fitPlaneXY( map );
    benchmark::DoNotOptimize( result );
  }
}

static void fitPlaneHalfUnknown( benchmark::State &state )
{
  GridMap<float> map = GridMapf::Random( state.range(), state.range() );
  std::default_random_engine generator( 42 );
  std::uniform_int_distribution<int> distribution( 0, 1 );
  for ( Eigen::Index row = 0; row < map.rows(); ++row ) {
    for ( Eigen::Index col = 0; col < map.cols(); ++col ) {
      if ( distribution( generator ) == 0 ) {
        map( row, col ) = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }
  for ( auto _ : state ) {
    PlaneEstimationResult result = hector_math::fitPlaneXY( map );
    benchmark::DoNotOptimize( result );
  }
}

BENCHMARK( fitPlane )->Arg( 100 )->Arg( 10000 )->Unit( benchmark::kMicrosecond );
BENCHMARK( fitPlaneHalfUnknown )->Arg( 100 )->Arg( 10000 )->Unit( benchmark::kMicrosecond );

BENCHMARK_MAIN();
