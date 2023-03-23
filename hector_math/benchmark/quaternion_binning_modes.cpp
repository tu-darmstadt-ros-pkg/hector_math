// Copyright (c) 2022 Aljoscha Schmidt. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
#include "hector_math/containers/quaternion_cache.h"

#include <benchmark/benchmark.h>
#include <random>
#include <ros/package.h>

using namespace hector_math;

template<typename Scalar, QuaternionBinningMode mode, int AXIS_BINS = 128, int ANGLE_COUNT = 512>
static void quaternionBinning( benchmark::State &state )
{
  for ( auto _ : state ) {
    auto index =
        computeBin<Scalar, AXIS_BINS, ANGLE_COUNT, mode>( Eigen::Quaternion<Scalar>::UnitRandom() );
    benchmark::DoNotOptimize( index );
  }
}

// BENCHMARK_TEMPLATE( quaternionBinning,float, QuaternionBinningMode::LargestDim, 128,512)->Unit( benchmark::kNanosecond )->Iterations( 2000000 );
// BENCHMARK_TEMPLATE( quaternionBinning,double, QuaternionBinningMode::LargestDim, 128,512)->Unit( benchmark::kNanosecond )->Iterations( 2000000 );

BENCHMARK_TEMPLATE( quaternionBinning, float, QuaternionBinningMode::SphericalFibonacci, 128, 512 )
    ->Unit( benchmark::kNanosecond )
    ->Iterations( 2000000 );
BENCHMARK_TEMPLATE( quaternionBinning, double, QuaternionBinningMode::SphericalFibonacci, 128, 512 )
    ->Unit( benchmark::kNanosecond )
    ->Iterations( 2000000 );

BENCHMARK_MAIN();
