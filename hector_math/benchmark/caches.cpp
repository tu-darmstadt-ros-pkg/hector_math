//
// Created by stefan on 06.12.21.
//
#include "hector_math/containers/quaternion_cache.h"

#include <benchmark/benchmark.h>
#include <random>
#include <ros/package.h>

using namespace hector_math;


template<typename Scalar,QuaternionBinningMode mode>
static void quaternionCacheInsert( benchmark::State &state )
{
  QuaternionCache<Scalar, int, mode> cache;
  std::vector<Eigen::Quaternion<Scalar>, Eigen::aligned_allocator<Eigen::Quaternion<Scalar>>> quaternions;
  quaternions.resize(state.max_iterations);
  for (size_t i = 0; i < state.max_iterations; ++i) quaternions[i] = Eigen::Quaternion<Scalar>::UnitRandom();
  size_t i = 0;
  for ( auto _ : state ) {
    cache.insert(quaternions[i], i);
  }
}

template<typename Scalar, QuaternionBinningMode mode>
static void quaternionCacheFind( benchmark::State &state )
{
  QuaternionCache<Scalar, int, mode> cache;
  std::vector<Eigen::Quaternion<Scalar>, Eigen::aligned_allocator<Eigen::Quaternion<Scalar>>> quaternions;
  quaternions.resize(state.max_iterations);
  for (size_t i = 0; i < state.max_iterations; ++i) {
    quaternions[i] = Eigen::Quaternion<Scalar>::UnitRandom();
    cache.insert(quaternions[i], i);
  }
  size_t i = 0;
  for ( auto _ : state ) {
    if (cache.find(quaternions[i])->second != i)
      throw std::runtime_error("Unexpected");
  }
}
BENCHMARK_TEMPLATE( quaternionCacheInsert, float, QuaternionBinningMode::LargestDim)->Unit( benchmark::kNanosecond )->Iterations( 2000000 );
BENCHMARK_TEMPLATE( quaternionCacheInsert, float, QuaternionBinningMode::Spherical)->Unit( benchmark::kNanosecond )->Iterations( 2000000 );
BENCHMARK_TEMPLATE( quaternionCacheInsert, float, QuaternionBinningMode::SphericalFibonacci)->Unit( benchmark::kNanosecond )->Iterations( 2000000 );
BENCHMARK_TEMPLATE( quaternionCacheFind, float, QuaternionBinningMode::LargestDim )->Unit( benchmark::kNanosecond )->Iterations( 2000000 );
BENCHMARK_TEMPLATE( quaternionCacheFind, float, QuaternionBinningMode::Spherical )->Unit( benchmark::kNanosecond )->Iterations( 2000000 );
BENCHMARK_TEMPLATE( quaternionCacheFind, float, QuaternionBinningMode::SphericalFibonacci )->Unit( benchmark::kNanosecond )->Iterations( 2000000 );

BENCHMARK_TEMPLATE( quaternionCacheInsert, double, QuaternionBinningMode::LargestDim )->Unit( benchmark::kNanosecond )->Iterations( 2000000 );
BENCHMARK_TEMPLATE( quaternionCacheInsert, double, QuaternionBinningMode::Spherical )->Unit( benchmark::kNanosecond )->Iterations( 2000000 );
BENCHMARK_TEMPLATE( quaternionCacheInsert, double, QuaternionBinningMode::SphericalFibonacci )->Unit( benchmark::kNanosecond )->Iterations( 2000000 );
BENCHMARK_TEMPLATE( quaternionCacheFind, double, QuaternionBinningMode::LargestDim )->Unit( benchmark::kNanosecond )->Iterations( 2000000 );
BENCHMARK_TEMPLATE( quaternionCacheFind, double, QuaternionBinningMode::Spherical )->Unit( benchmark::kNanosecond )->Iterations( 2000000 );
BENCHMARK_TEMPLATE( quaternionCacheFind, double, QuaternionBinningMode::SphericalFibonacci )->Unit( benchmark::kNanosecond )->Iterations( 2000000 );

BENCHMARK_MAIN();
