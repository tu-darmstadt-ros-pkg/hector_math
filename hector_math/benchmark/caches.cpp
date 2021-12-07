//
// Created by stefan on 06.12.21.
//
#include "hector_math/containers/quaternion_cache.h"

#include <benchmark/benchmark.h>
#include <ros/package.h>

using namespace hector_math;

template<typename Scalar>
static void quaternionCacheInsert( benchmark::State &state )
{
  QuaternionCache<Scalar, int> cache;
  std::vector<Eigen::Quaternion<Scalar>, Eigen::aligned_allocator<Eigen::Quaternion<Scalar>>> quaternions;
  quaternions.resize(state.max_iterations);
  for (size_t i = 0; i < state.max_iterations; ++i) quaternions[i] = Eigen::Quaternion<Scalar>::UnitRandom();
  size_t i = 0;
  for ( auto _ : state ) {
    cache.insert(quaternions[i], i);
  }
}

template<typename Scalar>
static void quaternionCacheFind( benchmark::State &state )
{
  QuaternionCache<Scalar, int> cache;
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
BENCHMARK_TEMPLATE( quaternionCacheInsert, float )->Unit( benchmark::kNanosecond );
BENCHMARK_TEMPLATE( quaternionCacheInsert, double )->Unit( benchmark::kNanosecond );
BENCHMARK_TEMPLATE( quaternionCacheFind, float )->Unit( benchmark::kNanosecond );
BENCHMARK_TEMPLATE( quaternionCacheFind, double )->Unit( benchmark::kNanosecond );

BENCHMARK_MAIN();
