// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <hector_math/map_operations/fit_plane.h>
#include <hector_math/types.h>

#include <benchmark/benchmark.h>
#include <random>

using namespace hector_math;

struct GridMapGenerator {

  std::vector<GridMapf> createGridmaps( Eigen::Index size )
  {
    if ( cache.find( size ) != cache.end() ) {
      return cache[size];
    }
    std::vector<GridMapf> maps;
    std::uniform_int_distribution<int> type_distribution( 0, 1 );
    for ( int i = 0; i < 20; ++i ) {
      int type = type_distribution( generator );
      switch ( type ) {
      case 0:
        maps.push_back( createHallway( size ) );
        break;
      case 1:
        maps.push_back( createStairs( size ) );
        break;
      }
    }
    cache[size] = maps;
    return maps;
  }

  GridMapf createHallway( Eigen::Index size )
  {
    double angle = angle_distribution( generator );
    GridMapf result = GridMapf::Constant( size, size, std::numeric_limits<float>::quiet_NaN() );
    for ( auto [row, col] : EigenIndexIterator( result ) ) {
      // Corridor in non-rotated frame is wall at x = 0.1 * size and x = 0.9 * size
      // Get coordinates in rotated frame
      double x =
          ( row - size / 2 ) * std::cos( angle ) - ( col - size / 2 ) * std::sin( angle ) + size / 2;
      if ( std::abs( x - 0.1 * size ) < 2 || std::abs( x - 0.9 * size ) < 2 ) {
        result( row, col ) = 2.0f + noise_distribution( generator );
      } else if ( 0.1 * size <= x && x <= 0.9 * size ) {
        result( row, col ) = 0.0f + noise_distribution( generator );
      }
    }
    return result;
  }

  GridMapf createStairs( Eigen::Index size )
  {
    float angle = angle_distribution( generator );
    GridMapf result = GridMapf::Constant( size, size, std::numeric_limits<float>::quiet_NaN() );
    for ( auto [row, col] : EigenIndexIterator( result ) ) {
      // Assuming a resolution of 0.025m, steps every 8 cells (0.2m) with height 0.2m
      // Get coordinates in rotated frame
      float y =
          ( row - size / 2 ) * std::sin( angle ) + ( col - size / 2 ) * std::cos( angle ) + size / 2;
      result( row, col ) = std::roundf( y / 8 ) * 0.2f + noise_distribution( generator );
    }
    return result;
  }

  std::default_random_engine generator{ 42 };
  std::uniform_real_distribution<float> noise_distribution{ -0.05, 0.05 };
  std::uniform_real_distribution<float> angle_distribution{ -M_PI, M_PI };
  std::unordered_map<int, std::vector<GridMapf>> cache;
};

static void halfNan( std::vector<GridMapf> &maps )
{
  std::default_random_engine generator( 42 );
  std::uniform_int_distribution<int> distribution( 0, 1 );
  for ( auto &map : maps ) {
    for ( auto [row, col] : EigenIndexIterator( map ) ) {
      if ( distribution( generator ) == 0 ) {
        map( row, col ) = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }
}

static void fitPlane( benchmark::State &state )
{
  std::vector<GridMapf> maps = GridMapGenerator().createGridmaps( state.range() );
  size_t i = 0;
  for ( auto _ : state ) {
    PlaneEstimationResult result = hector_math::fitPlaneXY( maps[i] );
    benchmark::DoNotOptimize( result );
    if ( ++i == maps.size() )
      i = 0;
  }
}

static void fitPlaneHalfUnknown( benchmark::State &state )
{
  std::vector<GridMapf> maps = GridMapGenerator().createGridmaps( state.range() );
  halfNan( maps );
  size_t i = 0;
  for ( auto _ : state ) {
    PlaneEstimationResult result = hector_math::fitPlaneXY( maps[i] );
    benchmark::DoNotOptimize( result );
    if ( ++i == maps.size() )
      i = 0;
  }
}

static void fitPlaneRobust( benchmark::State &state )
{
  std::vector<GridMapf> maps = GridMapGenerator().createGridmaps( state.range() );
  size_t i = 0;
  for ( auto _ : state ) {
    PlaneEstimationResult result;
    hector_math::fitPlaneXYRobust( maps[i], result );
    benchmark::DoNotOptimize( result );
    if ( ++i == maps.size() )
      i = 0;
  }
}

static void fitPlaneRobustHalfUnknown( benchmark::State &state )
{
  std::vector<GridMapf> maps = GridMapGenerator().createGridmaps( state.range() );
  halfNan( maps );
  size_t i = 0;
  for ( auto _ : state ) {
    PlaneEstimationResult result;
    hector_math::fitPlaneXYRobust( maps[i], result );
    benchmark::DoNotOptimize( result );
    if ( ++i == maps.size() )
      i = 0;
  }
}

BENCHMARK( fitPlane )->Arg( 100 )->Arg( 1000 )->Unit( benchmark::kMicrosecond );
BENCHMARK( fitPlaneHalfUnknown )->Arg( 100 )->Arg( 1000 )->Unit( benchmark::kMicrosecond );
BENCHMARK( fitPlaneRobust )->Arg( 100 )->Arg( 1000 )->Unit( benchmark::kMicrosecond );
BENCHMARK( fitPlaneRobustHalfUnknown )->Arg( 100 )->Arg( 1000 )->Unit( benchmark::kMicrosecond );

BENCHMARK_MAIN();
