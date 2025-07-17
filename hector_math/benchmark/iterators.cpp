// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "hector_math/iterators/circle_iterator.h"
#include "hector_math/iterators/eigen_iterator.h"
#include "hector_math/iterators/polygon_iterator.h"
#include "hector_math/iterators/rectangle_iterator.h"
#include "iterators_input.h"

#if BENCHMARK_ENABLE_GRIDMAP

#include <grid_map_core/iterators/CircleIterator.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>

#endif

#include <benchmark/benchmark.h>
#include <ros/package.h>

using namespace hector_math;

template<typename Scalar>
static void rectangleIterator( benchmark::State &state )
{
  GridMap<Scalar> map( 200, 200 );

  for ( auto _ : state ) {
    // See test_hector_iterators for what this will iterate
    iterateRectangle<Scalar>( Vector2<Scalar>( 0, 1 ), Vector2<Scalar>( 1, 190 ),
                              Vector2<Scalar>( 180, 0 ),
                              [&map]( Eigen::Index x, Eigen::Index y ) { ++map( x, y ); } );
  }
  benchmark::DoNotOptimize( map );
}

static void rectangleIteratorClass( benchmark::State &state )
{
  GridMapd map( 200, 200 );

  for ( auto _ : state ) {
    for ( const auto &index :
          RectangleIndexIterator<>( Vector2d( 0, 1 ), Vector2d( 1, 190 ), Vector2d( 180, 0 ) ) ) {
      ++map( index.row, index.col );
    }
  }
  benchmark::DoNotOptimize( map );
}

BENCHMARK_TEMPLATE( rectangleIterator, float )->Unit( benchmark::kMicrosecond );
BENCHMARK_TEMPLATE( rectangleIterator, double )->Unit( benchmark::kMicrosecond );
BENCHMARK( rectangleIteratorClass )->Unit( benchmark::kMicrosecond );

template<typename Scalar>
static void polygonIterator( benchmark::State &state )
{
  Polygon<Scalar> polygon = createPolygon<Scalar>();
  GridMap<Scalar> map( 200, 200 );

  for ( auto _ : state ) {
    iteratePolygon<Scalar>( polygon / Scalar( 0.005 ),
                            [&map]( Eigen::Index x, Eigen::Index y ) { ++map( x, y ); } );
  }
  benchmark::DoNotOptimize( map );
}

static void polygonIteratorClass( benchmark::State &state )
{
  Polygond polygon = createPolygon<double>();
  GridMapd map( 200, 200 );

  for ( auto _ : state ) {
    for ( const auto &[row, col] : PolygonIndexIterator( polygon / 0.005 ) ) { ++map( row, col ); }
  }
  benchmark::DoNotOptimize( map );
}
BENCHMARK_TEMPLATE( polygonIterator, float )->Unit( benchmark::kMicrosecond );
BENCHMARK_TEMPLATE( polygonIterator, double )->Unit( benchmark::kMicrosecond );
BENCHMARK( polygonIteratorClass )->Unit( benchmark::kMicrosecond );

#if BENCHMARK_ENABLE_GRIDMAP
static void comparisonGridmapPolygonIterator( benchmark::State &state )
{
  grid_map::GridMap map;
  map.setGeometry( grid_map::Length( 1, 1 ), 0.005 );
  Polygon<double> polygon = createPolygon<double>();
  grid_map::Polygon gm_polygon;
  for ( Eigen::Index i = 0; i < polygon.cols(); ++i )
    gm_polygon.addVertex( grid_map::Position( polygon.col( i ).x(), polygon.col( i ).y() ) );

  GridMap<float> data( 200, 200 );
  // For fairness we also just use the iterator to access a 2D array to rule out grid map access performance impacting the benchmark
  for ( auto _ : state ) {
    for ( grid_map::PolygonIterator iterator( map, gm_polygon ); !iterator.isPastEnd(); ++iterator ) {
      ++data( ( *iterator ).x(), ( *iterator ).y() );
    }
  }
  benchmark::DoNotOptimize( data );
}

BENCHMARK( comparisonGridmapPolygonIterator )->Unit( benchmark::kMicrosecond );
#endif

template<typename Scalar>
static void circleIterator( benchmark::State &state )
{
  GridMap<Scalar> map( 200, 200 );

  for ( auto _ : state ) {
    iterateCircle<Scalar>( Vector2<Scalar>( 100, 100 ), 100,
                           [&map]( Eigen::Index x, Eigen::Index y ) { ++map( x, y ); } );
  }
  benchmark::DoNotOptimize( map );
}

static void circleIteratorClass( benchmark::State &state )
{
  GridMapd map( 200, 200 );

  for ( auto _ : state ) {
    for ( const auto &index : CircleIndexIterator( Vector2d( 100, 100 ), 100 ) ) {
      ++map( index.row, index.col );
    }
  }
  benchmark::DoNotOptimize( map );
}

BENCHMARK_TEMPLATE( circleIterator, float )->Unit( benchmark::kMicrosecond );
BENCHMARK_TEMPLATE( circleIterator, double )->Unit( benchmark::kMicrosecond );
BENCHMARK( circleIteratorClass )->Unit( benchmark::kMicrosecond );

#if BENCHMARK_ENABLE_GRIDMAP
static void comparisonGridmapCircleIterator( benchmark::State &state )
{
  grid_map::GridMap map( { "type" } );
  map.setGeometry( grid_map::Length( 200, 200 ), 1 );
  GridMap<float> data( 200, 200 );

  // For fairness we also just use the iterator to access a 2D array to rule out grid map access performance impacting the benchmark
  for ( auto _ : state ) {
    for ( grid_map::CircleIterator iterator( map, grid_map::Position( 100, 100 ), 100 );
          !iterator.isPastEnd(); ++iterator ) {
      ++data( ( *iterator ).x(), ( *iterator ).y() );
    }
  }
  benchmark::DoNotOptimize( data );
}

BENCHMARK( comparisonGridmapCircleIterator )->Unit( benchmark::kMicrosecond );
#endif

template<int Option>
static void eigenIterator( benchmark::State &state )
{
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Option> map( state.range( 0 ),
                                                                    state.range( 0 ) );
  map.setRandom();

  for ( auto _ : state ) {
    float sum = 0;
    iterateDenseBase( map, [&map, &sum]( Eigen::Index x, Eigen::Index y ) { sum += map( x, y ); } );
    benchmark::DoNotOptimize( sum );
  }
}

template<int Option>
static void eigenValueIteratorClass( benchmark::State &state )
{
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Option> map( state.range( 0 ),
                                                                    state.range( 0 ) );
  map.setRandom();

  for ( auto _ : state ) {
    float sum = 0;
    for ( const auto &val : EigenValueIterator( map ) ) { sum += val; }
    benchmark::DoNotOptimize( sum );
  }
}

template<int Option>
static void eigenIndexIteratorClass( benchmark::State &state )
{
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Option> map( state.range( 0 ),
                                                                    state.range( 0 ) );
  map.setRandom();

  for ( auto _ : state ) {
    float sum = 0;
    for ( const auto &[row, col] : EigenIndexIterator( map ) ) { sum += map( row, col ); }
    benchmark::DoNotOptimize( sum );
  }
}

template<int Option>
static void eigenRowColLoopBaseline( benchmark::State &state )
{
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Option> map( state.range( 0 ),
                                                                    state.range( 0 ) );
  map.setRandom();

  for ( auto _ : state ) {
    float sum = 0;
    for ( Eigen::Index row = 0; row < map.rows(); ++row ) {
      for ( Eigen::Index col = 0; col < map.cols(); ++col ) { sum += map( row, col ); }
    }
    benchmark::DoNotOptimize( sum );
  }
}

BENCHMARK_TEMPLATE( eigenIterator, Eigen::RowMajor )->Unit( benchmark::kMicrosecond )->Arg( 100 );
BENCHMARK_TEMPLATE( eigenIterator, Eigen::ColMajor )->Unit( benchmark::kMicrosecond )->Arg( 100 );
BENCHMARK_TEMPLATE( eigenValueIteratorClass, Eigen::RowMajor )->Unit( benchmark::kMicrosecond )->Arg( 100 );
BENCHMARK_TEMPLATE( eigenValueIteratorClass, Eigen::ColMajor )->Unit( benchmark::kMicrosecond )->Arg( 100 );
BENCHMARK_TEMPLATE( eigenIndexIteratorClass, Eigen::RowMajor )->Unit( benchmark::kMicrosecond )->Arg( 100 );
BENCHMARK_TEMPLATE( eigenIndexIteratorClass, Eigen::ColMajor )->Unit( benchmark::kMicrosecond )->Arg( 100 );
BENCHMARK_TEMPLATE( eigenRowColLoopBaseline, Eigen::RowMajor )->Unit( benchmark::kMicrosecond )->Arg( 100 );
BENCHMARK_TEMPLATE( eigenRowColLoopBaseline, Eigen::ColMajor )->Unit( benchmark::kMicrosecond )->Arg( 100 );
BENCHMARK_TEMPLATE( eigenIterator, Eigen::RowMajor )->Unit( benchmark::kMicrosecond )->Arg( 10000 );
BENCHMARK_TEMPLATE( eigenIterator, Eigen::ColMajor )->Unit( benchmark::kMicrosecond )->Arg( 10000 );
BENCHMARK_TEMPLATE( eigenValueIteratorClass, Eigen::RowMajor )
    ->Unit( benchmark::kMicrosecond )
    ->Arg( 10000 );
BENCHMARK_TEMPLATE( eigenValueIteratorClass, Eigen::ColMajor )
    ->Unit( benchmark::kMicrosecond )
    ->Arg( 10000 );
BENCHMARK_TEMPLATE( eigenIndexIteratorClass, Eigen::RowMajor )
    ->Unit( benchmark::kMicrosecond )
    ->Arg( 10000 );
BENCHMARK_TEMPLATE( eigenIndexIteratorClass, Eigen::ColMajor )
    ->Unit( benchmark::kMicrosecond )
    ->Arg( 10000 );
BENCHMARK_TEMPLATE( eigenRowColLoopBaseline, Eigen::RowMajor )
    ->Unit( benchmark::kMicrosecond )
    ->Arg( 10000 );
BENCHMARK_TEMPLATE( eigenRowColLoopBaseline, Eigen::ColMajor )
    ->Unit( benchmark::kMicrosecond )
    ->Arg( 10000 );

BENCHMARK_MAIN();
