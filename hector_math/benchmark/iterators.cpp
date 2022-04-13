//
// Created by stefan on 20.08.21.
//

#include "hector_math/iterators/circle_iterator.h"
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
  GridMap<Scalar> map( 20, 20 );

  for ( auto _ : state ) {
    // See test_hector_iterators for what this will iterate
    iterateRectangle<Scalar>( Vector2<Scalar>( 0, 1 ), Vector2<Scalar>( 1, 19 ),
                              Vector2<Scalar>( 18, 0 ),
                              [&map]( Eigen::Index x, Eigen::Index y ) { ++map( x, y ); } );
  }
}
BENCHMARK_TEMPLATE( rectangleIterator, float )->Unit( benchmark::kMicrosecond );
BENCHMARK_TEMPLATE( rectangleIterator, double )->Unit( benchmark::kMicrosecond );

template<typename Scalar>
static void polygonIterator( benchmark::State &state )
{
  Polygon<Scalar> polygon = createPolygon<Scalar>();
  GridMap<Scalar> map( 20, 20 );

  for ( auto _ : state ) {
    iteratePolygon<Scalar>( polygon / Scalar( 0.05 ),
                            [&map]( Eigen::Index x, Eigen::Index y ) { ++map( x, y ); } );
  }
}
BENCHMARK_TEMPLATE( polygonIterator, float )->Unit( benchmark::kMicrosecond );
BENCHMARK_TEMPLATE( polygonIterator, double )->Unit( benchmark::kMicrosecond );

#if BENCHMARK_ENABLE_GRIDMAP
static void comparisonGridmapPolygonIterator( benchmark::State &state )
{
  grid_map::GridMap map;
  map.setGeometry( grid_map::Length( 1, 1 ), 0.05 );
  Polygon<double> polygon = createPolygon<double>();
  grid_map::Polygon gm_polygon;
  for ( Eigen::Index i = 0; i < polygon.cols(); ++i )
    gm_polygon.addVertex( grid_map::Position( polygon.col( i ).x(), polygon.col( i ).y() ) );

  GridMap<float> data( 20, 20 );
  // For fairness we also just use the iterator to access a 2D array to rule out grid map access performance impacting the benchmark
  for ( auto _ : state ) {
    for ( grid_map::PolygonIterator iterator( map, gm_polygon ); !iterator.isPastEnd(); ++iterator ) {
      ++data( ( *iterator ).x(), ( *iterator ).y() );
    }
  }
}

BENCHMARK( comparisonGridmapPolygonIterator )->Unit( benchmark::kMicrosecond );
#endif

template<typename Scalar>
static void circleIterator( benchmark::State &state )
{
  GridMap<Scalar> map( 20, 20 );

  for ( auto _ : state ) {
    iterateCircle<Scalar>( Vector2<Scalar>( 10, 10 ), 10,
                           [&map]( Eigen::Index x, Eigen::Index y ) { ++map( x, y ); } );
  }
}
BENCHMARK_TEMPLATE( circleIterator, float )->Unit( benchmark::kMicrosecond );
BENCHMARK_TEMPLATE( circleIterator, double )->Unit( benchmark::kMicrosecond );

#if BENCHMARK_ENABLE_GRIDMAP
static void comparisonGridmapCircleIterator( benchmark::State &state )
{
  grid_map::GridMap map( { "type" } );
  map.setGeometry( grid_map::Length( 20, 20 ), 1 );
  GridMap<float> data( 20, 20 );

  // For fairness we also just use the iterator to access a 2D array to rule out grid map access performance impacting the benchmark
  for ( auto _ : state ) {
    for ( grid_map::CircleIterator iterator( map, grid_map::Position( 10, 10 ), 10 );
          !iterator.isPastEnd(); ++iterator ) {
      ++data( ( *iterator ).x(), ( *iterator ).y() );
    }
  }
}

BENCHMARK( comparisonGridmapCircleIterator )->Unit( benchmark::kMicrosecond );
#endif

BENCHMARK_MAIN();
