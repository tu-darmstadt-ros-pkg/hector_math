//
// Created by stefan on 20.08.21.
//

#include "hector_math/iterators/polygon_iterator.h"
#include "iterators_input.h"

#if BENCHMARK_ENABLE_GRIDMAP

#include <grid_map_core/iterators/PolygonIterator.hpp>

#endif

#include <benchmark/benchmark.h>
#include <ros/package.h>

using namespace hector_math;

template<typename Scalar>
static void polygonIterator( benchmark::State &state )
{
  Polygon<Scalar> polygon = createPolygon<Scalar>();

  for ( auto _ : state ) {
    int count = 0;
    iteratePolygon<Scalar>( polygon / Scalar( 0.05 ),
                            [&count]( Eigen::Index x, Eigen::Index y ) { ++count; } );
    benchmark::DoNotOptimize( count );
  }
}
BENCHMARK_TEMPLATE( polygonIterator, float )->Unit( benchmark::kMicrosecond );
BENCHMARK_TEMPLATE( polygonIterator, double )->Unit( benchmark::kMicrosecond );

#if BENCHMARK_ENABLE_GRIDMAP
static void comparisonGridmapPolygonIterator( benchmark::State &state )
{
  grid_map::GridMap map;
  map.setGeometry( grid_map::Length( 20, 20 ), 0.05 );
  Polygon<double> polygon = createPolygon<double>();
  grid_map::Polygon gm_polygon;
  for ( Eigen::Index i = 0; i < polygon.cols(); ++i )
    gm_polygon.addVertex( grid_map::Position( polygon.col( i ).x(), polygon.col( i ).y() ) );

  for ( auto _ : state ) {
    int count = 0;
    for ( grid_map::PolygonIterator iterator( map, gm_polygon ); !iterator.isPastEnd(); ++iterator ) {
      ++count;
    }
    benchmark::DoNotOptimize( count );
  }
}

BENCHMARK( comparisonGridmapPolygonIterator )->Unit( benchmark::kMicrosecond );
#endif

BENCHMARK_MAIN();
