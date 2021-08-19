//
// Created by stefan on 19.08.21.
//

#include "hector_math/robot/urdf_robot_model.h"
#include <benchmark/benchmark.h>
#include <ros/package.h>
#include <fstream>

using namespace hector_math;

template<typename Scalar>
static void centerOfMass( benchmark::State &state )
{
  std::string package_path = ros::package::getPath( ROS_PACKAGE_NAME );
  // Inefficient, doesn't matter here but don't copy this approach
  std::ifstream urdf_stream( package_path + "/benchmark/pr2.urdf" );
  std::string urdf((std::istreambuf_iterator<char>( urdf_stream )), std::istreambuf_iterator<char>());

  urdf::Model model;
  if ( !model.initString( urdf ))
  {
    throw std::runtime_error( "Failed to load robot description!" );
  }
  UrdfRobotModel<Scalar> robot_model( model );

  for ( auto _ : state )
  {
    robot_model.updateJointPositions(std::unordered_map<std::string, Scalar>{});
    Vector3<Scalar> com = robot_model.centerOfMass();
    benchmark::DoNotOptimize( com );
  }
}

BENCHMARK_TEMPLATE( centerOfMass, float )->Unit( benchmark::kMicrosecond );
BENCHMARK_TEMPLATE( centerOfMass, double )->Unit( benchmark::kMicrosecond );

BENCHMARK_MAIN();
