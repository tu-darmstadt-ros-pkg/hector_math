//
// Created by stefan on 19.08.21.
//

#include "hector_math_ros/urdf/robot_model.h"
#include <benchmark/benchmark.h>
#include <ros/package.h>
#include <fstream>

using namespace hector_math;

urdf::Model loadUrdf()
{
  std::string package_path = ros::package::getPath( ROS_PACKAGE_NAME );
  // Inefficient, doesn't matter here but don't copy this approach
  std::ifstream urdf_stream( package_path + "/benchmark/pr2.urdf" );
  std::string urdf((std::istreambuf_iterator<char>( urdf_stream )), std::istreambuf_iterator<char>());

  urdf::Model model;
  if ( !model.initString( urdf ))
  {
    throw std::runtime_error( "Failed to load urdf description!" );
  }
  return model;
}

template<typename Scalar>
static void centerOfMass( benchmark::State &state )
{
  urdf::Model model = loadUrdf();
  std::vector<std::string> joint_names;
  std::vector<Scalar> joint_values;
  for ( const auto &joint : model.joints_ )
  {
    joint_names.push_back( joint.first );
    joint_values.push_back( 0 );
  }
  UrdfRobotModel<Scalar> robot_model( model, joint_names, joint_values );

  for ( auto _ : state )
  {
    robot_model.updateJointPositions( std::vector<Scalar>{} );
    Vector3<Scalar> com = robot_model.centerOfMass();
    if ( !com.template isApprox( Vector3<Scalar>( -0.01612981, 0.004683246, 0.4767422 ), 0.00001 ))
      throw std::runtime_error( "COM is not where it should be!" );
  }
}

template<typename Scalar>
static void boundingBox( benchmark::State &state )
{
  urdf::Model model = loadUrdf();
  std::vector<std::string> joint_names;
  std::vector<Scalar> joint_values;
  for ( const auto &joint : model.joints_ )
  {
    joint_names.push_back( joint.first );
    joint_values.push_back( 0 );
  }
  UrdfRobotModel<Scalar> robot_model( model, joint_names, joint_values );

  for ( auto _ : state )
  {
    robot_model.updateJointPositions( std::vector<Scalar>{} );
    Eigen::AlignedBox<Scalar, 3> bb = robot_model.axisAlignedBoundingBox();
    if ( !bb.isApprox( Eigen::AlignedBox<Scalar, 3>( Vector3<Scalar>( -0.315, -0.3484, -0.0005 ),
                                                     Vector3<Scalar>( 0.7715, 0.3484, 1.241625 )),
                       0.0001 ))
      throw std::runtime_error( "Bounding box is not what it should be!" );
    benchmark::DoNotOptimize( bb );
  }
}

BENCHMARK_TEMPLATE( centerOfMass, float )->Unit( benchmark::kMicrosecond );
BENCHMARK_TEMPLATE( centerOfMass, double )->Unit( benchmark::kMicrosecond );
BENCHMARK_TEMPLATE( boundingBox, float )->Unit( benchmark::kMicrosecond );
BENCHMARK_TEMPLATE( boundingBox, double )->Unit( benchmark::kMicrosecond );

BENCHMARK_MAIN();
