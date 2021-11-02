//
// Created by Stefan Fabian on 17.08.21.
//

#include "hector_math/map_operations/find_minmax.h"
#include "hector_math/robot/urdf_robot_footprint.h"
#include "hector_math/robot/urdf_robot_model.h"

#include <gtest/gtest.h>
#include <ros/ros.h>

int main( int argc, char **argv )
{
  ros::init( argc, argv, "test_math" );
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
