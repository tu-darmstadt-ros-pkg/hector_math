// Copyright (c) 2023 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <hector_math_ros/robot/joint_state_subscriber.h>

using namespace hector_math;

class MockRobotModel : public RobotModel<double>
{
public:
  MockRobotModel() : RobotModel<double>( { "a", "b", "c" }, { 1, 2, 3 } ) { }
  explicit MockRobotModel( bool empty ) : RobotModel<double>( {} ) { }

  bool updated = false;
  double mass_ = 0;
  const double &mass() const override { return mass_; }

protected:
  Vector3<double> computeCenterOfMass() const override { return {}; }
  Polygon<double> computeFootprint() const override { return {}; }
  Eigen::AlignedBox<double, 3> computeAxisAlignedBoundingBox() const override { return {}; }

  void onJointStatesUpdated() override
  {
    RobotModel::onJointStatesUpdated();
    updated = true;
  }
};

//! @param wait_count Max time to wait in increments of 33 ms
bool waitFor( const std::function<bool()> &pred, int wait_count = 10 )
{
  while ( --wait_count > 0 ) {
    if ( pred() )
      return true;
    ros::spinOnce();
    ros::Duration( 0.033 ).sleep();
  }
  return false;
}

::testing::AssertionResult equal( const std::vector<double> &a, const std::vector<double> &b )
{
  if ( a.size() != b.size() )
    return ::testing::AssertionFailure() << "Size differed. " << a.size() << " vs " << b.size();
  for ( size_t i = 0; i < a.size(); ++i ) {
    if ( a[i] == b[i] )
      continue;
    return ::testing::AssertionFailure() << "Differed at " << i << ". a: " << a[i] << ", b: " << b[i];
  }
  return ::testing::AssertionSuccess();
}

TEST( JointStateSubscriber, tests )
{
  ros::NodeHandle nh;
  ros::Time::init();
  ASSERT_TRUE( ros::Time::isValid() );
  ros::Publisher pub = nh.advertise<sensor_msgs::JointState>( "/test_joint_states", 1 );
  auto model = std::make_shared<MockRobotModel>();
  auto sub = std::make_shared<JointStateSubscriber<double>>( model, "/test_joint_states" );
  ASSERT_TRUE( waitFor( [&]() { return pub.getNumSubscribers() == 1; } ) )
      << "Failed to connect joint state subscriber to publisher!";
  EXPECT_FALSE( sub->waitForFullState( ros::Duration( 1 ) ) );
  EXPECT_FALSE( model->updated );
  ASSERT_TRUE( equal( model->jointPositions(), { 1, 2, 3 } ) );
  // The following test relies on there not being another spinner that would dispatch the message
  // before we wait for it.
  sensor_msgs::JointState state;
  state.name = { "a", "b" };
  state.position = { 2, 3 };
  pub.publish( state );
  ASSERT_TRUE( waitFor( [&]() { return model->updated; } ) );
  EXPECT_TRUE( equal( model->jointPositions(), { 2, 3, 3 } ) );
  model->updated = false;
  state.position = { 3, 4 };
  pub.publish( state );
  ASSERT_FALSE( model->updated );
  EXPECT_FALSE( sub->waitForFullState( ros::Duration( 1 ) ) );
  ASSERT_TRUE( model->updated );
  ASSERT_TRUE( equal( model->jointPositions(), { 3, 4, 3 } ) );

  model->updated = false;
  state.position = { 5, 6 };
  pub.publish( state );
  state.name = { "c", "d" }; // include non-existant entry which should be ignored
  state.position = { 7, 8 };
  pub.publish( state );
  ASSERT_FALSE( model->updated );
  EXPECT_TRUE( sub->waitForFullState( ros::Duration( 1 ) ) );
  ASSERT_TRUE( model->updated );
  ASSERT_TRUE( equal( model->jointPositions(), { 5, 6, 7 } ) );

  sub->pause();
  model->updated = false;
  state.position = { 1, 2 };
  pub.publish( state );
  ASSERT_FALSE( waitFor( [&]() { return model->updated; }, 5 ) );
  ASSERT_TRUE( equal( model->jointPositions(), { 5, 6, 7 } ) );
  sub->resume();
  ASSERT_TRUE( model->updated );
  ASSERT_TRUE( equal( model->jointPositions(), { 5, 6, 1 } ) );
  model->updated = false;
  sub->pause();
  sub->resume();
  ASSERT_FALSE( model->updated );

  model = std::make_shared<MockRobotModel>( true ); // empty model
  sub = std::make_shared<JointStateSubscriber<double>>( model, "/test_joint_states" );
  ASSERT_TRUE(
      sub->waitForFullState( ros::Duration( 0.1 ) ) ); // Empty model should return immediately
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "test_joint_state_subscriber" );
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
