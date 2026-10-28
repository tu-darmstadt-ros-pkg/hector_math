// Copyright (c) 2023 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <ament_index_cpp/get_package_share_path.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <hector_math_ros/robot/joint_state_subscriber.hpp>

using namespace hector_math;
using namespace std::chrono_literals;

class MockRobotModel : public RobotModel<double>
{
public:
  MockRobotModel() : RobotModel<double>( { "a", "b", "c" }, { 1, 2, 3 } ) { }
  explicit MockRobotModel( bool /* empty */ ) : RobotModel<double>( {} ) { }

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
bool waitFor( const rclcpp::Node::SharedPtr &node, const std::function<bool()> &pred,
              std::chrono::milliseconds timeout = std::chrono::milliseconds( 200 ) )
{
  rclcpp::Time start = node->get_clock()->now();
  while ( ( node->get_clock()->now() - start ) < timeout && rclcpp::ok() ) {
    if ( pred() )
      return true;
    usleep( 33 * 1000 );
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
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>( "test_joint_state_subscriber" );
  rclcpp::executors::MultiThreadedExecutor executor( {}, 2 );
  executor.add_node( node );
  std::promise<void> spin_promise;
  std::future<void> spin_future = spin_promise.get_future();
  std::thread spin_thread( [&]() { executor.spin_until_future_complete( spin_future ); } );
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub =
      node->create_publisher<sensor_msgs::msg::JointState>( "/test_joint_states", 1 );
  auto model = std::make_shared<MockRobotModel>();
  auto sub = std::make_shared<JointStateSubscriber<double>>( *node, model, "/test_joint_states" );
  ASSERT_TRUE( waitFor( node, [&]() { return pub->get_subscription_count() == 1; } ) )
      << "Failed to connect joint state subscriber to publisher!";
  EXPECT_FALSE( sub->waitForFullState( 1s ) );
  EXPECT_FALSE( model->updated );
  ASSERT_TRUE( equal( model->jointPositions(), { 1, 2, 3 } ) );
  // The following test relies on there not being another spinner that would dispatch the message
  // before we wait for it.
  sensor_msgs::msg::JointState state;
  state.name = { "a", "b" };
  state.position = { 2, 3 };
  pub->publish( state );
  ASSERT_TRUE( waitFor( node, [&]() { return model->updated; } ) );
  EXPECT_TRUE( equal( model->jointPositions(), { 2, 3, 3 } ) );
  model->updated = false;
  state.position = { 3, 4 };
  pub->publish( state );
  ASSERT_FALSE( model->updated );
  EXPECT_FALSE( sub->waitForFullState( 1s ) );
  ASSERT_TRUE( model->updated );
  ASSERT_TRUE( equal( model->jointPositions(), { 3, 4, 3 } ) );

  model->updated = false;
  state.position = { 5, 6 };
  pub->publish( state );
  state.name = { "c", "d" }; // include non-existant entry which should be ignored
  state.position = { 7, 8 };
  pub->publish( state );
  ASSERT_FALSE( model->updated );
  EXPECT_TRUE( sub->waitForFullState( 1s ) );
  ASSERT_TRUE( model->updated );
  ASSERT_TRUE( equal( model->jointPositions(), { 5, 6, 7 } ) );

  sub->pause();
  model->updated = false;
  state.position = { 1, 2 };
  pub->publish( state );
  ASSERT_FALSE( waitFor( node, [&]() { return model->updated; } ) );
  ASSERT_TRUE( equal( model->jointPositions(), { 5, 6, 7 } ) );
  sub->resume();
  ASSERT_TRUE( model->updated );
  ASSERT_TRUE( equal( model->jointPositions(), { 5, 6, 1 } ) );
  model->updated = false;
  sub->pause();
  sub->resume();
  ASSERT_FALSE( model->updated );

  model = std::make_shared<MockRobotModel>( true ); // empty model
  sub = std::make_shared<JointStateSubscriber<double>>( *node, model, "/test_joint_states" );
  ASSERT_TRUE( sub->waitForFullState( 100ms ) ); // Empty model should return immediately
  spin_promise.set_value();
  spin_thread.join();
}

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
