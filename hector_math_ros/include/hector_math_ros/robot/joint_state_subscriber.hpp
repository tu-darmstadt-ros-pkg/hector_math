// Copyright (c) 2023 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_ROS_JOINT_STATE_SUBSCRIBER_H
#define HECTOR_MATH_ROS_JOINT_STATE_SUBSCRIBER_H

#include "hector_math/robot/robot_model.h"

#include <cmath>
#include <mutex>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace hector_math
{

/*!
 * Convenience class to keep a RobotModel in sync with the actual joint positions as published on
 * /joint_states
 */
template<typename Scalar>
class JointStateSubscriber
{
public:
  explicit JointStateSubscriber( rclcpp::Node &node, typename RobotModel<Scalar>::Ptr model,
                                 const std::string &topic = "/joint_states",
                                 rclcpp::QoS qos = rclcpp::QoS( 10 ) )
      : model_( std::move( model ) ), clock_( node.get_clock() )
  {
    const std::vector<std::string> &joint_names = model_->jointNames();
    for ( size_t i = 0; i < joint_names.size(); ++i ) { indexes_.insert( { joint_names[i], i } ); }
    positions_ = model_->jointPositions();
    updated_.resize( positions_.size() );
    updated_.assign( positions_.size(), false );
    sub_ = node.create_subscription<sensor_msgs::msg::JointState>(
        topic, qos,
        std::bind( &JointStateSubscriber<Scalar>::onJointStateMessage, this, std::placeholders::_1 ) );
  }

  // Moving or copying is not allowed. Would be anyway due to mutex but also doesn't really make sense.
  JointStateSubscriber( const JointStateSubscriber<Scalar> & ) = delete;
  JointStateSubscriber( JointStateSubscriber<Scalar> && ) = delete;
  JointStateSubscriber &operator=( const JointStateSubscriber<Scalar> & ) = delete;
  JointStateSubscriber &operator=( JointStateSubscriber<Scalar> &&other ) = delete;

  /*!
   * Wait for a full update of all joint positions.
   * Usually, the joint states are broadcast by multiple publishers, hence, this method waits until
   * it received an update for each joint position in the RobotModel.
   * If the node clock hasn't started yet, this method will first wait until it is before waiting the specified
   * duration.
   *
   * @param timeout The maximum duration to wait before giving up. Pass -1 to wait indefinitely. Default: -1
   * @param spin If true spin the ros main CallbackQueue while waiting to process messages, if you have
   *   your own spinner, pass false.
   * @return True if a full update was received, false if not.
   */
  bool waitForFullState( const std::chrono::milliseconds &timeout = -1 )
  {
    using namespace std::chrono_literals;
    if ( positions_.empty() )
      return true;
    waiting_for_set_ = true;
    clock_->wait_until_started();
    rclcpp::Time start_time = clock_->now();
    while ( timeout.count() < 0 || ( clock_->now() - start_time ) < timeout ) {
      clock_->sleep_for( 10ms );
      if ( std::all_of( updated_.begin(), updated_.end(), []( bool x ) { return x; } ) ) {
        waiting_for_set_ = false;
        updateJointPositions();
        return true;
      }
    }
    waiting_for_set_ = false;
    if ( std::any_of( updated_.begin(), updated_.end(), []( bool x ) { return x; } ) )
      updateJointPositions();
    return false;
  }

  //! Stops updating the joint states of the model but will continue collecting changes.
  void pause() { paused_ = true; }

  //! Updates the joint positions of the model to the latest information and resumes
  //! continuously updating the model.
  void resume()
  {
    paused_ = false;
    if ( !std::any_of( updated_.begin(), updated_.end(), []( bool x ) { return x; } ) )
      return;
    updateJointPositions();
  }

  //! The minimum difference in joint position before the model is updated.
  //! This is used to avoid unnecessary updates due to position noise.
  Scalar minJointPositionChange() const { return min_joint_position_change_; }

  //! @copydoc minJointPositionChange
  void setMinJointPositionChange( Scalar value ) { min_joint_position_change_ = value; }

private:
  void onJointStateMessage( const sensor_msgs::msg::JointState::ConstSharedPtr &msg )
  {
    {
      std::lock_guard lock( update_mutex_ );
      for ( size_t i = 0; i < msg->name.size(); ++i ) {
        auto it = indexes_.find( msg->name[i] );
        if ( it == indexes_.end() || msg->position.size() <= i )
          continue;
        positions_[it->second] = msg->position[i];
        updated_[it->second] = true;
      }
    }
    if ( paused_ || waiting_for_set_ )
      return;
    updateJointPositions();
  }

  void updateJointPositions()
  {
    std::lock_guard lock( update_mutex_ );
    bool changed = false;
    const auto &current_positions = model_->jointPositions();
    for ( size_t i = 0; i < positions_.size(); ++i ) {
      if ( std::abs( current_positions[i] - positions_[i] ) >= min_joint_position_change_ ) {
        changed = true;
        break;
      }
    }
    updated_.assign( updated_.size(), false );
    if ( !changed )
      return;
    model_->updateJointPositions( positions_ );
  }

  typename RobotModel<Scalar>::Ptr model_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::SubscriptionBase::SharedPtr sub_;
  std::unordered_map<std::string, size_t> indexes_;
  std::vector<Scalar> positions_;
  typedef std::vector<bool> BoolBitset;
  BoolBitset updated_;
  std::mutex update_mutex_;
  std::atomic<bool> paused_{ false };
  std::atomic<bool> waiting_for_set_{ false };
  std::atomic<Scalar> min_joint_position_change_{ 1E-4 };
};
} // namespace hector_math

#endif // HECTOR_MATH_ROS_JOINT_STATE_SUBSCRIBER_H
