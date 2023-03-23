// Copyright (c) 2023 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_ROS_JOINT_STATE_SUBSCRIBER_H
#define HECTOR_MATH_ROS_JOINT_STATE_SUBSCRIBER_H

#include "hector_math/robot/robot_model.h"

#include <mutex>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <sensor_msgs/JointState.h>

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
  explicit JointStateSubscriber( typename RobotModel<Scalar>::Ptr model,
                                 const std::string &topic = "/joint_states" )
      : JointStateSubscriber( model, {}, topic )
  {
  }

  explicit JointStateSubscriber( typename RobotModel<Scalar>::Ptr model, const ros::NodeHandle &nh,
                                 const std::string &topic = "/joint_states" )
      : model_( std::move( model ) ), nh_( nh )
  {
    const std::vector<std::string> &joint_names = model_->jointNames();
    for ( size_t i = 0; i < joint_names.size(); ++i ) { indexes_.insert( { joint_names[i], i } ); }
    positions_ = model_->jointPositions();
    updated_.resize( positions_.size() );
    updated_.assign( positions_.size(), false );
    sub_ = nh_.subscribe( topic, 10, &JointStateSubscriber<Scalar>::onJointStateMessage, this );
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
   * If ros::Time is not valid, this method will first wait until it is before waiting the specified
   * duration.
   *
   * @param duration The maximum duration to wait before giving up. Pass 0 to wait indefinitely. Default: 0
   * @param spin If true spin the ros main CallbackQueue while waiting to process messages, if you have
   *   your own spinner, pass false.
   * @return True if a full update was received, false if not.
   */
  bool waitForFullState( const ros::Duration &duration = ros::Duration( 0 ), bool spin = true )
  {
    if ( positions_.empty() )
      return true;
    waiting_for_set_ = true;
    if ( !duration.isZero() && !ros::Time::isValid() ) {
      ROS_INFO_NAMED( "JointStateSubscriber", "Waiting for time to become valid." );
      ros::Time::waitForValid();
    }
    ros::Time start_time = ros::Time::now();
    while ( duration.isZero() || ( ros::Time::now() - start_time ) < duration ) {
      if ( spin )
        ros::spinOnce();
      else
        ros::Duration( 0.01 ).sleep();
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

private:
  void onJointStateMessage( const sensor_msgs::JointState::ConstPtr &msg )
  {
    {
      std::lock_guard<std::mutex> lock( update_mutex_ );
      for ( size_t i = 0; i < msg->name.size(); ++i ) {
        auto it = indexes_.find( msg->name[i] );
        if ( it == indexes_.end() )
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
    std::lock_guard<std::mutex> lock( update_mutex_ );
    model_->updateJointPositions( positions_ );
    updated_.assign( updated_.size(), false );
  }

  typename RobotModel<Scalar>::Ptr model_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::unordered_map<std::string, size_t> indexes_;
  std::vector<Scalar> positions_;
  typedef std::vector<bool> BoolBitset;
  BoolBitset updated_;
  std::mutex update_mutex_;
  std::atomic<bool> paused_{ false };
  std::atomic<bool> waiting_for_set_{ false };
};
} // namespace hector_math

#endif // HECTOR_MATH_ROS_JOINT_STATE_SUBSCRIBER_H
