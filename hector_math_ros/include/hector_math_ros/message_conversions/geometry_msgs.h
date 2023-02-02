//
// Created by stefan on 19.10.21.
//

#ifndef HECTOR_MATH_ROS_CONVERSIONS_GEOMETRY_MSGS_H
#define HECTOR_MATH_ROS_CONVERSIONS_GEOMETRY_MSGS_H

#include <hector_math/types.h>

#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>

namespace hector_math
{
/*!
 * IMPORTANT: These conversion methods are provided as headers but the library does not depend on
 * the message types. Hence, make sure that your project depends on them if you use them.
 */
template<typename Scalar>
inline Pose<Scalar> msgToPose( const geometry_msgs::Pose &msg );
template<typename Scalar>
inline Pose<Scalar> msgToPose( const geometry_msgs::Transform &msg );

template<typename Scalar>
inline Eigen::Quaternion<Scalar> msgToQuaternion( const geometry_msgs::Quaternion &msg );

template<typename Scalar>
inline Isometry3<Scalar> msgToTransform( const geometry_msgs::Pose &msg );
template<typename Scalar>
inline Isometry3<Scalar> msgToTransform( const geometry_msgs::Transform &msg );

template<typename Scalar>
inline Twist<Scalar> msgToTwist( const geometry_msgs::Twist &msg );

template<typename Scalar>
inline Vector3<Scalar> msgToVector( const geometry_msgs::Point &msg );
template<typename Scalar>
inline Vector3<Scalar> msgToVector( const geometry_msgs::Vector3 &msg );

template<typename Derived>
inline geometry_msgs::Point vectorToPointMsg( const Eigen::DenseBase<Derived> &vec );
template<typename Derived>
inline geometry_msgs::Vector3 vectorToVectorMsg( const Eigen::DenseBase<Derived> &vec );

template<typename Scalar>
inline geometry_msgs::Pose poseToPoseMsg( const Pose<Scalar> &pose );
template<typename Scalar>
inline geometry_msgs::Transform poseToTransformMsg( const Pose<Scalar> &pose );

template<typename Scalar>
inline geometry_msgs::Quaternion quaternionToMsg( const Eigen::Quaternion<Scalar> &q );

template<typename Scalar>
inline geometry_msgs::Pose transformToPoseMsg( const Isometry3<Scalar> &pose );
template<typename Scalar>
inline geometry_msgs::Transform transformToTransformMsg( const Isometry3<Scalar> &pose );

template<typename Scalar>
inline geometry_msgs::Twist twistToMsg( const hector_math::Twist<Scalar> &twist );

template<typename Scalar>
Pose<Scalar> msgToPose( const geometry_msgs::Pose &msg )
{
  return { msgToVector<Scalar>( msg.position ), msgToQuaternion<Scalar>( msg.orientation ) };
}

template<typename Scalar>
Pose<Scalar> hector_math::msgToPose( const geometry_msgs::Transform &msg )
{
  return { msgToVector<Scalar>( msg.translation ), msgToQuaternion<Scalar>( msg.rotation ) };
}

template<typename Scalar>
Eigen::Quaternion<Scalar> msgToQuaternion( const geometry_msgs::Quaternion &msg )
{
  return { static_cast<Scalar>( msg.w ), static_cast<Scalar>( msg.x ), static_cast<Scalar>( msg.y ),
           static_cast<Scalar>( msg.z ) };
}

template<typename Scalar>
Isometry3<Scalar> msgToTransform( const geometry_msgs::Pose &msg )
{
  Isometry3<Scalar> transform = Isometry3<Scalar>::Identity();
  transform.linear() = msgToQuaternion<Scalar>( msg.orientation ).toRotationMatrix();
  transform.translation() = msgToVector<Scalar>( msg.position );
  return transform;
}

template<typename Scalar>
Isometry3<Scalar> hector_math::msgToTransform( const geometry_msgs::Transform &msg )
{
  Isometry3<Scalar> transform = Isometry3<Scalar>::Identity();
  transform.linear() = msgToQuaternion<Scalar>( msg.rotation ).toRotationMatrix();
  transform.translation() = msgToVector<Scalar>( msg.translation );
  return transform;
}

template<typename Scalar>
Twist<Scalar> msgToTwist( const geometry_msgs::Twist &msg )
{
  return Twist<Scalar>{ msgToVector<Scalar>( msg.linear ), msgToVector<Scalar>( msg.angular ) };
}

template<typename Scalar>
Vector3<Scalar> msgToVector( const geometry_msgs::Point &msg )
{
  return { static_cast<Scalar>( msg.x ), static_cast<Scalar>( msg.y ), static_cast<Scalar>( msg.z ) };
}

template<typename Scalar>
Vector3<Scalar> hector_math::msgToVector( const geometry_msgs::Vector3 &msg )
{
  return { static_cast<Scalar>( msg.x ), static_cast<Scalar>( msg.y ), static_cast<Scalar>( msg.z ) };
}

template<typename Derived>
geometry_msgs::Point vectorToPointMsg( const Eigen::DenseBase<Derived> &vec )
{
  geometry_msgs::Point msg;
  msg.x = vec.x();
  msg.y = vec.y();
  msg.z = vec.z();
  return msg;
}

template<typename Derived>
geometry_msgs::Vector3 vectorToVectorMsg( const Eigen::DenseBase<Derived> &vec )
{
  geometry_msgs::Vector3 msg;
  msg.x = vec.x();
  msg.y = vec.y();
  msg.z = vec.z();
  return msg;
}

template<typename Scalar>
geometry_msgs::Pose poseToPoseMsg( const Pose<Scalar> &pose )
{
  geometry_msgs::Pose msg;
  msg.position = vectorToPointMsg( pose.translation() );
  msg.orientation = quaternionToMsg( pose.orientation() );
  return msg;
}

template<typename Scalar>
geometry_msgs::Transform poseToTransformMsg( const Pose<Scalar> &pose )
{
  geometry_msgs::Transform msg;
  msg.translation = vectorToVectorMsg( pose.translation() );
  msg.rotation = quaternionToMsg( pose.orientation() );
  return msg;
}

template<typename Scalar>
geometry_msgs::Quaternion quaternionToMsg( const Eigen::Quaternion<Scalar> &q )
{
  geometry_msgs::Quaternion msg;
  msg.w = q.w();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  return msg;
}

template<typename Scalar>
geometry_msgs::Pose transformToPoseMsg( const Isometry3<Scalar> &pose )
{
  geometry_msgs::Pose msg;
  msg.position = vectorToPointMsg( pose.translation() );
  msg.orientation = quaternionToMsg( Eigen::Quaternion<Scalar>( pose.linear() ) );
  return msg;
}

template<typename Scalar>
geometry_msgs::Transform transformToTransformMsg( const Isometry3<Scalar> &pose )
{
  geometry_msgs::Transform msg;
  msg.translation = vectorToVectorMsg( pose.translation() );
  msg.rotation = quaternionToMsg( Eigen::Quaternion<Scalar>( pose.linear() ) );
  return msg;
}

template<typename Scalar>
geometry_msgs::Twist twistToMsg( const Twist<Scalar> &twist )
{
  geometry_msgs::Twist msg;
  msg.linear = vectorToVectorMsg( twist.linear() );
  msg.angular = vectorToVectorMsg( twist.angular() );
  return msg;
}
} // namespace hector_math

#endif // HECTOR_MATH_ROS_CONVERSIONS_GEOMETRY_MSGS_H
