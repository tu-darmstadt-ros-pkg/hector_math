//
// Created by stefan on 13.02.23.
//

#ifndef HECTOR_MATH_DIRECTION_DISCRETIZATION_H
#define HECTOR_MATH_DIRECTION_DISCRETIZATION_H

#include "hector_math/types.h"

namespace hector_math
{
/*!
 * Computes the opposing direction for the given direction.
 * For example for DIRECTIONS=8, if passed direction = 0 which points in positive x-direction, this
 * method will return 4 which points in negative x-direction.
 *
 * @tparam DIRECTIONS The number of directions.
 * @param direction The direction we want the inverse or opposite direction for.
 * @return The inverse/opposite direction of the given direction in the range [0, DIRECTIONS).
 */
template<int DIRECTIONS>
constexpr int invertDirection( int direction ) noexcept;

//! @return The given direction + 1 wrapped to the range [0, DIRECTIONS).
template<int DIRECTIONS>
constexpr int incrementDirection( int direction ) noexcept;

//! @return The given direction - 1 wrapped to the range [0, DIRECTIONS).
template<int DIRECTIONS>
constexpr int decrementDirection( int direction ) noexcept;

/*!
 * @tparam DIRECTIONS Encodes the number of directions the local x-y-plane is split into.
 *   0 points in positive x-direction and increments are equidistant angles counter-clockwise when
 *   viewed from above.
 *
 * @return The direction for the given angle (in rad) as a scalar as it may lie between two direction increments.
 */
template<int DIRECTIONS, typename Scalar>
constexpr Scalar directionFromAngle( Scalar angle ) noexcept;

//! This method only uses the rotation part around the z-axis of the quaternion. @see directionFromAngle
//! @return The direction for the given quaternion as a scalar as it may lie between two direction increments.
template<int DIRECTIONS, typename Scalar>
constexpr Scalar directionFromQuaternion( const Eigen::Quaternion<Scalar> &q );

//! This method only uses the rotation part around the z-axis of the quaternion. @see directionFromAngle
//! @return The direction for the given quaternion as a scalar as it may lie between two direction increments.
template<int DIRECTIONS, typename Scalar>
constexpr Scalar directionFromQuaternionRounded( const Eigen::Quaternion<Scalar> &q );

//! @see directionFromAngle
//! @return The rotation angle (in rad) around the z-axis for the given direction.
template<int DIRECTIONS, typename Scalar>
constexpr Scalar angleFromDirection( int direction );

//! @see directionFromAngle
//! @return The rotation angle (in rad) around the z-axis for the given direction.
template<typename Scalar>
constexpr Scalar angleFromDirection( int direction, int directions );

//! @see directionFromAngle
//! @return A quaternion encoding the rotation around the z-axis for the given direction.
template<int DIRECTIONS, typename Scalar>
constexpr Eigen::Quaternion<Scalar> quaternionFromDirection( int direction );

//! @see directionFromAngle
//! @return A quaternion encoding the rotation around the z-axis for the given direction.
template<typename Scalar>
constexpr Eigen::Quaternion<Scalar> quaternionFromDirection( int direction, int directions );

//! @see directionFromAngle
//! @return The rotation angle (in rad) around the z-axis for the given direction difference.
template<int DIRECTIONS, typename Scalar>
constexpr Scalar angleFromDirectionDifference( int diff );

//! @see directionFromAngle
//! @return The minimum angle (in rad) needed to rotate from one orientation to another.
template<int DIRECTIONS, typename Scalar>
constexpr Scalar minAngleBetweenDirections( int from, int to );

// =======================
// === Implementations ===
// =======================

template<int DIRECTIONS>
constexpr int invertDirection( int direction ) noexcept
{
  assert( 0 <= direction && direction < DIRECTIONS && "Passed direction was already out of bounds!" );
  constexpr bool is_power_of_two = ( DIRECTIONS & ( DIRECTIONS - 1 ) ) == 0;
  if ( is_power_of_two ) {
    return ( direction + DIRECTIONS / 2 ) & ( DIRECTIONS - 1 );
  }
  // Slow general implementation
  int result = direction + DIRECTIONS / 2;
  if ( result >= DIRECTIONS )
    result -= DIRECTIONS;
  return result;
}

template<int DIRECTIONS>
constexpr int incrementDirection( int direction ) noexcept
{
  assert( 0 <= direction && direction < DIRECTIONS && "Passed direction was already out of bounds!" );
  constexpr bool is_power_of_two = ( DIRECTIONS & ( DIRECTIONS - 1 ) ) == 0;
  if ( is_power_of_two ) {
    return ( direction + 1 ) & ( DIRECTIONS - 1 );
  }
  // Slow general implementation
  int result = direction + 1;
  if ( result >= DIRECTIONS )
    result -= DIRECTIONS;
  return result;
}

template<int DIRECTIONS>
constexpr int decrementDirection( int direction ) noexcept
{
  assert( 0 <= direction && direction < DIRECTIONS && "Passed direction was already out of bounds!" );
  constexpr bool is_power_of_two = ( DIRECTIONS & ( DIRECTIONS - 1 ) ) == 0;
  if ( is_power_of_two ) {
    return ( direction - 1 ) & ( DIRECTIONS - 1 );
  }
  // Slow general implementation
  int result = direction - 1;
  if ( result >= DIRECTIONS )
    result -= DIRECTIONS;
  return result;
}

template<int DIRECTIONS, typename Scalar>
constexpr Scalar directionFromAngle( Scalar angle ) noexcept
{
  constexpr Scalar multiplier = DIRECTIONS / Scalar( 2 * M_PI );
  return angle * multiplier;
}

template<int DIRECTIONS, typename Scalar>
constexpr Scalar directionFromQuaternion( const Eigen::Quaternion<Scalar> &q )
{
  if ( q.coeffs().isZero() )
    return std::numeric_limits<Scalar>::quiet_NaN();
  constexpr Scalar multiplier = DIRECTIONS / Scalar( 2 * M_PI );
  bool identity = std::abs( q.w() ) < 1E-9 && std::abs( q.z() ) < 1E-9;
  Scalar angle =
      identity ? 0 : 2 * std::acos( q.w() / hector_math::Vector2<Scalar>( q.w(), q.z() ).norm() );
  return angle * multiplier;
}

template<int DIRECTIONS, typename Scalar>
constexpr Scalar directionFromQuaternionRounded( const Eigen::Quaternion<Scalar> &q )
{
  if ( q.coeffs().isZero() )
    return std::numeric_limits<Scalar>::quiet_NaN();
  constexpr Scalar multiplier = DIRECTIONS / Scalar( 2 * M_PI );
  bool identity = std::abs( q.w() ) < 1E-9 && std::abs( q.z() ) < 1E-9;
  Scalar angle =
      identity ? 0 : 2 * std::acos( q.w() / hector_math::Vector2<Scalar>( q.w(), q.z() ).norm() );
  int direction = std::round( angle * multiplier );
  constexpr bool is_power_of_two = ( DIRECTIONS & ( DIRECTIONS - 1 ) ) == 0;
  if ( is_power_of_two ) {
    return direction & ( DIRECTIONS - 1 );
  }
  // Slow general implementation
  if ( direction >= DIRECTIONS )
    direction -= DIRECTIONS;
  return direction;
}

template<int DIRECTIONS, typename Scalar>
constexpr Scalar angleFromDirection( int direction )
{
  constexpr Scalar multiplier = Scalar( 2 * M_PI ) / DIRECTIONS;
  return direction * multiplier;
}

template<typename Scalar>
constexpr Scalar angleFromDirection( int direction, int directions )
{
  const Scalar multiplier = Scalar( 2 * M_PI ) / directions;
  return direction * multiplier;
}

template<int DIRECTIONS, typename Scalar>
constexpr Eigen::Quaternion<Scalar> quaternionFromDirection( int direction )
{
  const Scalar angle = angleFromDirection<DIRECTIONS, Scalar>( direction ) / 2;
  return Eigen::Quaternion<Scalar>( std::cos( angle ), 0, 0, std::sin( angle ) );
}

template<typename Scalar>
constexpr Eigen::Quaternion<Scalar> quaternionFromDirection( int direction, int directions )
{
  const Scalar angle = angleFromDirection<Scalar>( direction, directions ) / 2;
  return Eigen::Quaternion<Scalar>( std::cos( angle ), 0, 0, std::sin( angle ) );
}

template<int DIRECTIONS, typename Scalar>
constexpr Scalar angleFromDirectionDifference( int diff )
{
  constexpr Scalar multiplier = 2 * Scalar( M_PI ) / DIRECTIONS;
  return diff * multiplier;
}

template<int DIRECTIONS, typename Scalar>
constexpr Scalar minAngleBetweenDirections( int from, int to )
{
  int direction_diff = to - from;
  if ( direction_diff < -DIRECTIONS / 2 )
    direction_diff += DIRECTIONS;
  else if ( direction_diff > DIRECTIONS / 2 )
    direction_diff -= DIRECTIONS;
  return angleFromDirectionDifference<DIRECTIONS, Scalar>( direction_diff );
}
} // namespace hector_math

#endif // HECTOR_MATH_DIRECTION_DISCRETIZATION_H
