//
// Created by stefan on 30.06.22.
//

#ifndef HECTOR_MATH_OPERATIONS_H
#define HECTOR_MATH_OPERATIONS_H

namespace hector_math
{

/*!
 * Clamps the given value to the range [min, max]. Propagates NaN values.
 * @param val The input value.
 * @param min The minimum value of the return value.
 * @param max The maximum value of the return value.
 * @return val if it is within min and max, min if val is smaller or equal to min, max if val is larger or equal to max.
 */
template<typename T>
T clamp( const T &val, const T &min, const T &max )
{
  if ( val < min )
    return min;
  return val > max ? max : val;
}
} // namespace hector_math

#endif // HECTOR_MATH_OPERATIONS_H
