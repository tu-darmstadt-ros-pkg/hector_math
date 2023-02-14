// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_URDF_ROBOT_FOOTPRINT_H
#define HECTOR_MATH_URDF_ROBOT_FOOTPRINT_H

#include <hector_math/robot/robot_footprint.h>

namespace hector_math
{

template<typename Scalar>
class UrdfRobotFootprint : public RobotFootprint<Scalar>
{
public:
  Scalar getMinimum( const GridMap<Scalar> &map, const Vector2<Scalar> &pos, Scalar orientation,
                     Scalar maximum ) const override
  {
    return 0;
  }
};
} // namespace hector_math

#endif // HECTOR_MATH_URDF_ROBOT_FOOTPRINT_H
