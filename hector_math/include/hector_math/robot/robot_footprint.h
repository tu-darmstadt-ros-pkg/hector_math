// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_FOOTPRINT_ACCESSOR_H
#define HECTOR_MATH_FOOTPRINT_ACCESSOR_H

#include "hector_math/types.h"
#include <urdf_model/model.h>

namespace hector_math
{

template<typename Scalar>
class RobotFootprint
{
public:
  virtual Scalar getMinimum( const GridMap<Scalar> &map, const Vector2<Scalar> &pos,
                             Scalar orientation, Scalar maximum ) const = 0;
};
} // namespace hector_math

#endif // HECTOR_MATH_FOOTPRINT_ACCESSOR_H
