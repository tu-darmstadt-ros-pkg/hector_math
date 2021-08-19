//
// Created by Stefan Fabian on 17.08.21.
//

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
  virtual Scalar getMinimum( const GridMap<Scalar> &map, const Vector2<Scalar> &pos, Scalar orientation, Scalar maximum ) const = 0;
};
}

#endif //HECTOR_MATH_FOOTPRINT_ACCESSOR_H
