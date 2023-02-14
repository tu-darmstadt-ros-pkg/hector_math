// Copyright (c) 2023 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_MATH_TWIST_H
#define HECTOR_MATH_TWIST_H

#include "hector_math/types/eigen.h"

namespace hector_math
{

template<typename Scalar>
class Twist
{
public:
  explicit Twist( const Vector3<Scalar> &linear = Vector3<Scalar>::Zero(),
                  const Vector3<Scalar> &angular = Vector3<Scalar>::Zero() )
      : linear_( linear ), angular_( angular )
  {
  }

  static Twist<Scalar> Zero() { return Twist<Scalar>(); }

  const Vector3<Scalar> &linear() const { return linear_; }
  Vector3<Scalar> &linear() { return linear_; }

  const Vector3<Scalar> &angular() const { return angular_; }
  Vector3<Scalar> &angular() { return angular_; }

private:
  Vector3<Scalar> linear_;
  Vector3<Scalar> angular_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace hector_math

#endif // HECTOR_MATH_TWIST_H
