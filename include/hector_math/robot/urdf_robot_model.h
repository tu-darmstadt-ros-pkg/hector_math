//
// Created by Stefan Fabian on 17.08.21.
//

#ifndef HECTOR_MATH_URDF_ROBOT_MODEL_H
#define HECTOR_MATH_URDF_ROBOT_MODEL_H

#include "hector_math/robot/robot_model.h"
#include <urdf/model.h>

namespace hector_math
{

template<typename Scalar>
class UrdfRobotModel : public RobotModel<Scalar>
{
public:

  explicit UrdfRobotModel( urdf::Model model, std::unordered_map<std::string, Scalar> joint_states = {} )
  : RobotModel<Scalar>(joint_states), model_( std::move( model ))
  {
  }

  const urdf::Model &urdfModel() const { return model_; }

protected:
  Vector3<Scalar> computeCenterOfMass() const override;

  Polygon<Scalar> computeFootprint() const override
  {
    return Polygon<Scalar>();
  }

private:
  urdf::Model model_;
};
}

#include "../impl/robot/urdf_robot_model_impl.hpp"

#endif //HECTOR_MATH_URDF_ROBOT_MODEL_H
