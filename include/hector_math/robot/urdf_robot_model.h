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

  /*!
   * @param model The urdf model of the robot.
   * @param joint_states Initial values for joints. Joints that are not specified default to 0.
   */
  explicit UrdfRobotModel( urdf::Model model, std::unordered_map<std::string, Scalar> joint_states = {} )
    : RobotModel<Scalar>( joint_states ), model_( std::move( model ))
  {
    link_tree_ = buildLinkTree( model_.root_link_ );
  }

  /*!
   * @param model The urdf model of the robot.
   * @param joint_names The names of joints that should be at the start of joint_positions. Joints that are not
   *   specified are added at the end.
   * @param joint_positions Initial values for the joint positions. Joints that are not specified default to 0.
   */
  explicit UrdfRobotModel( urdf::Model model, std::vector<std::string> joint_names,
                           std::vector<Scalar> joint_positions = {} )
    : RobotModel<Scalar>( joint_names, joint_positions ), model_( std::move( model ))
  {
    link_tree_ = buildLinkTree( model_.root_link_ );
  }

  const urdf::Model &urdfModel() const { return model_; }

protected:
  struct Joint;
  struct LinkTree
  {
    std::vector<Joint> children;
    Vector3<Scalar> inertial_origin;
    double inertial_mass;
  };
  struct Joint
  {
    LinkTree child_link;
    Isometry3<Scalar> parent_to_joint_transform;
    Vector3<Scalar> axis;
    size_t joint_state_index;
    double mimic_multiplier;
    double mimic_offset;
    int type;
    bool is_mimic;
  };

  Vector3<Scalar> computeCenterOfMass() const override;

  Polygon<Scalar> computeFootprint() const override
  {
    return Polygon<Scalar>();
  }

  void onJointStatesUpdated() override;

  Isometry3<Scalar> transformForJoint( const Joint &joint,
                                       const std::vector<Scalar> &joint_positions ) const;

  Scalar computeWeightedCenterOfMass( const LinkTree &root, const Isometry3<Scalar> &transform,
                                      const std::vector<Scalar> &joint_positions, Vector3<Scalar> &com ) const;

  LinkTree buildLinkTree( const urdf::LinkSharedPtr &root );

  urdf::Model model_;
  LinkTree link_tree_;
};
}

#include "../impl/robot/urdf_robot_model_impl.hpp"

#endif //HECTOR_MATH_URDF_ROBOT_MODEL_H
