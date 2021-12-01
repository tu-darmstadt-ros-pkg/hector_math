//
// Created by Stefan Fabian on 17.08.21.
//

#ifndef HECTOR_MATH_URDF_ROBOT_MODEL_H
#define HECTOR_MATH_URDF_ROBOT_MODEL_H

#include <hector_math/robot/robot_model.h>
#include <urdf/model.h>

namespace hector_math
{

struct UrdfRobotModelSettings {
  //! If there is no collision geometry, fall back on visual geometries for
  //! bounding box and footprint computation.
  bool fallback_on_visual = false;
};

template<typename Scalar>
class UrdfRobotModel : public RobotModel<Scalar>
{
public:
  using Ptr = std::shared_ptr<UrdfRobotModel<Scalar>>;
  using ConstPtr = std::shared_ptr<const UrdfRobotModel<Scalar>>;

  /*!
   * @param model The urdf model of the urdf.
   * @param joint_states Initial values for joints. Joints that are not
   * specified default to 0.
   */
  explicit UrdfRobotModel( urdf::Model model,
                           std::unordered_map<std::string, Scalar> joint_states = {} )
      : RobotModel<Scalar>( joint_states ), model_( std::move( model ) )
  {
    link_tree_ = buildLinkTree( model_.root_link_ );
  }

  /*!
   * @param model The urdf model of the urdf.
   * @param joint_names The names of joints that should be at the start of
   * joint_positions. Joints that are not specified are added at the end.
   * @param joint_positions Initial values for the joint positions. Joints that
   * are not specified default to 0.
   */
  explicit UrdfRobotModel( urdf::Model model, std::vector<std::string> joint_names,
                           std::vector<Scalar> joint_positions = {} )
      : RobotModel<Scalar>( joint_names, joint_positions ), model_( std::move( model ) )
  {
    link_tree_ = buildLinkTree( model_.root_link_ );
  }

  const urdf::Model &urdfModel() const { return model_; }

  struct Joint;

  struct LinkGeometry {
    Isometry3<Scalar> origin;
    Vector3<Scalar> dims;
    int type;
#if __cplusplus < 201703L

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
  };

  struct LinkTree {
#if __cplusplus >= 201703L // In C++17 the alignment macros and allocators are
                           // not necessary anymore
    std::vector<Joint> children;
    std::vector<LinkGeometry> geometries;
#else
    std::vector<Joint, Eigen::aligned_allocator<Joint>> children;
    std::vector<LinkGeometry, Eigen::aligned_allocator<LinkGeometry>> geometries;
#endif
    Vector3<Scalar> inertial_origin;
    double inertial_mass;
#if __cplusplus < 201703L

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
  };

  struct Joint {
    LinkTree child_link;
    Isometry3<Scalar> parent_to_joint_transform;
    Vector3<Scalar> axis;
    size_t joint_state_index;
    double mimic_multiplier;
    double mimic_offset;
    int type;
    bool is_mimic;

    Isometry3<Scalar> getTransform( const std::vector<Scalar> &joint_positions ) const;
#if __cplusplus < 201703L

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
  };

  const LinkTree &linkTree() const { return link_tree_; }

protected:
  Vector3<Scalar> computeCenterOfMass() const override;

  Polygon<Scalar> computeFootprint() const override
  {
    Polygon<Scalar> result( 2, 4 );
    const Eigen::AlignedBox<Scalar, 3> &bb = this->axisAlignedBoundingBox();
    result.col( 0 ) << bb.min().x(), bb.min().y();
    result.col( 1 ) << bb.min().x(), bb.max().y();
    result.col( 2 ) << bb.max().x(), bb.max().y();
    result.col( 3 ) << bb.max().x(), bb.min().y();
    return result;
  }

  Eigen::AlignedBox<Scalar, 3>
  computeAxisAlignedBoundingBox( const LinkTree &root, const Isometry3<Scalar> &transform ) const;

  Eigen::AlignedBox<Scalar, 3> computeAxisAlignedBoundingBox() const override;

  void onJointStatesUpdated() override;

  Isometry3<Scalar> transformForJoint( const Joint &joint ) const;

  Scalar computeWeightedCenterOfMass( const LinkTree &root, const Isometry3<Scalar> &transform,
                                      Vector3<Scalar> &com ) const;

  LinkTree buildLinkTree( const urdf::LinkSharedPtr &root );

  urdf::Model model_;
  LinkTree link_tree_;
  UrdfRobotModelSettings settings_;
#if __cplusplus < 201703L

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};
} // namespace hector_math

#include "../impl/urdf/robot_model_impl.hpp"

#endif // HECTOR_MATH_URDF_ROBOT_MODEL_H
