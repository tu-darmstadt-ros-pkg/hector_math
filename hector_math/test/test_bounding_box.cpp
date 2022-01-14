
#include <hector_math/shapes/bounding_box.h>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "hector_math/types.h"
#include "eigen_tests.h"


using namespace hector_math;
template<typename Scalar>
class BoundingBoxTest : public testing::Test
{
};

typedef testing::Types<float, double> Implementations;

TYPED_TEST_CASE( BoundingBoxTest, Implementations );

TYPED_TEST( BoundingBoxTest, sphere )
{
  using Scalar = TypeParam;

  // Test Case 1: sphere placed at origin
  Isometry3<Scalar> transform;
  transform.setIdentity();
  Eigen::AlignedBox<Scalar, 3> box = computeBoundingBoxForSphere<Scalar>(3,transform);
  Vector3<Scalar> minimum = {-3,-3,-3};
  Vector3<Scalar> maximum = {3,3,3};
  EXPECT_TRUE(EIGEN_ARRAY_EQUAL(box.min(),minimum ));
  EXPECT_TRUE(EIGEN_ARRAY_EQUAL(box.max(),maximum));

  // Test Case 2 : Sphere shifted
  transform.translation().x() = 1.4;
  transform.translation().y() = -2.4;
  transform.translation().z() = 3.4;
  box = computeBoundingBoxForSphere<Scalar>(10,transform);
  minimum = {-8.6,-12.4,-6.6};
  maximum = {11.4,7.6,13.4};
  EXPECT_TRUE(EIGEN_ARRAY_EQUAL(box.min(),minimum ));
  EXPECT_TRUE(EIGEN_ARRAY_EQUAL(box.max(),maximum));

  // Test Case 3 radius 0
  box = computeBoundingBoxForSphere<Scalar>(0,transform);
  minimum = {1.4,-2.4,3.4};
  maximum = {1.4,-2.4,3.4};
  EXPECT_TRUE(EIGEN_ARRAY_EQUAL(box.min(),minimum ));
  EXPECT_TRUE(EIGEN_ARRAY_EQUAL(box.max(),maximum));
}

TYPED_TEST( BoundingBoxTest, box ) {
  using Scalar = TypeParam;

  // Test Case 1: box placed at origin
  Isometry3<Scalar> transform;
  transform.setIdentity();
  Eigen::AlignedBox<Scalar, 3> box = computeBoundingBoxForBox<Scalar>({4,6,8},transform);
  Vector3<Scalar> minimum {-2,-3,-4}; // half of cube length/width/height
  Vector3<Scalar> maximum  {2,3,4};
  EXPECT_TRUE(EIGEN_ARRAY_EQUAL(box.min(),minimum ));
  EXPECT_TRUE(EIGEN_ARRAY_EQUAL(box.max(),maximum));

  // Test Case 2: box shifted
  transform.translation().x() = 1;
  transform.translation().y() = -2;
  transform.translation().z() = 3;
  box = computeBoundingBoxForBox<Scalar>({6,4,5},transform);
  minimum = {-2,-4,0.5};
  maximum = {4,0,5.5};
  EXPECT_TRUE(EIGEN_ARRAY_EQUAL(box.min(),minimum ));
  EXPECT_TRUE(EIGEN_ARRAY_EQUAL(box.max(),maximum));

  // Test Case 3: 0 height
  box = computeBoundingBoxForBox<Scalar>({6,4,0},transform);
  minimum = {-2,-4,3};
  maximum = {4,0,3};
  EXPECT_TRUE(EIGEN_ARRAY_EQUAL(box.min(),minimum ));
  EXPECT_TRUE(EIGEN_ARRAY_EQUAL(box.max(),maximum));

  // Test Case 4 all dim 0
  box = computeBoundingBoxForBox<Scalar>({0,0,0},transform);
  minimum = transform.translation();
  maximum = transform.translation();
  EXPECT_TRUE(EIGEN_ARRAY_EQUAL(box.min(),minimum ));
  EXPECT_TRUE(EIGEN_ARRAY_EQUAL(box.max(),maximum));

  // Test Case 5: rotated Box 45° around x-Axis
  transform.setIdentity();
  transform.rotate(Eigen::AngleAxis<Scalar>(M_PI/4, Vector3<Scalar>{1,0,0}));
  box = computeBoundingBoxForBox<Scalar>({4,4,4},transform);
  minimum = {-2,-2/std::cos(M_PI/4),-2/std::cos(M_PI/4)};
  maximum = {2,2/std::cos(M_PI/4),2/std::cos(M_PI/4)};
  EXPECT_TRUE(EIGEN_ARRAY_NEAR(box.min(),minimum,1e-4 ));
  EXPECT_TRUE(EIGEN_ARRAY_NEAR(box.max(),maximum,1e-4));

  // Test Case 6: rotated Box 30° around [1,2,3] axis & different dimensions
  transform.setIdentity();// undo previous rotation
  transform.translation().x() = 10;
  transform.translation().y() = -2;
  transform.translation().z() = 6;
  transform.rotate(Eigen::Quaternion<Scalar>(0.9659258, 0.0691723, 0.1383446, 0.2075169 ));
  box = computeBoundingBoxForBox<Scalar>({4,3,2},transform);
  minimum = {7.380211, -4.2727308, 4.28417075};
  maximum = {12.619789, 0.2727308, 7.71582925};
  EXPECT_TRUE(EIGEN_ARRAY_NEAR(box.min(),minimum,1e-4 ));
  EXPECT_TRUE(EIGEN_ARRAY_NEAR(box.max(),maximum,1e-4));

}

TYPED_TEST( BoundingBoxTest, cylinder )
{
  using Scalar = TypeParam;

  // Test Case 1: cylinder placed at origin
  Isometry3<Scalar> transform;
  transform.setIdentity();
  Eigen::AlignedBox<Scalar, 3> box = computeBoundingBoxForCylinder<Scalar>( 2,10, transform );
  Vector3<Scalar> minimum{ -2, -2, -5 }; // height equals half of cylinder length
  Vector3<Scalar> maximum{ 2, 2, 5 };
  EXPECT_TRUE( EIGEN_ARRAY_EQUAL( box.min(), minimum ) );
  EXPECT_TRUE( EIGEN_ARRAY_EQUAL( box.max(), maximum ) );

  // Test Case 2: cylinder shifted
  transform.translation().x() = 1;
  transform.translation().y() = -2;
  transform.translation().z() = 3;
  box = computeBoundingBoxForCylinder<Scalar>( 2,10, transform );
  minimum = { -1, -4, -2 }; // height equals half of cylinder length
  maximum = { 3, 0, 8 };
  EXPECT_TRUE( EIGEN_ARRAY_EQUAL( box.min(), minimum ) );
  EXPECT_TRUE( EIGEN_ARRAY_EQUAL( box.max(), maximum ) );

  //Testcase rotation around z-axis -> bounding box should be equal + previous translation [1,-2,3]
  transform.rotate(Eigen::Quaternion<Scalar>(  0.976296,0, 0, 0.2164396 ));
  box = computeBoundingBoxForCylinder<Scalar>( 2,10, transform );
  minimum = { -1, -4, -2 }; // height equals half of cylinder length
  maximum = { 3, 0, 8 };
  EXPECT_TRUE( EIGEN_ARRAY_EQUAL( box.min(), minimum ) );
  EXPECT_TRUE( EIGEN_ARRAY_EQUAL( box.max(), maximum ) );

  // Test Case 4: cylinder rotated, 30° degrees around [1,0,0]
  transform.setIdentity();
  transform.rotate(Eigen::Quaternion<Scalar>(0.9659258,0.258819, 0, 0));
  box = computeBoundingBoxForCylinder<Scalar>( 1.5,12, transform );
  minimum = { -1.5, -4.2, -5.9 }; // height equals half of cylinder length
  maximum = { 1.5, 4.2, 5.9 };
  EXPECT_TRUE( EIGEN_ARRAY_NEAR( box.min(), minimum ,1e-01) );
  EXPECT_TRUE( EIGEN_ARRAY_NEAR( box.max(), maximum, 1e-01 ) );
}


  int main( int argc, char **argv )
{
  ros::init( argc, argv, "test_hector_iterators" );
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
