//
// Created by Stefan Fabian on 30.11.21.
//

#include "eigen_tests.h"
#include <hector_math/types.h>

using namespace hector_math;

template<typename Scalar>
::testing::AssertionResult posesAreNear( const Pose<Scalar> &a, const Pose<Scalar> &b,
                                         Scalar precision )
{
  ::testing::AssertionResult result =
      EIGEN_MATRIX_NEAR( a.translation(), b.translation(), precision );
  if ( !result )
    return result;
  result = EIGEN_QUATERNIONS_NEAR( a.orientation(), b.orientation(), precision );
  return result;
}

template<typename T>
class PoseTest : public ::testing::Test
{
public:
  static T shared_;
  T value_;
};

typedef ::testing::Types<double, float> MyTypes;
TYPED_TEST_CASE( PoseTest, MyTypes );

TYPED_TEST( PoseTest, constructors )
{
  const TypeParam PRECISION = 4 * std::numeric_limits<TypeParam>::epsilon();
  using PoseT = Pose<TypeParam>;
  using QuaternionT = Eigen::Quaternion<TypeParam>;
  using TranslationT = Eigen::Translation<TypeParam, 3>;

  PoseT pose( TranslationT( 1.11, 2.22, 3.33 ) );
  EXPECT_TRUE( EIGEN_MATRIX_NEAR( pose.translation(), Vector3<TypeParam>( 1.11, 2.22, 3.33 ), 1E-9 ) );
  EXPECT_TRUE( EIGEN_QUATERNIONS_NEAR( pose.orientation(), QuaternionT::Identity(), PRECISION ) );
  pose = PoseT::Translation( 1.12, 2.23, 3.34 );
  EXPECT_TRUE( EIGEN_MATRIX_NEAR( pose.translation(), Vector3<TypeParam>( 1.12, 2.23, 3.34 ), 1E-9 ) );
  EXPECT_TRUE( EIGEN_QUATERNIONS_NEAR( pose.orientation(), QuaternionT::Identity(), PRECISION ) );
  pose = PoseT::Translation( Vector3<TypeParam>( 1.13, 2.24, 3.35 ) );
  EXPECT_TRUE( EIGEN_MATRIX_NEAR( pose.translation(), Vector3<TypeParam>( 1.13, 2.24, 3.35 ), 1E-9 ) );
  EXPECT_TRUE( EIGEN_QUATERNIONS_NEAR( pose.orientation(), QuaternionT::Identity(), PRECISION ) );

  pose = PoseT( QuaternionT( 0.7154386984244389, 0.38423560866431405, -0.037022701876509426,
                             0.5823570943818509 ) );
  EXPECT_TRUE( EIGEN_MATRIX_NEAR( pose.translation(), Vector3<TypeParam>::Zero(), PRECISION ) );
  EXPECT_TRUE( EIGEN_QUATERNIONS_NEAR(
      pose.orientation(),
      QuaternionT( 0.71543869842, 0.384235608664, -0.0370227018765, 0.58235709438 ), 1E-9 ) );

  Isometry3<TypeParam> isometry = Isometry3<TypeParam>::Identity();
  EXPECT_TRUE( posesAreNear( PoseT( isometry ), PoseT::Identity(), TypeParam( 1E-9 ) ) );
  isometry.linear() << 0.8700247, -0.1102823, 0.4805152, 0.3182428, 0.8700247, -0.3765349,
      -0.3765349, 0.4805152, 0.7920395;
  isometry.translation() << 100.100, 200.221, -300.130;
  PoseT expected;
  expected.translation() = { 100.100, 200.221, -300.130 };
  expected.orientation() = { 0.9396926, 0.2280134, 0.2280134, 0.1140067 };
  EXPECT_TRUE( posesAreNear( PoseT( isometry ), expected, TypeParam( 1E-6 ) ) );
  EXPECT_TRUE( EIGEN_MATRIX_NEAR( isometry.linear(), expected.rotation(), TypeParam( 1E-6 ) ) );
  EXPECT_TRUE(
      EIGEN_MATRIX_NEAR( expected.asTransform().matrix(), isometry.matrix(), TypeParam( 1E-6 ) ) );
  EXPECT_TRUE( posesAreNear( PoseT( isometry.translation(), isometry.linear() ), expected,
                             TypeParam( 1E-6 ) ) );
  EXPECT_TRUE( posesAreNear( PoseT( TranslationT( isometry.translation() ), isometry.linear() ),
                             expected, TypeParam( 1E-6 ) ) );
  EXPECT_TRUE(
      posesAreNear( PoseT( isometry.translation(),
                           Eigen::AngleAxis<TypeParam>(
                               0.6981317, Vector3<TypeParam>( 0.666667, 0.666667, 0.333333 ) ) ),
                    expected, TypeParam( 1E-6 ) ) );
  EXPECT_TRUE(
      posesAreNear( PoseT( 100.100, 200.221, -300.130, 0.9396926, 0.2280134, 0.2280134, 0.1140067 ),
                    expected, TypeParam( 1E-6 ) ) );
}

TYPED_TEST( PoseTest, poseTransformations )
{
  const TypeParam PRECISION = 4 * std::numeric_limits<TypeParam>::epsilon();
  using PoseT = Pose<TypeParam>;
  using QuaternionT = Eigen::Quaternion<TypeParam>;
  using TranslationT = Eigen::Translation<TypeParam, 3>;

  // Rotations
  EXPECT_TRUE(
      posesAreNear( PoseT::Identity() * QuaternionT::Identity(), PoseT::Identity(), PRECISION ) );
  EXPECT_TRUE(
      posesAreNear( QuaternionT::Identity() * PoseT::Identity(), PoseT::Identity(), PRECISION ) );
  PoseT expected = PoseT::Identity();
  QuaternionT orientation( 0.71, 0, 0, 0.71 );
  expected.orientation() = orientation;
  EXPECT_TRUE( EIGEN_QUATERNIONS_NEAR( expected.orientation(), orientation, PRECISION ) );
  EXPECT_TRUE( posesAreNear( orientation * PoseT::Identity(), expected, PRECISION ) );
  EXPECT_TRUE( posesAreNear( PoseT::Identity() * orientation, expected, PRECISION ) );
  PoseT input = PoseT::Identity();
  input *= orientation;
  EXPECT_TRUE( posesAreNear( input, expected, PRECISION ) );

  // Translations
  EXPECT_TRUE(
      posesAreNear( PoseT::Identity() * TranslationT::Identity(), PoseT::Identity(), PRECISION ) );
  EXPECT_TRUE(
      posesAreNear( TranslationT::Identity() * PoseT::Identity(), PoseT::Identity(), PRECISION ) );
  expected.setIdentity();
  EXPECT_TRUE( posesAreNear( expected, PoseT::Identity(), PRECISION ) );
  expected.translation() = { 2.1, 1.3, 3.141592 };
  TranslationT translation = { 2.1, 1.3, 3.141592 };
  EXPECT_TRUE( posesAreNear( PoseT::Identity() * translation, expected, PRECISION ) );
  EXPECT_TRUE( posesAreNear( translation * PoseT::Identity(), expected, PRECISION ) );
  input.setIdentity();
  input *= translation;
  EXPECT_TRUE( posesAreNear( input, expected, PRECISION ) );
  expected.translation() = -expected.translation();
  EXPECT_TRUE( posesAreNear( input.inverse(), expected, PRECISION ) );

  // Transforms
  EXPECT_TRUE( posesAreNear( PoseT::Identity() * PoseT::Identity(), PoseT::Identity(), PRECISION ) );
  input.setIdentity();
  input.orientation() = { 0.7154387, 0.3842356, -0.0370227, 0.5823571 };
  input.normalize();
  EXPECT_TRUE(
      posesAreNear( ( input * input.inverse() ).normalized(), PoseT::Identity(), PRECISION ) );
  EXPECT_TRUE( posesAreNear( input.inverse() * input, PoseT::Identity(), TypeParam( 1E-9 ) ) );
  input.translation() = { 2.1, 3.2, 4.31 };
  EXPECT_TRUE( posesAreNear( input * input.inverse(), PoseT::Identity(), PRECISION ) );
  EXPECT_TRUE( posesAreNear( input.inverse() * input, PoseT::Identity(), PRECISION ) );
  input *= input.inverse();
  EXPECT_TRUE( posesAreNear( input, PoseT::Identity(), PRECISION ) );
  EXPECT_TRUE( posesAreNear( PoseT::Identity().inverse(), PoseT::Identity(), PRECISION ) );

  Isometry3<TypeParam> isometry = Isometry3<TypeParam>::Identity();
  // clang-format off
  isometry.linear() << 0.8700247, -0.1102823, 0.4805152,
                       0.3182428, 0.8700247, -0.3765349,
                      -0.3765349, 0.4805152, 0.7920395;
  // clang-format on
  isometry.translation() << 100.100, 200.221, -300.130;
  input.orientation() = { 0.7154387, 0.3842356, -0.0370227, 0.5823571 };
  input.normalize();
  input.translation() = { 2.1, 3.2, 4.31 };
  expected.translation() = { 103.64516888, 202.05052333, -295.96938442 };
  expected.orientation() = { 0.5267306646647845, 0.6611990407435172, 0.039359864962600966,
                             0.5327489431728354 };
  EXPECT_TRUE( posesAreNear( isometry * input, expected, TypeParam( 1E-4 ) ) );

  PoseT a( TranslationT( 2.22, 3.33, 1 ), QuaternionT( 0.854, 0.354, 0.354, 0.146 ).normalized() );
  PoseT b( TranslationT( 1.11, 0, 2.22 ), QuaternionT( 0.954, 0, 0.301, 0 ).normalized() );
  PoseT c( TranslationT( 4.57466381, 2.7728981, 1.5528981 ),
           QuaternionT( 0.7074612860298535, 0.29347931969943314, 0.5941814854397381,
                        0.24559474757895383 ) );
  EXPECT_TRUE( posesAreNear( a * b, c, TypeParam( 1E-6 ) ) );
  EXPECT_FALSE( posesAreNear( b * a, c, TypeParam( 1E-6 ) ) );
  c.translation() = { 3.5019163, 3.33, 1.76487756 };
  c.orientation() = { 0.7074612860298535, 0.38128435209560224, 0.5941814854397381,
                      0.03269761423481789 };
  EXPECT_TRUE( posesAreNear( b * a, c, TypeParam( 1E-6 ) ) );

  Vector3<TypeParam> expected_v = { 7.80816587, 1.66134534, 0.77578394 };
  EXPECT_TRUE( EIGEN_MATRIX_NEAR( c * Vector3<TypeParam>( 1.23, 0, 4.56 ), expected_v, 1E-6 ) );

  Eigen::Matrix<TypeParam, 3, 8> input_m;
  // @formatter:off
  // clang-format off
  input_m << 1.1, 2.2, 0, 4.4,   0,     0, 11.11,     0,
             5.5,   0, 0, 7.7, 8.8,     0,     0, 12.12,
             9.9, 6.6, 0,   0, 3.3, 10.10,     0,     0;
  // @formatter:on
  // clang-format on
  Eigen::Matrix<TypeParam, 3, 8> expected_m;
  // @formatter:off
  // clang-format off
  expected_m << 14.6304527,   9.85710772,     3.50191630,  7.91831853,  9.93876630,  12.2450315,   6.74335275,   8.43281241,
                 2.81214097,  1.12444542,     3.33000000, 10.9719406,   7.90045175,  -1.72637514,  8.87798734,  11.9001274,
                 4.07950485, -0.00912100880,  1.76487756,  2.62866886,  6.86467489,   1.79660379, -7.29851177,   8.77441254;
  // @formatter:on
  // clang-format on
  EXPECT_TRUE( EIGEN_MATRIX_NEAR( c * input_m, expected_m, 1E-6 ) );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
