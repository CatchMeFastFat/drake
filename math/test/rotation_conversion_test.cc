/// @file
/// Tests that rotation conversion functions are inverses.

#include <cmath>
#include <iostream>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/cross_product.h"
#include "drake/math/normalize_vector.h"
#include "drake/math/quaternion.h"
<<<<<<< HEAD
=======
#include "drake/math/roll_pitch_yaw.h"
>>>>>>> intial
#include "drake/math/rotation_matrix.h"

using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::AngleAxis;
using Eigen::AngleAxisd;
using Eigen::Quaternion;
using Eigen::Quaterniond;
using std::sin;
using std::cos;
using std::numeric_limits;

namespace drake {
namespace math {
namespace {

// kSweepSize is an even number, so that no samples are taken at zero. This
// test scales as O(N^4) in sweep size, so be cautious about turning it up!
const int kSweepSize = 6;

<<<<<<< HEAD
const double kEpsilon = numeric_limits<double>::epsilon();
const double kTolerance = 1.0E-12;

GTEST_TEST(EigenEulerAngleTest, MakeXYZRotation) {
  // Verify MakeXRotation(theta), MakeYRotation(theta), MakeZRotation(theta) is
  // the same as AngleAxis equivalents.
  const double theta = 0.1234567;  // Arbitrary angle.
  const RotationMatrixd Rx(RotationMatrixd::MakeXRotation(theta));
  const RotationMatrixd Ry(RotationMatrixd::MakeYRotation(theta));
  const RotationMatrixd Rz(RotationMatrixd::MakeZRotation(theta));
  const Quaterniond qx(Eigen::AngleAxisd(theta, Vector3d::UnitX()));
  const Quaterniond qy(Eigen::AngleAxisd(theta, Vector3d::UnitY()));
  const Quaterniond qz(Eigen::AngleAxisd(theta, Vector3d::UnitZ()));
  const double tolerance = 32 * kEpsilon;
  EXPECT_TRUE(Rx.IsNearlyEqualTo(RotationMatrixd(qx), tolerance).value());
  EXPECT_TRUE(Ry.IsNearlyEqualTo(RotationMatrixd(qy), tolerance).value());
  EXPECT_TRUE(Rz.IsNearlyEqualTo(RotationMatrixd(qz), tolerance).value());
}

GTEST_TEST(EigenEulerAngleTest, BodyXYZ) {
  // Verify ea = Eigen::eulerAngles(0, 1, 2) returns Euler angles about
  // Body-fixed x-y'-z'' axes by [ea(0), ea(1), ea(2)].
  const Vector3d input_angles(0.5, 0.4, 0.3);
  const Matrix3d bodyXYZ_rotmat =
      (RotationMatrix<double>::MakeXRotation(input_angles(0)) *
       RotationMatrix<double>::MakeYRotation(input_angles(1)) *
       RotationMatrix<double>::MakeZRotation(input_angles(2))).matrix();
  const Vector3d output_angles = bodyXYZ_rotmat.eulerAngles(0, 1, 2);
=======
Vector4d EigenQuaternionToOrderWXYZ(const Quaterniond& q) {
  return Vector4d(q.w(), q.x(), q.y(), q.z());
}

bool check_rpy_range(const Vector3d& rpy) {
  return rpy(0) <= M_PI && rpy(0) >= -M_PI && rpy(1) <= M_PI / 2 &&
         rpy(1) >= -M_PI / 2 && rpy(2) <= M_PI && rpy(2) >= -M_PI;
}

bool AreQuaternionsForSameOrientation(const Eigen::Quaterniond& quat1,
                                      const Eigen::Quaterniond& quat2,
                                      double precision = 1E-12) {
  // The same orientation is described by both a quaternion and the negative of
  // that quaternion.
  const Eigen::Vector4d q1(quat1.w(), quat1.x(), quat1.y(), quat1.z());
  const Eigen::Vector4d q2(quat2.w(), quat2.x(), quat2.y(), quat2.z());
  return q1.isApprox(q2, precision) || q1.isApprox(-q2, precision);
}

/// Returns a %Quaternion associated with a Space-fixed (extrinsic) X-Y-Z
/// rotation by "roll-pitch-yaw" angles `[r, p, y]`.
/// @param[in] rpy radian measures of "roll-pitch-yaw" angles.
template <typename T>
Eigen::Quaternion<T> SpaceXYZAnglesToEigenQuaternion(const Vector3<T> rpy) {
  using std::cos;
  using std::sin;
  const T c0 = cos(rpy(0)/2), s0 = sin(rpy(0)/2);
  const T c1 = cos(rpy(1)/2), s1 = sin(rpy(1)/2);
  const T c2 = cos(rpy(2)/2), s2 = sin(rpy(2)/2);

  const T w = c0 * c1 * c2 + s0 * s1 * s2;
  const T x = s0 * c1 * c2 - c0 * s1 * s2;
  const T y = c0 * s1 * c2 + s0 * c1 * s2;
  const T z = c0 * c1 * s2 - s0 * s1 * c2;
  return Eigen::Quaternion<T>(w, x, y, z);
}

bool AreRollPitchYawForSameOrientation(const Vector3d& rpy1,
                                       const Vector3d& rpy2) {
  // Note: When pitch is close to PI/2 or -PI/2, derivative calculations for
  // Euler angle can encounter numerical problems.  However, although values
  // of angles may "jump around" (hence, difficult derivatives), the angles'
  // values should be accurately reproduced.
  const double precision = 1E-13;
  const Eigen::Quaterniond q1 = SpaceXYZAnglesToEigenQuaternion(rpy1);
  const Eigen::Quaterniond q2 = SpaceXYZAnglesToEigenQuaternion(rpy2);
  return AreQuaternionsForSameOrientation(q1, q2, precision);
}

Matrix3d CalcRotationMatrixAboutZ(double a) {
  // Returns 3 x 3 R_AB matrix where vA = R_AB * v_B.
  // vB is a vector expressed in basis B and vA is the equivalent vector, but
  // expressed in basis A.
  // basis B is obtained by rotating basis A about Z axis by angle a.
  Matrix3d ret;
  ret << cos(a), -sin(a), 0, sin(a), cos(a), 0, 0, 0, 1;
  return ret;
}

Matrix3d CalcRotationMatrixAboutY(double b) {
  // Returns 3 x 3 R_AB matrix where vA = R_AB * v_B.
  // vB is a vector expressed in basis B and vA is the equivalent vector, but
  // expressed in basis A.
  // basis B is obtained by rotating basis A about Y axis by angle b.
  Matrix3d ret;
  ret << cos(b), 0, sin(b), 0, 1, 0, -sin(b), 0, cos(b);
  return ret;
}

Matrix3d CalcRotationMatrixAboutX(double c) {
  // Returns 3 x 3 R_AB matrix where vA = R_AB * v_B.
  // vB is a vector expressed in basis B and vA is the equivalent vector, but
  // expressed in basis A.
  // basis B is obtained by rotating basis A about X axis by angle c.
  Matrix3d ret;
  ret << 1, 0, 0, 0, cos(c), -sin(c), 0, sin(c), cos(c);
  return ret;
}
GTEST_TEST(EigenEulerAngleTest, BodyXYZ) {
  // Verify ea = Eigen::eulerAngles(0, 1, 2) returns Euler angles about
  // Body-fixed x-y'-z'' axes by [ea(0), ea(1), ea(2)].
  Vector3d input_angles(0.5, 0.4, 0.3);
  Matrix3d bodyXYZ_rotmat = CalcRotationMatrixAboutX(input_angles(0)) *
                            CalcRotationMatrixAboutY(input_angles(1)) *
                            CalcRotationMatrixAboutZ(input_angles(2));
  auto output_angles = bodyXYZ_rotmat.eulerAngles(0, 1, 2);
>>>>>>> intial
  // input_angles.isApprox(output_angles) is a valid test (rathan than
  // comparing the converted quaternions) since all the angles are between
  // 0 and PI/2.
  EXPECT_TRUE(input_angles.isApprox(output_angles));
}

GTEST_TEST(EigenEulerAngleTest, SpaceXYZ) {
  // Verify ea = Eigen::eulerAngles(2, 1, 0) returns Euler angles about
  // Body-fixed z-y'-x'' axes by [ea(0), ea(1), ea(2)].
  const Vector3d input_angles(0.5, 0.4, 0.3);
<<<<<<< HEAD
  const Matrix3d spaceXYZ_rotmat =
      (RotationMatrix<double>::MakeZRotation(input_angles(0)) *
       RotationMatrix<double>::MakeYRotation(input_angles(1)) *
       RotationMatrix<double>::MakeXRotation(input_angles(2))).matrix();
  const Vector3d output_angles = spaceXYZ_rotmat.eulerAngles(2, 1, 0);
=======
  const Matrix3d spaceXYZ_rotmat = CalcRotationMatrixAboutZ(input_angles(0)) *
                                   CalcRotationMatrixAboutY(input_angles(1)) *
                                   CalcRotationMatrixAboutX(input_angles(2));
  auto output_angles = spaceXYZ_rotmat.eulerAngles(2, 1, 0);
>>>>>>> intial
  // input_angles.isApprox(output_angles) is a valid test (rathan than
  // comparing the converted quaternions) since all the angles are between
  // 0 and PI/2.
  EXPECT_TRUE(input_angles.isApprox(output_angles));
}

GTEST_TEST(EigenEulerAngleTest, BodyZYZ) {
  // Verify ea = Eigen::eulerAngles(2, 1, 0) returns Euler angles about
  // Body-fixed z-y'-z'' axes by [ea(0), ea(1), ea(2)].
<<<<<<< HEAD
  const Vector3d input_angles(0.5, 0.4, 0.3);
  const Matrix3d bodyZYZ_angles =
      (RotationMatrix<double>::MakeZRotation(input_angles(0)) *
       RotationMatrix<double>::MakeYRotation(input_angles(1)) *
       RotationMatrix<double>::MakeZRotation(input_angles(2))).matrix();
  const Vector3d output_angles = bodyZYZ_angles.eulerAngles(2, 1, 2);
=======
  Vector3d input_angles(0.5, 0.4, 0.3);
  Matrix3d bodyZYZ_angles = CalcRotationMatrixAboutZ(input_angles(0)) *
                            CalcRotationMatrixAboutY(input_angles(1)) *
                            CalcRotationMatrixAboutZ(input_angles(2));
  auto output_angles = bodyZYZ_angles.eulerAngles(2, 1, 2);
>>>>>>> intial
  // input_angles.isApprox(output_angles) is a valid test (rathan than
  // comparing the converted quaternions) since all the angles are between
  // 0 and PI/2.
  EXPECT_TRUE(input_angles.isApprox(output_angles));
}
<<<<<<< HEAD

=======
>>>>>>> intial
class RotationConversionTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  void SetUp() override {
    SetupRPYTestCases();
    SetupAngleAxisTestCases();
    SetupQuaternionTestCases();
    SetupRotationMatrixTestCases();
  }

  void SetupRPYTestCases() {
    // Set up a variety of specific tests for angles that may cause numerical
    // problems as well as a sweep of values to test general functionality.
    // Singularity issue associated with the second angle = pi/2
    // in Euler Body-fixed z-y'-x'' rotation sequence.
    // Singularity issue associated with the second angle = -pi/2
    // in Euler Body-fixed z-y'-x'' rotation sequence.
    // Singularity issue associated with the second angle close to pi/2
    // in Euler Body-fixed z-y'-x'' rotation sequence.
    // Singularity issue associated with the second angle close to -pi/2
    // in Euler Body-fixed z-y'-x'' rotation sequence.

    // pitch = pi/2
<<<<<<< HEAD
    const RollPitchYaw<double> pitch_half_pi(M_PI / 4, M_PI / 2, M_PI / 3);
    rpy_test_cases_.push_back(pitch_half_pi);

    // pitch = -pi/2
    const RollPitchYaw<double> pitch_neg_half_pi(M_PI / 4, -M_PI / 2, M_PI / 3);
    rpy_test_cases_.push_back(pitch_neg_half_pi);

    // pitch = 0.5*pi-eps
    const RollPitchYaw<double> pitch_near_half_piA(
        M_PI / 4, 0.5 * M_PI - kEpsilon, M_PI / 3);
    rpy_test_cases_.push_back(pitch_near_half_piA);

    // pitch = 0.5*pi-1.5*eps
    const RollPitchYaw<double> pitch_near_half_piB(
        M_PI / 4, 0.5 * M_PI - 1.5 * kEpsilon, M_PI / 3);
    rpy_test_cases_.push_back(pitch_near_half_piB);

    // pitch = 0.5*pi-2*eps
    const RollPitchYaw<double> pitch_near_half_piC(
        M_PI / 4, 0.5 * M_PI - 2 * kEpsilon, M_PI / 3);
    rpy_test_cases_.push_back(pitch_near_half_piC);

    // pitch = 0.5*pi - 1E-15
    const RollPitchYaw<double> pitch_near_half_piD(
        M_PI * 0.8, 0.5 * M_PI - 1E-15, 0.9 * M_PI);
    rpy_test_cases_.push_back(pitch_near_half_piD);

    // pitch = -0.5*pi+eps
    const RollPitchYaw<double> pitch_near_neg_half_piA(
        M_PI * -0.9, -0.5 * M_PI + kEpsilon, M_PI * 0.3);
    rpy_test_cases_.push_back(pitch_near_neg_half_piA);

    // pitch = -0.5*pi+1.5*eps
    const RollPitchYaw<double> pitch_near_neg_half_piB(
        M_PI * -0.6, -0.5 * M_PI + 1.5 * kEpsilon, M_PI * 0.3);
    rpy_test_cases_.push_back(pitch_near_neg_half_piB);

    // pitch = -0.5*pi+2*eps
    const RollPitchYaw<double> pitch_near_neg_half_piC(
        M_PI * -0.5, -0.5 * M_PI + 2 * kEpsilon, M_PI * 0.4);
    rpy_test_cases_.push_back(pitch_near_neg_half_piC);

    // pitch = -0.5*pi + 1E-15
    const RollPitchYaw<double> pitch_near_neg_half_piD(
        M_PI * 0.9, -0.5 * M_PI + 1E-15, 0.8 * M_PI);
    rpy_test_cases_.push_back(pitch_near_neg_half_piD);
=======
    rpy_test_cases_.push_back(Vector3d(M_PI / 4, M_PI / 2, M_PI / 3));

    // pitch = -pi/2
    rpy_test_cases_.push_back(Vector3d(M_PI / 4, -M_PI / 2, M_PI / 3));

    // pitch = 0.5*pi-eps
    rpy_test_cases_.push_back(Vector3d(
        M_PI / 4, 0.5 * M_PI - numeric_limits<double>::epsilon(), M_PI / 3));

    // pitch = 0.5*pi-1.5*eps
    rpy_test_cases_.push_back(
        Vector3d(M_PI / 4, 0.5 * M_PI - 1.5 * numeric_limits<double>::epsilon(),
                 M_PI / 3));

    // pitch = 0.5*pi-2*eps
    rpy_test_cases_.push_back(
        Vector3d(M_PI / 4, 0.5 * M_PI - 2 * numeric_limits<double>::epsilon(),
                 M_PI / 3));

    // pitch = 0.5*pi - 1E-15
    rpy_test_cases_.push_back(
        Vector3d(M_PI * 0.8, 0.5 * M_PI - 1E-15, 0.9 * M_PI));

    // pitch = -0.5*pi+eps
    rpy_test_cases_.push_back(
        Vector3d(M_PI * -0.9, -0.5 * M_PI + numeric_limits<double>::epsilon(),
                 M_PI * 0.3));

    // pitch = -0.5*pi+1.5*eps
    rpy_test_cases_.push_back(Vector3d(
        M_PI * -0.6, -0.5 * M_PI + 1.5 * numeric_limits<double>::epsilon(),
        M_PI * 0.3));

    // pitch = -0.5*pi+2*eps
    rpy_test_cases_.push_back(Vector3d(
        M_PI * -0.5, -0.5 * M_PI + 2 * numeric_limits<double>::epsilon(),
        M_PI * 0.4));

    // pitch = -0.5*pi + 1E-15
    rpy_test_cases_.push_back(
        Vector3d(M_PI * 0.9, -0.5 * M_PI + 1E-15, 0.8 * M_PI));
>>>>>>> intial

    // non-singular cases
    auto roll = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize,
                                           -0.99 * M_PI, M_PI);
    auto pitch = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize,
                                            -0.49 * M_PI, 0.49 * M_PI);
    auto yaw = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize,
                                          -0.99 * M_PI, M_PI);
    for (int i = 0; i < roll.size(); ++i) {
      for (int j = 0; j < pitch.size(); ++j) {
        for (int k = 0; k < yaw.size(); ++k) {
<<<<<<< HEAD
          const RollPitchYaw<double> rpy(roll(i), pitch(j), yaw(k));
          rpy_test_cases_.push_back(rpy);
=======
          rpy_test_cases_.push_back(Vector3d(roll(i), pitch(j), yaw(k)));
>>>>>>> intial
        }
      }
    }
  }

  void addAngleAxisTestCase(double angle, const Vector3d& axis) {
    angle_axis_test_cases_.push_back(AngleAxisd(angle, axis));
  }

  void SetupAngleAxisTestCases() {
    // Set up a variety of specific tests for angles/axes that may cause
    // numerical problems as well as a sweep of values to test general
    // functionality.
    // Degenerate case, 0 rotation around x axis
    // Degenerate case, 0 rotation around y axis
    // Degenerate case, 0 rotation around z axis
    // Degenerate case, 0 rotation around a unit axis
    // Almost degenerate case, small positive rotation around an arbitrary axis
    // Almost degenerate case, small negative rotation around an arbitrary axis
    // Differentiation issue at 180 rotation around x axis
    // Differentiation issue at 180 rotation around y axis
    // Differentiation issue at 180 rotation around z axis
    // Differentiation issue at 180 rotation around an arbitrary unit axis
    // Differentiation issue close to 180 rotation around an arbitrary axis

    // 0 rotation around x axis
    addAngleAxisTestCase(0, Vector3d::UnitX());

    // 0 rotation around y axis
    addAngleAxisTestCase(0, Vector3d::UnitY());

    // 0 rotation around z axis
    addAngleAxisTestCase(0, Vector3d::UnitZ());

    // 0 rotation around an arbitrary axis
    Vector3d axis(0.5 * sqrt(2), 0.4 * sqrt(2), 0.3 * sqrt(2));
    addAngleAxisTestCase(0, axis);

    // epsilon rotation around an arbitrary axis
<<<<<<< HEAD
    addAngleAxisTestCase(kEpsilon, axis);
=======
    addAngleAxisTestCase(numeric_limits<double>::epsilon(), axis);
>>>>>>> intial

    // 1E-10 rotation around an arbitrary axis
    addAngleAxisTestCase(1E-10, axis);

<<<<<<< HEAD
    // -epsilon rotation around an arbitrary axis
    addAngleAxisTestCase(-kEpsilon, axis);
=======
    // -epsilon rotation around an arbitary axis
    addAngleAxisTestCase(-numeric_limits<double>::epsilon(), axis);
>>>>>>> intial

    // -1E-10 rotation around an arbitrary axis
    addAngleAxisTestCase(-1E-10, axis);

    // 180 rotation around x axis
    addAngleAxisTestCase(M_PI, Vector3d::UnitX());

    // 180 rotation around y axis
    addAngleAxisTestCase(M_PI, Vector3d::UnitY());

    // 180 rotation around z axis
    addAngleAxisTestCase(M_PI, Vector3d::UnitZ());

    // -180 rotation around x axis
    addAngleAxisTestCase(-M_PI, Vector3d::UnitX());

    // -180 rotation around y axis
    addAngleAxisTestCase(-M_PI, Vector3d::UnitY());

    // -180 rotation around z axis
    addAngleAxisTestCase(-M_PI, Vector3d::UnitZ());

<<<<<<< HEAD
    // 180 rotation around an arbitrary axis
    addAngleAxisTestCase(M_PI, axis);

    // -180 rotation around an arbitrary axis
    addAngleAxisTestCase(-M_PI, axis);

    // (1-epsilon)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((1 - kEpsilon) * M_PI, axis);

    // (-1+epsilon)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((-1 + kEpsilon) * M_PI, axis);

    // (1-2*epsilon)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((1 - 2 * kEpsilon) * M_PI, axis);

    // (-1+2*epsilon)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((-1 + 2 * kEpsilon) * M_PI, axis);
=======
    // 180 rotation around an arbitary axis
    addAngleAxisTestCase(M_PI, axis);

    // -180 rotation around an arbitary axis
    addAngleAxisTestCase(-M_PI, axis);

    // (1-epsilon)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((1 - numeric_limits<double>::epsilon()) * M_PI, axis);

    // (-1+epsilon)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((-1 + numeric_limits<double>::epsilon()) * M_PI, axis);

    // (1-2*epsilon)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((1 - 2 * numeric_limits<double>::epsilon()) * M_PI,
                         axis);

    // (-1+2*epsilon)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((-1 + 2 * numeric_limits<double>::epsilon()) * M_PI,
                         axis);
>>>>>>> intial

    // (1-1E-10)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((1 - 1E-10) * M_PI, axis);

    // (-1+1E-10)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((-1 + 1E-10) * M_PI, axis);

    // non-singularity cases
    auto a_x = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize, -1, 1);
    auto a_y = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize, -1, 1);
    auto a_z = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize, -1, 1);
    auto a_angle = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize,
                                              -0.95 * M_PI, 0.95 * M_PI);
    for (int i = 0; i < a_x.size(); ++i) {
      for (int j = 0; j < a_y.size(); ++j) {
        for (int k = 0; k < a_z.size(); ++k) {
          Vector3d axis_ijk(a_x(i), a_y(j), a_z(k));
          if (axis_ijk.norm() > 1E-3) {
            axis_ijk.normalize();
            for (int l = 0; l < a_angle.size(); ++l) {
              addAngleAxisTestCase(a_angle(l), axis_ijk);
            }
          }
        }
      }
    }
  }

  void SetupQuaternionTestCases() {
    // Set up a variety of general tests for quaternions.
    auto qw = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize, -1, 1);
    auto qx = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize, -1, 1);
    auto qy = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize, -1, 1);
    auto qz = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize, -1, 1);
    for (int i = 0; i < qw.size(); ++i) {
      for (int j = 0; j < qx.size(); ++j) {
        for (int k = 0; k < qy.size(); ++k) {
          for (int l = 0; l < qz.size(); ++l) {
            Vector4d q(qw(i), qx(j), qy(k), qz(l));
            if (q.norm() > 1E-3) {
              q.normalize();
              quaternion_test_cases_.push_back(
                  Quaterniond(q(0), q(1), q(2), q(3)));
            }
          }
        }
      }
    }
  }

  void SetupRotationMatrixTestCases() {
<<<<<<< HEAD
    for (const RollPitchYaw<double>& rpyi : rpy_test_cases_) {
      const RotationMatrix<double> Ri(rpyi);
      rotation_matrix_test_cases_.push_back(Ri);
    }
    for (const Eigen::AngleAxisd& ai : angle_axis_test_cases_) {
      const RotationMatrix<double> Ri(ai);
      rotation_matrix_test_cases_.push_back(Ri);
    }
    for (const Quaterniond& qi : quaternion_test_cases_) {
      const RotationMatrix<double> Ri(qi);
      rotation_matrix_test_cases_.push_back(Ri);
    }
  }
  std::vector<RollPitchYaw<double>> rpy_test_cases_;
  std::vector<AngleAxisd> angle_axis_test_cases_;
  std::vector<Quaterniond> quaternion_test_cases_;
  std::vector<RotationMatrix<double>> rotation_matrix_test_cases_;
};

TEST_F(RotationConversionTest, quat2RotmatTest) {
  for (const Quaterniond& qi : quaternion_test_cases_) {
    // Compute the rotation matrix using Eigen geometry module, compare the
    // result with RotationMatrix(quaternion).
    const Matrix3d rotmat_expected = qi.toRotationMatrix();
    const Matrix3d rotmat = RotationMatrix<double>(qi).matrix();
    EXPECT_TRUE(CompareMatrices(rotmat_expected, rotmat, 1E-10,
                                MatrixCompareType::absolute));
    // RotationMatrix(quaternion) is inverse of RotationMatrix::ToQuaternion().
    const Eigen::Quaterniond quat_expected =
        RotationMatrix<double>::ToQuaternion(rotmat);
    EXPECT_TRUE(
        AreQuaternionsEqualForOrientation(qi, quat_expected, kTolerance));
=======
    for (const Vector3d& rpyi : rpy_test_cases_) {
      rotation_matrix_test_cases_.push_back(rpy2rotmat(rpyi));
    }
    for (const Eigen::AngleAxisd& ai : angle_axis_test_cases_) {
      const RotationMatrix<double> Ri(ai);
      rotation_matrix_test_cases_.push_back(Ri.matrix());
    }
    for (const Quaterniond& qi : quaternion_test_cases_) {
      auto q = EigenQuaternionToOrderWXYZ((qi));
      auto R = quat2rotmat(q);
      rotation_matrix_test_cases_.push_back(R);
    }
  }
  std::vector<Vector3d> rpy_test_cases_;
  std::vector<AngleAxisd> angle_axis_test_cases_;
  std::vector<Quaterniond> quaternion_test_cases_;
  std::vector<Matrix3d> rotation_matrix_test_cases_;
};

TEST_F(RotationConversionTest, QuatRotmat) {
  for (const Quaterniond& qi : quaternion_test_cases_) {
    // Compute the rotation matrix using Eigen geometry module, compare the
    // result with quat2rotmat
    const Matrix3d rotmat_expected = qi.toRotationMatrix();
    const Matrix3d rotmat = quat2rotmat(EigenQuaternionToOrderWXYZ(qi));
    EXPECT_TRUE(CompareMatrices(rotmat_expected, rotmat, 1E-10,
                                MatrixCompareType::absolute));
    // quat2rotmat should be the inversion of ToQuaternion().
    const Eigen::Quaterniond quat_expected =
        RotationMatrix<double>::ToQuaternion(rotmat);
    EXPECT_TRUE(AreQuaternionsForSameOrientation(qi, quat_expected));
>>>>>>> intial
  }
}

TEST_F(RotationConversionTest, QuatRPY) {
  for (const Quaterniond& qi : quaternion_test_cases_) {
<<<<<<< HEAD
    const RollPitchYaw<double> rpy(qi);
    const Eigen::Quaterniond q_expected = rpy.ToQuaternion();
    // Test RollPitchYaw::ToQuaternion() is inverse of RollPitchYaw(quaternion).
    EXPECT_TRUE(AreQuaternionsEqualForOrientation(qi, q_expected, kTolerance));
    EXPECT_TRUE(rpy.IsRollPitchYawInCanonicalRange());
  }
}

TEST_F(RotationConversionTest, RotmatQuat) {
  // Compare Eigen's rotation matrix to quaternion result with the result from
  // RotationMatrix::ToQuaternion().
  for (const RotationMatrix<double>& Ri : rotation_matrix_test_cases_) {
    const Eigen::Quaterniond quat_drake = Ri.ToQuaternion();
    const Eigen::Quaterniond quat_eigen = Quaterniond(Ri.matrix());
    EXPECT_TRUE(
        AreQuaternionsEqualForOrientation(quat_drake, quat_eigen, kTolerance));
    // Ensure the calculated quaternion produces the same rotation matrix.
    // This test accuracy to near machine precision and uses a tolerance of
    // 32 * kEpsilon (allows for 5 of the 53 mantissa bits to be inaccurate).
    // This 5-bit estimate seems to be a reasonably tight bound which
    // nevertheless passes a representative sampling of compilers and platforms.
    const RotationMatrix<double> rotmat(quat_drake);
    EXPECT_TRUE(Ri.IsNearlyEqualTo(rotmat, 32 * kEpsilon).value());
  }
}

TEST_F(RotationConversionTest, rotmat2rpyTest) {
  for (const RotationMatrix<double> Ri : rotation_matrix_test_cases_) {
    const RollPitchYaw<double> rpy(Ri);
    const RotationMatrix<double> rotmat_expected(rpy);
    // RollPitchYaw(RotationMatrix) is inverse of RotationMatrix(RollPitchYaw).
    EXPECT_TRUE(Ri.IsNearlyEqualTo(rotmat_expected, 256 * kEpsilon).value());
    EXPECT_TRUE(rpy.IsRollPitchYawInCanonicalRange());
  }
}

TEST_F(RotationConversionTest, rpy2rotmatTest) {
  for (const RollPitchYaw<double>& rpyi : rpy_test_cases_) {
    const double roll = rpyi.roll_angle();
    const double pitch = rpyi.pitch_angle();
    const double yaw = rpyi.yaw_angle();
    const Quaterniond q = Eigen::AngleAxisd(yaw, Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(pitch, Vector3d::UnitY()) *
                          Eigen::AngleAxisd(roll, Vector3d::UnitX());
    const RotationMatrix<double> R_from_quaternion(q);

    // Compute rotation matrix by rotz(rpy(2))*roty(rpy(1))*rotx(rpy(0)),
    // then compare the result with RotationMatrix(RollPitchYaw).
    const RotationMatrix<double> R_from_rpy(rpyi);
    EXPECT_TRUE(
        R_from_rpy.IsNearlyEqualTo(R_from_quaternion, 512 * kEpsilon).value());

    // RollPitchYaw(RotationMatrix) is inverse of RotationMatrix(RollPitchYaw).
    const RollPitchYaw<double> rpy_expected(R_from_rpy);
    EXPECT_TRUE(rpyi.IsNearlySameOrientation(rpy_expected, kTolerance));
  }
}

TEST_F(RotationConversionTest, rpy2QuatTest) {
  for (const RollPitchYaw<double>& rpyi : rpy_test_cases_) {
    const Eigen::Quaterniond q = rpyi.ToQuaternion();
    // Verify rpyi.ToQuaternion() is inverse of RollPitchYaw(Quaternion).
    const RollPitchYaw<double> rpy_expected(q);
    EXPECT_TRUE(rpyi.IsNearlySameOrientation(rpy_expected, 512 * kEpsilon));
=======
    const Vector3d rpy = QuaternionToSpaceXYZ(qi);
    // rpy2quat should be the inversion of QuaternionToSpaceXYZ().
    const Vector4d quat4 = rpy2quat(rpy);
    const Eigen::Quaterniond q_expected(quat4(0), quat4(1), quat4(2), quat4(3));
    EXPECT_TRUE(AreQuaternionsForSameOrientation(qi, q_expected));
    EXPECT_TRUE(check_rpy_range(rpy));
  }
}

TEST_F(RotationConversionTest, QuatEigenQuaternion) {
  for (const Quaterniond& qi_eigen : quaternion_test_cases_) {
    Vector4d qi = EigenQuaternionToOrderWXYZ(qi_eigen);
    Quaterniond eigenQuat = quat2eigenQuaternion(qi);
    Matrix3d R_expected = quat2rotmat(qi);
    Matrix3d R_eigen = eigenQuat.matrix();
    EXPECT_TRUE(CompareMatrices(R_expected, R_eigen, 1e-6,
                                MatrixCompareType::absolute));
  }
}
TEST_F(RotationConversionTest, RotmatQuat) {
  // Compare Eigen's rotation matrix to quaternion result with the result from
  // RotationMatrix::ToQuaternion().
  for (const auto& Ri : rotation_matrix_test_cases_) {
    const Eigen::Quaterniond quat = RotationMatrix<double>::ToQuaternion(Ri);
    const Eigen::Quaterniond quat_expected = Quaterniond(Ri);
    EXPECT_TRUE(AreQuaternionsForSameOrientation(quat, quat_expected));
    // Ensure the calculated quaternion produces the same rotation matrix.
    const RotationMatrix<double> rotmat(quat);
    EXPECT_TRUE(Ri.isApprox(rotmat.matrix()));
  }
}

TEST_F(RotationConversionTest, RotmatRPY) {
  for (const auto& Ri : rotation_matrix_test_cases_) {
    Vector3d rpy = rotmat2rpy(Ri);
    // rotmat2rpy should be the inversion of rpy2rotmat
    Matrix3d rotmat_expected = rpy2rotmat(rpy);
    EXPECT_TRUE(Ri.isApprox(rotmat_expected, 1E-10));
    EXPECT_TRUE(check_rpy_range(rpy));
  }
}

TEST_F(RotationConversionTest, RPYRotmat) {
  // Compute the rotation matrix by rotz(rpy(2))*roty(rpy(1))*rotx(rpy(0)),
  // then compare the result with rpy2rotmat
  for (const Vector3d& rpyi : rpy_test_cases_) {
    const Quaterniond q = Eigen::AngleAxisd(rpyi(2), Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(rpyi(1), Vector3d::UnitY()) *
                          Eigen::AngleAxisd(rpyi(0), Vector3d::UnitX());
    Matrix3d rotmat = rpy2rotmat(rpyi);
    EXPECT_TRUE(CompareMatrices(rotmat, q.toRotationMatrix(),
                                1E-10, MatrixCompareType::absolute));
    // rpy2rotmat should be the inversion of rotmat2rpy
    Vector3d rpy_expected = rotmat2rpy(rotmat);
    EXPECT_TRUE(AreRollPitchYawForSameOrientation(rpyi, rpy_expected));
  }
}

// Verifies the correctness of drake::math::QuaternionToSpaceXYZ() by comparing
// its output to a quaternion obtained from SpaceXYZAnglesToEigenQuaternion().
TEST_F(RotationConversionTest, RPYQuat) {
  for (const Vector3d& rpyi : rpy_test_cases_) {
    const Eigen::Quaterniond q = SpaceXYZAnglesToEigenQuaternion(rpyi);
    const Vector4d quat4 = rpy2quat(rpyi);
    const Eigen::Quaterniond quat(quat4(0), quat4(1), quat4(2), quat4(3));
    EXPECT_TRUE(AreQuaternionsForSameOrientation(quat, q));
    // QuaternionToSpaceXYZ() should be the inversion of rpy2quat.
    const Vector3d rpy_expected = QuaternionToSpaceXYZ(q);
    EXPECT_TRUE(AreRollPitchYawForSameOrientation(rpyi, rpy_expected));
    EXPECT_TRUE(
        AreRollPitchYawForSameOrientation(rpyi, rotmat2rpy(rpy2rotmat(rpyi))));
  }
}

// Verifies the correctness of the method
// drake::math::RollPitchYawToQuaternion() by comparing its output to a
// quaternion obtained in the local function SpaceXYZAnglesToEigenQuaternion().
TEST_F(RotationConversionTest, RollPitchYawToQuaternion) {
  // Compute the quaternion representation using Eigen's geometry model,
  // compare the result with rpy2quat
  for (const Vector3d& rpyi : rpy_test_cases_) {
    const Vector3d spaceXYZ_angles(rpyi(0), rpyi(1), rpyi(2));
    auto quat_expected = SpaceXYZAnglesToEigenQuaternion(spaceXYZ_angles);
    auto quat = RollPitchYawToQuaternion(rpyi);
    EXPECT_TRUE(
        quat.isApprox(quat_expected, Eigen::NumTraits<double>::epsilon()));
>>>>>>> intial
  }
}

}  // namespace
}  // namespace math
}  // namespace drake
<<<<<<< HEAD

=======
>>>>>>> intial
