/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

#include <dart/simulation/experimental/detail/variational/discrete_mechanics_math.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <vector>

namespace {

using namespace dart::simulation::experimental::detail::variational;

Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

// ad as a 6x6 matrix in the [angular; linear] convention (matches motionCross).
Matrix6 adMatrix(const Vector6& v)
{
  Matrix6 m = Matrix6::Zero();
  m.topLeftCorner<3, 3>() = skew(v.head<3>());
  m.bottomLeftCorner<3, 3>() = skew(v.tail<3>());
  m.bottomRightCorner<3, 3>() = skew(v.head<3>());
  return m;
}

// Ad_T as a 6x6 matrix: [[R, 0], [skew(p) R, R]].
Matrix6 adjointMatrix(const Eigen::Isometry3d& t)
{
  Matrix6 m = Matrix6::Zero();
  m.topLeftCorner<3, 3>() = t.linear();
  m.bottomLeftCorner<3, 3>() = skew(t.translation()) * t.linear();
  m.bottomRightCorner<3, 3>() = t.linear();
  return m;
}

std::vector<Vector6> sampleTwists()
{
  std::vector<Vector6> v;
  v.push_back((Vector6() << 0, 0, 0, 0, 0, 0).finished());
  v.push_back((Vector6() << 0.3, -0.2, 0.1, 0.5, -0.4, 0.2).finished());
  v.push_back((Vector6() << 0.0, 0.0, 0.0, 1.0, 2.0, -3.0)
                  .finished()); // pure translation
  v.push_back((Vector6() << 1.2, -0.7, 0.4, -0.3, 0.6, 0.9).finished());
  v.push_back((Vector6() << 0.05, 0.0, 0.0, 0.0, 0.0, 0.0)
                  .finished()); // small rotation
  return v;
}

} // namespace

TEST(DiscreteMechanicsMath, ZeroTwistIdentities)
{
  const Vector6 zero = Vector6::Zero();
  const Vector6 w = (Vector6() << 1, 2, 3, 4, 5, 6).finished();

  EXPECT_TRUE(dexpInv(zero, w).isApprox(w));
  EXPECT_TRUE(dexpInvTranspose(zero, w).isApprox(w));
  EXPECT_TRUE(se3Exp(zero).isApprox(Eigen::Isometry3d::Identity()));
  EXPECT_LT(se3Log(Eigen::Isometry3d::Identity()).norm(), 1e-12);
}

TEST(DiscreteMechanicsMath, ExpLogRoundTrip)
{
  for (const auto& xi : sampleTwists()) {
    const Eigen::Isometry3d t = se3Exp(xi);
    const Vector6 recovered = se3Log(t);
    EXPECT_TRUE(recovered.isApprox(xi, 1e-9))
        << "xi=" << xi.transpose() << " recovered=" << recovered.transpose();
    // Exp(Log(T)) == T.
    EXPECT_TRUE(se3Exp(recovered).isApprox(t, 1e-9));
  }
}

TEST(DiscreteMechanicsMath, LogMapHandlesNearPiRotation)
{
  // Rotation by exactly pi about a tilted axis exercises the theta->pi branch.
  const Eigen::Vector3d axis = Eigen::Vector3d(1.0, 0.5, -0.3).normalized();
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = Eigen::AngleAxisd(kPi, axis).toRotationMatrix();
  t.translation() = Eigen::Vector3d(0.2, -0.1, 0.4);

  const Vector6 xi = se3Log(t);
  // The recovered rotation must reproduce T's rotation (axis sign may flip at
  // pi).
  const Eigen::Isometry3d rebuilt = se3Exp(xi);
  EXPECT_TRUE(rebuilt.linear().isApprox(t.linear(), 1e-6))
      << "rebuilt=\n"
      << rebuilt.linear() << "\nexpected=\n"
      << t.linear();
  EXPECT_NEAR(xi.head<3>().norm(), kPi, 1e-6);
}

TEST(DiscreteMechanicsMath, AppliedFormsMatchMatrixForms)
{
  // dexp^{-1}/dexp^{-T} delegate to the exact SE3 left-Jacobian inverse and are
  // validated in test_discrete_mechanics_lie_group_parity; here we pin the
  // Lie-bracket applied forms against their matrix forms.
  const Vector6 w = (Vector6() << 0.7, -1.1, 0.5, 2.0, -0.3, 1.4).finished();
  for (const auto& v : sampleTwists()) {
    const Matrix6 a = adMatrix(v);
    const Matrix6 dadM = a.transpose();
    // ad applied form vs matrix.
    EXPECT_TRUE(ad(v, w).isApprox(a * w, 1e-12));
    // dad applied form vs transposed matrix.
    EXPECT_TRUE(dad(v, w).isApprox(dadM * w, 1e-12));
  }
}

TEST(DiscreteMechanicsMath, DexpInverseIsInverseOfDexp)
{
  // For small twists, dexpInv(V) * dexp(V) ~ I to the series truncation order.
  // dexp(V) = sum_{j>=0} ad_V^j / (j+1)!  (right-trivialized tangent of exp).
  const std::vector<Vector6> small
      = {(Vector6() << 0.04, -0.03, 0.02, 0.05, -0.01, 0.03).finished(),
         (Vector6() << 0.0, 0.06, 0.0, -0.02, 0.0, 0.04).finished()};
  for (const auto& v : small) {
    const Matrix6 a = adMatrix(v);
    const Matrix6 dexpM = Matrix6::Identity() + a / 2.0 + a * a / 6.0
                          + a * a * a / 24.0 + a * a * a * a / 120.0;
    const Matrix6 dexpInvM
        = Matrix6::Identity() - a / 2.0 + a * a / 12.0 - a * a * a * a / 720.0;
    EXPECT_LT((dexpInvM * dexpM - Matrix6::Identity()).norm(), 1e-6);
  }
}

TEST(DiscreteMechanicsMath, DualAdjointsMatchMatrixForms)
{
  const Vector6 f = (Vector6() << 1.0, -2.0, 0.5, 0.3, 0.7, -1.2).finished();
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear()
      = Eigen::AngleAxisd(0.6, Eigen::Vector3d(0.2, 0.9, -0.3).normalized())
            .toRotationMatrix();
  t.translation() = Eigen::Vector3d(0.4, -0.5, 0.8);

  // dAdT(T,F) == Ad_T^T F.
  EXPECT_TRUE(dAdT(t, f).isApprox(adjointMatrix(t).transpose() * f, 1e-10));
  // dAdInvT(T,F) == Ad_{T^-1}^T F.
  EXPECT_TRUE(dAdInvT(t, f).isApprox(
      adjointMatrix(t.inverse()).transpose() * f, 1e-10));
  // The two are mutual inverses: dAdInvT(T, dAdT(T, F)) == F.
  EXPECT_TRUE(dAdInvT(t, dAdT(t, f)).isApprox(f, 1e-10));
}

TEST(DiscreteMechanicsMath, AdInvRLinearRotatesIntoBody)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear()
      = Eigen::AngleAxisd(0.9, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d g(0.0, 0.0, -9.81);
  const Vector6 result = adInvRLinear(t, g);
  EXPECT_LT(result.head<3>().norm(), 1e-12);
  EXPECT_TRUE(result.tail<3>().isApprox(t.linear().transpose() * g, 1e-12));
}
