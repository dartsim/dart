/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

// Cross-validates the variational integrator's SE(3) discrete-mechanics kernels
// against DART's shared dart::math::lie_group library. This both documents that
// the discrete-mechanics math is "in common" with the general Lie-group API and
// guards the equivalence: the inverse right-trivialized tangent dexp^{-1} is
// the left-Jacobian inverse SE3::LeftJacobianInverse (the exact Barfoot-Furgale
// closed form), and the ported exp/log/ad agree with the library.

#include <dart/simulation/experimental/detail/variational/discrete_mechanics_math.hpp>

#include <dart/math/lie_group/se3.hpp>
#include <dart/math/lie_group/se3_tangent.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <vector>

namespace {

namespace dm = dart::simulation::experimental::detail::variational;
using dm::Matrix6;
using dm::Vector6;
using SE3 = dart::math::SE3<double>;
using SE3Tangent = dart::math::SE3Tangent<double>;

SE3Tangent makeTangent(const Vector6& xi)
{
  return SE3Tangent(
      Eigen::Vector3d(xi.head<3>()), Eigen::Vector3d(xi.tail<3>()));
}

std::vector<Vector6> sampleTwists()
{
  return {
      (Vector6() << 0.2, -0.15, 0.1, 0.3, -0.2, 0.25).finished(),
      (Vector6() << 0.0, 0.0, 0.0, 1.0, -2.0, 0.5).finished(),
      (Vector6() << 0.6, 0.0, 0.0, 0.0, 0.4, -0.3).finished(),
      (Vector6() << 0.05, -0.02, 0.03, 0.01, 0.0, -0.04).finished()};
}

} // namespace

TEST(DiscreteMechanicsLieGroupParity, ExpMatchesSE3Tangent)
{
  for (const auto& xi : sampleTwists()) {
    const Eigen::Isometry3d mine = dm::se3Exp(xi);
    const Eigen::Isometry3d lib = makeTangent(xi).exp().toIsometry3();
    EXPECT_TRUE(mine.isApprox(lib, 1e-9)) << "xi=" << xi.transpose();
  }
}

TEST(DiscreteMechanicsLieGroupParity, LogMatchesSE3)
{
  for (const auto& xi : sampleTwists()) {
    const SE3 g = makeTangent(xi).exp();
    const Vector6 mine = dm::se3Log(g.toIsometry3());
    const Vector6 lib = g.log().params();
    EXPECT_TRUE(mine.isApprox(lib, 1e-9))
        << "mine=" << mine.transpose() << " lib=" << lib.transpose();
  }
}

TEST(DiscreteMechanicsLieGroupParity, AdMatchesSE3TangentBracket)
{
  const Vector6 w = (Vector6() << 0.7, -0.3, 0.5, 0.2, 0.9, -0.4).finished();
  for (const auto& xi : sampleTwists()) {
    const Vector6 mine = dm::ad(xi, w);
    const Vector6 lib = makeTangent(xi).ad(makeTangent(w)).params();
    EXPECT_TRUE(mine.isApprox(lib, 1e-12))
        << "mine=" << mine.transpose() << " lib=" << lib.transpose();
  }
}

// The discrete-mechanics dexp^{-1} (truncated Bernoulli series) is the
// left-Jacobian inverse; it agrees with the library's exact closed form to the
// 4th-order truncation error, and is clearly the LEFT (not RIGHT) inverse.
TEST(DiscreteMechanicsLieGroupParity, DexpInvIsLeftJacobianInverse)
{
  const Vector6 w = (Vector6() << 1.0, -0.5, 0.3, 0.8, 0.2, -0.6).finished();
  for (const auto& xi : sampleTwists()) {
    const Matrix6 leftJinv = SE3::LeftJacobianInverse(xi);
    const Matrix6 rightJinv = SE3::RightJacobianInverse(xi);

    // dexpInv(xi, w) matches the exact left-Jacobian inverse to truncation tol.
    const Vector6 mine = dm::dexpInv(xi, w);
    EXPECT_LT((mine - leftJinv * w).norm(), 1e-3) << "xi=" << xi.transpose();

    // Its transpose matches the transpose of the left-Jacobian inverse.
    const Vector6 mineT = dm::dexpInvTranspose(xi, w);
    EXPECT_LT((mineT - leftJinv.transpose() * w).norm(), 1e-3);

    // Confirm it is the LEFT inverse, not the RIGHT one (they differ at O(ad)
    // for twists with a nonzero angular part).
    if (xi.head<3>().norm() > 0.1) {
      EXPECT_GT((mine - rightJinv * w).norm(), 1e-2)
          << "dexpInv should match Left, not Right: xi=" << xi.transpose();
    }
  }
}
