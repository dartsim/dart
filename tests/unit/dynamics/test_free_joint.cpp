/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

using namespace dart::dynamics;

//==============================================================================
// The in-place FreeJoint::integratePositions(dt) has identity-frame fast paths
// (added when forward-porting the DART 6 LTS optimization). This verifies it
// stays equivalent to the unchanged reference vector overload
// Joint::integratePositions(q0, v, dt) across identity, translated-parent, and
// fully-offset joint frames.
TEST(FreeJoint, IntegratePositionsMatchesVectorOverload)
{
  auto expectMatchesVectorOverload
      = [](const Eigen::Isometry3d& parentBodyToJoint,
           const Eigen::Isometry3d& childBodyToJoint,
           const char* label) {
          SkeletonPtr skel = Skeleton::create(label);

          auto [freeJoint, freeBody]
              = skel->createJointAndBodyNodePair<FreeJoint>();
          (void)freeBody;

          freeJoint->setTransformFromParentBodyNode(parentBodyToJoint);
          freeJoint->setTransformFromChildBodyNode(childBodyToJoint);

          Eigen::Isometry3d initialTransform = Eigen::Isometry3d::Identity();
          initialTransform.linear()
              = (Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX())
                 * Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitY())
                 * Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitZ()))
                    .toRotationMatrix();
          initialTransform.translation() = Eigen::Vector3d(0.4, -0.2, 0.3);

          const Eigen::Vector6d q0
              = FreeJoint::convertToPositions(initialTransform);
          Eigen::Vector6d v;
          v << 0.17, -0.11, 0.07, 0.4, -0.2, 0.1;

          freeJoint->setPositions(q0);
          freeJoint->setVelocities(v);

          const double dt = 0.003;
          const Eigen::VectorXd expected
              = static_cast<const Joint*>(freeJoint)->integratePositions(
                  q0, v, dt);
          skel->integratePositions(dt);

          EXPECT_TRUE(freeJoint->getPositions().isApprox(expected, 1e-12))
              << label << "\nexpected: " << expected.transpose()
              << "\nactual: " << freeJoint->getPositions().transpose();
        };

  expectMatchesVectorOverload(
      Eigen::Isometry3d::Identity(),
      Eigen::Isometry3d::Identity(),
      "identity joint frames");

  Eigen::Isometry3d parentBodyToJoint = Eigen::Isometry3d::Identity();
  parentBodyToJoint.linear()
      = Eigen::AngleAxisd(0.13, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  parentBodyToJoint.translation() = Eigen::Vector3d(0.03, -0.05, 0.07);

  Eigen::Isometry3d translatedParentBodyToJoint = Eigen::Isometry3d::Identity();
  translatedParentBodyToJoint.translation()
      = Eigen::Vector3d(0.03, -0.05, 0.07);

  expectMatchesVectorOverload(
      translatedParentBodyToJoint,
      Eigen::Isometry3d::Identity(),
      "translated parent joint frame");

  Eigen::Isometry3d childBodyToJoint = Eigen::Isometry3d::Identity();
  childBodyToJoint.linear()
      = Eigen::AngleAxisd(-0.08, Eigen::Vector3d::UnitY()).toRotationMatrix();
  childBodyToJoint.translation() = Eigen::Vector3d(-0.02, 0.04, -0.06);

  expectMatchesVectorOverload(
      parentBodyToJoint, childBodyToJoint, "offset joint frames");

  // Identity child frame but a rotated parent frame exercises the
  // identity-child / non-identity-parent-rotation fast-path branch.
  expectMatchesVectorOverload(
      parentBodyToJoint,
      Eigen::Isometry3d::Identity(),
      "rotated parent + identity child");
}

//==============================================================================
// Identity child frame selects the fast paths in updateRelativeTransform(),
// updateRelativeJacobian(), and updateRelativeJacobianTimeDeriv(). Verify those
// paths run for both identity and rotated parent frames and produce the
// expected block-diagonal Jacobian structure (R^T on the diagonal 3x3 blocks,
// zero off-diagonal blocks).
TEST(FreeJoint, IdentityChildFrameFastPaths)
{
  auto makeJoint = [](const Eigen::Isometry3d& parent) {
    auto skel = Skeleton::create("fj_fastpath");
    auto [joint, body] = skel->createJointAndBodyNodePair<FreeJoint>();
    (void)body;
    joint->setTransformFromParentBodyNode(parent);
    joint->setTransformFromChildBodyNode(Eigen::Isometry3d::Identity());

    Eigen::Vector6d q;
    q << 0.2, -0.1, 0.05, 0.4, -0.2, 0.3;
    Eigen::Vector6d v;
    v << 0.17, -0.11, 0.07, 0.4, -0.2, 0.1;
    joint->setPositions(q);
    joint->setVelocities(v);
    return std::make_pair(skel, joint);
  };

  Eigen::Isometry3d rotatedParent = Eigen::Isometry3d::Identity();
  rotatedParent.linear()
      = Eigen::AngleAxisd(0.21, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  rotatedParent.translation() = Eigen::Vector3d(0.05, -0.03, 0.02);

  for (const auto& parent : {Eigen::Isometry3d::Identity(), rotatedParent}) {
    auto [skel, joint] = makeJoint(parent);
    (void)skel;

    const Eigen::Isometry3d T = joint->getRelativeTransform();
    EXPECT_TRUE(T.matrix().allFinite());

    const Eigen::Matrix3d Rt = T.linear().transpose();

    const Eigen::Matrix6d J = joint->getRelativeJacobian();
    EXPECT_TRUE(J.allFinite());
    // Extra parens: the comma in topLeftCorner<3, 3> would otherwise split the
    // gtest macro argument.
    EXPECT_TRUE((J.topLeftCorner<3, 3>().isApprox(Rt, 1e-12)));
    EXPECT_TRUE((J.bottomRightCorner<3, 3>().isApprox(Rt, 1e-12)));
    EXPECT_TRUE((J.topRightCorner<3, 3>().isZero()));
    EXPECT_TRUE((J.bottomLeftCorner<3, 3>().isZero()));

    const Eigen::Matrix6d Jdot = joint->getRelativeJacobianTimeDeriv();
    EXPECT_TRUE(Jdot.allFinite());
    EXPECT_TRUE((Jdot.topRightCorner<3, 3>().isZero()));
    EXPECT_TRUE((Jdot.bottomLeftCorner<3, 3>().isZero()));
  }
}

//==============================================================================
// The identity-child integratePositions fast path supports non-EXP_MAP
// coordinate charts via its convertToPositions() branch. Exercise that branch
// and confirm the integration stays finite.
TEST(FreeJoint, IntegratePositionsFastPathNonExpMapChart)
{
  for (auto chart :
       {FreeJoint::CoordinateChart::EULER_XYZ,
        FreeJoint::CoordinateChart::EULER_ZYX}) {
    auto skel = Skeleton::create("fj_chart");
    auto [joint, body] = skel->createJointAndBodyNodePair<FreeJoint>();
    (void)body;
    joint->setCoordinateChart(chart);
    joint->setTransformFromParentBodyNode(Eigen::Isometry3d::Identity());
    joint->setTransformFromChildBodyNode(Eigen::Isometry3d::Identity());

    Eigen::Vector6d q;
    q << 0.15, -0.07, 0.04, 0.2, -0.1, 0.05;
    Eigen::Vector6d v;
    v << 0.1, -0.05, 0.03, 0.2, -0.1, 0.05;
    joint->setPositions(q);
    joint->setVelocities(v);

    skel->integratePositions(0.003);
    EXPECT_TRUE(joint->getPositions().allFinite());
  }
}
