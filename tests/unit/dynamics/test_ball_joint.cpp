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

#include <dart/dynamics/ball_joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <gtest/gtest.h>

using namespace dart::dynamics;

//=============================================================================
TEST(BallJoint, CopyAndConstAccess)
{
  auto skeleton = Skeleton::create("ball_joint");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<BallJoint>();
  (void)body;

  BallJoint::Properties props;
  props.mCoordinateChart = BallJoint::CoordinateChart::EULER_XYZ;
  joint->setProperties(props);

  const BallJoint* constJoint = joint;
  EXPECT_EQ(constJoint->getType(), BallJoint::getStaticType());
  EXPECT_TRUE(constJoint->isCyclic(0));

  const Eigen::Vector3d positions(0.1, -0.2, 0.3);
  const auto rotationXyz = BallJoint::convertToRotation(
      positions, BallJoint::CoordinateChart::EULER_XYZ);
  const auto rotationZyx = BallJoint::convertToRotation(
      positions, BallJoint::CoordinateChart::EULER_ZYX);
  EXPECT_NEAR(rotationXyz.determinant(), 1.0, 1e-12);
  EXPECT_NEAR(rotationZyx.determinant(), 1.0, 1e-12);

  const auto jacobian = constJoint->getRelativeJacobianStatic(positions);
  EXPECT_EQ(jacobian.cols(), 3);

  const Eigen::Vector3d q1(0.0, 0.0, 0.0);
  const Eigen::Vector3d q2(0.1, 0.0, 0.0);
  const auto diff = constJoint->getPositionDifferencesStatic(q2, q1);
  EXPECT_EQ(diff.size(), 3);

  const auto jacobianDeriv = constJoint->getRelativeJacobianTimeDeriv();
  EXPECT_EQ(jacobianDeriv.cols(), 3);

  joint->copy(static_cast<const BallJoint*>(nullptr));

  auto [otherJoint, otherBody]
      = skeleton->createJointAndBodyNodePair<BallJoint>();
  (void)otherBody;
  otherJoint->copy(*joint);
  EXPECT_EQ(otherJoint->getCoordinateChart(), joint->getCoordinateChart());
}
