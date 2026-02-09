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

#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/universal_joint.hpp>

#include <gtest/gtest.h>

using namespace dart::dynamics;

//=============================================================================
TEST(UniversalJoint, CopyAndConstAccess)
{
  auto skeleton = Skeleton::create("universal_joint");
  auto [joint, body] = skeleton->createJointAndBodyNodePair<UniversalJoint>();
  (void)body;

  UniversalJoint::Properties props;
  props.mAxis[0] = Eigen::Vector3d::UnitX();
  props.mAxis[1] = Eigen::Vector3d::UnitZ();
  joint->setProperties(props);

  const UniversalJoint* constJoint = joint;
  EXPECT_EQ(constJoint->getType(), UniversalJoint::getStaticType());
  EXPECT_TRUE(constJoint->getAxis1().isApprox(Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(constJoint->getAxis2().isApprox(Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(constJoint->isCyclic(0));

  const Eigen::Vector2d positions(0.1, -0.2);
  const auto jacobian = constJoint->getRelativeJacobianStatic(positions);
  EXPECT_EQ(jacobian.cols(), 2);

  joint->copy(static_cast<const UniversalJoint*>(nullptr));
  joint->copy(joint);

  auto [otherJoint, otherBody]
      = skeleton->createJointAndBodyNodePair<UniversalJoint>();
  (void)otherBody;
  otherJoint->copy(*joint);
  EXPECT_TRUE(otherJoint->getAxis1().isApprox(joint->getAxis1()));

  otherJoint->setAxis1(Eigen::Vector3d::UnitY());
  joint->copy(otherJoint);
  EXPECT_TRUE(joint->getAxis1().isApprox(Eigen::Vector3d::UnitY()));
}
