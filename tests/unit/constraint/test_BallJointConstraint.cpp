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

#include <dart/constraint/BallJointConstraint.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::constraint;

namespace {

SkeletonPtr createFreeSkeleton(const std::string& name)
{
  auto skel = Skeleton::create(name);
  skel->createJointAndBodyNodePair<FreeJoint>();
  return skel;
}

} // namespace

TEST(BallJointConstraint, CreateWithTwoBodies)
{
  auto skel1 = createFreeSkeleton("skel1");
  auto skel2 = createFreeSkeleton("skel2");
  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);

  Eigen::Vector3d jointPos(1.0, 2.0, 3.0);

  auto constraint
      = std::make_shared<BallJointConstraint>(body1, body2, jointPos);

  EXPECT_EQ(constraint->getBodyNode1(), body1);
  EXPECT_EQ(constraint->getBodyNode2(), body2);
}

TEST(BallJointConstraint, CreateWithOneBody)
{
  auto skel = createFreeSkeleton("skel");
  auto body = skel->getBodyNode(0);

  Eigen::Vector3d jointPos(0.5, 0.5, 0.5);

  auto constraint = std::make_shared<BallJointConstraint>(body, jointPos);

  EXPECT_EQ(constraint->getBodyNode1(), body);
  EXPECT_EQ(constraint->getBodyNode2(), nullptr);
}

TEST(BallJointConstraint, DimensionIsThree)
{
  auto skel1 = createFreeSkeleton("skel1");
  auto skel2 = createFreeSkeleton("skel2");
  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);

  Eigen::Vector3d jointPos(0.0, 0.0, 0.0);

  auto constraint
      = std::make_shared<BallJointConstraint>(body1, body2, jointPos);

  EXPECT_EQ(constraint->getDimension(), 3u);
}

TEST(BallJointConstraint, GetType)
{
  auto skel = createFreeSkeleton("skel");
  auto body = skel->getBodyNode(0);

  Eigen::Vector3d jointPos(0.0, 0.0, 0.0);

  auto constraint = std::make_shared<BallJointConstraint>(body, jointPos);

  EXPECT_EQ(constraint->getType(), BallJointConstraint::getStaticType());
  EXPECT_FALSE(constraint->getType().empty());
}

TEST(BallJointConstraint, StaticSettings)
{
  double originalAllowance = BallJointConstraint::getErrorAllowance();
  double originalErp = BallJointConstraint::getErrorReductionParameter();
  double originalErv = BallJointConstraint::getMaxErrorReductionVelocity();
  double originalCfm = BallJointConstraint::getConstraintForceMixing();

  BallJointConstraint::setErrorAllowance(0.005);
  EXPECT_DOUBLE_EQ(BallJointConstraint::getErrorAllowance(), 0.005);

  BallJointConstraint::setErrorReductionParameter(0.5);
  EXPECT_DOUBLE_EQ(BallJointConstraint::getErrorReductionParameter(), 0.5);

  BallJointConstraint::setMaxErrorReductionVelocity(5.0);
  EXPECT_DOUBLE_EQ(BallJointConstraint::getMaxErrorReductionVelocity(), 5.0);

  BallJointConstraint::setConstraintForceMixing(1e-4);
  EXPECT_DOUBLE_EQ(BallJointConstraint::getConstraintForceMixing(), 1e-4);

  BallJointConstraint::setErrorAllowance(originalAllowance);
  BallJointConstraint::setErrorReductionParameter(originalErp);
  BallJointConstraint::setMaxErrorReductionVelocity(originalErv);
  BallJointConstraint::setConstraintForceMixing(originalCfm);
}
