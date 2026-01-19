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

#include <dart/simulation/World.hpp>

#include <dart/constraint/WeldJointConstraint.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::constraint;
using namespace dart::simulation;

namespace {

SkeletonPtr createFreeSkeleton(const std::string& name)
{
  auto skel = Skeleton::create(name);
  skel->createJointAndBodyNodePair<FreeJoint>();
  return skel;
}

} // namespace

TEST(WeldJointConstraint, CreateWithTwoBodies)
{
  auto skel1 = createFreeSkeleton("skel1");
  auto skel2 = createFreeSkeleton("skel2");
  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);

  auto constraint = std::make_shared<WeldJointConstraint>(body1, body2);

  EXPECT_EQ(constraint->getBodyNode1(), body1);
  EXPECT_EQ(constraint->getBodyNode2(), body2);
}

TEST(WeldJointConstraint, CreateWithOneBody)
{
  auto skel = createFreeSkeleton("skel");
  auto body = skel->getBodyNode(0);

  auto constraint = std::make_shared<WeldJointConstraint>(body);

  EXPECT_EQ(constraint->getBodyNode1(), body);
  EXPECT_EQ(constraint->getBodyNode2(), nullptr);
}

TEST(WeldJointConstraint, SetAndGetRelativeTransform)
{
  auto skel1 = createFreeSkeleton("skel1");
  auto skel2 = createFreeSkeleton("skel2");
  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);

  auto constraint = std::make_shared<WeldJointConstraint>(body1, body2);

  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  constraint->setRelativeTransform(offset);

  EXPECT_TRUE(constraint->getRelativeTransform().isApprox(offset));
}

TEST(WeldJointConstraint, DimensionIsSix)
{
  auto skel1 = createFreeSkeleton("skel1");
  auto skel2 = createFreeSkeleton("skel2");
  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);

  auto constraint = std::make_shared<WeldJointConstraint>(body1, body2);

  EXPECT_EQ(constraint->getDimension(), 6u);
}

TEST(WeldJointConstraint, GetType)
{
  auto skel = createFreeSkeleton("skel");
  auto body = skel->getBodyNode(0);

  auto constraint = std::make_shared<WeldJointConstraint>(body);

  EXPECT_EQ(constraint->getType(), WeldJointConstraint::getStaticType());
  EXPECT_FALSE(constraint->getType().empty());
}

TEST(WeldJointConstraint, StaticSettings)
{
  double originalAllowance = WeldJointConstraint::getErrorAllowance();
  double originalErp = WeldJointConstraint::getErrorReductionParameter();
  double originalErv = WeldJointConstraint::getMaxErrorReductionVelocity();
  double originalCfm = WeldJointConstraint::getConstraintForceMixing();

  WeldJointConstraint::setErrorAllowance(0.005);
  EXPECT_DOUBLE_EQ(WeldJointConstraint::getErrorAllowance(), 0.005);

  WeldJointConstraint::setErrorReductionParameter(0.5);
  EXPECT_DOUBLE_EQ(WeldJointConstraint::getErrorReductionParameter(), 0.5);

  WeldJointConstraint::setMaxErrorReductionVelocity(5.0);
  EXPECT_DOUBLE_EQ(WeldJointConstraint::getMaxErrorReductionVelocity(), 5.0);

  WeldJointConstraint::setConstraintForceMixing(1e-4);
  EXPECT_DOUBLE_EQ(WeldJointConstraint::getConstraintForceMixing(), 1e-4);

  WeldJointConstraint::setErrorAllowance(originalAllowance);
  WeldJointConstraint::setErrorReductionParameter(originalErp);
  WeldJointConstraint::setMaxErrorReductionVelocity(originalErv);
  WeldJointConstraint::setConstraintForceMixing(originalCfm);
}
