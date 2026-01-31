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

#include "dart/constraint/coupler_constraint.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/joint.hpp"
#include "dart/dynamics/mimic_dof_properties.hpp"
#include "dart/dynamics/revolute_joint.hpp"
#include "dart/dynamics/skeleton.hpp"

#include <gtest/gtest.h>

#include <vector>

using namespace dart;

namespace {

class ExposedCouplerConstraint final : public constraint::CouplerConstraint
{
public:
  using CouplerConstraint::applyImpulse;
  using CouplerConstraint::applyUnitImpulse;
  using CouplerConstraint::CouplerConstraint;
  using CouplerConstraint::excite;
  using CouplerConstraint::getInformation;
  using CouplerConstraint::getVelocityChange;
  using CouplerConstraint::isActive;
  using CouplerConstraint::unexcite;
  using CouplerConstraint::update;
};

struct LcpBuffers
{
  std::vector<double> x;
  std::vector<double> lo;
  std::vector<double> hi;
  std::vector<double> b;
  std::vector<double> w;
  std::vector<int> findex;
};

void prepareConstraintInfo(
    constraint::ConstraintInfo& info,
    LcpBuffers& buffers,
    std::size_t dim,
    double invTimeStep,
    constraint::ConstraintPhase phase,
    bool useSplitImpulse)
{
  buffers.x.assign(dim, 0.0);
  buffers.lo.assign(dim, 0.0);
  buffers.hi.assign(dim, 0.0);
  buffers.b.assign(dim, 0.0);
  buffers.w.assign(dim, 0.0);
  buffers.findex.assign(dim, -1);

  info.x = buffers.x.data();
  info.lo = buffers.lo.data();
  info.hi = buffers.hi.data();
  info.b = buffers.b.data();
  info.w = buffers.w.data();
  info.findex = buffers.findex.data();
  info.invTimeStep = invTimeStep;
  info.phase = phase;
  info.useSplitImpulse = useSplitImpulse;
}

} // namespace

TEST(CouplerConstraint, CouplerConstraintMaintainsMimic)
{
  auto skeleton = dynamics::Skeleton::create("coupler");
  skeleton->setTimeStep(0.01);

  dynamics::BodyNode::Properties bodyProps;
  bodyProps.mInertia.setMass(1.0);

  dynamics::RevoluteJoint::Properties joint1Props;
  joint1Props.mName = "joint1";
  joint1Props.mAxis = Eigen::Vector3d::UnitZ();
  auto pair1 = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
      nullptr, joint1Props, bodyProps);

  dynamics::RevoluteJoint::Properties joint2Props;
  joint2Props.mName = "joint2";
  joint2Props.mAxis = Eigen::Vector3d::UnitZ();
  auto pair2 = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
      pair1.second, joint2Props, bodyProps);

  auto* joint1 = pair1.first;
  auto* joint2 = pair2.first;
  ASSERT_NE(joint1, nullptr);
  ASSERT_NE(joint2, nullptr);

  joint2->setActuatorType(dynamics::Joint::MIMIC);
  joint1->setPosition(0, 0.3);
  joint2->setPosition(0, 0.0);

  std::vector<dynamics::MimicDofProperties> mimicProps(1);
  mimicProps[0].mReferenceJoint = joint1;
  mimicProps[0].mReferenceDofIndex = 0;
  mimicProps[0].mMultiplier = 0.5;
  mimicProps[0].mOffset = 0.1;
  mimicProps[0].mConstraintType = dynamics::MimicConstraintType::Coupler;

  ExposedCouplerConstraint constraint(joint2, mimicProps);
  constraint.update();
  EXPECT_TRUE(constraint.isActive());
  EXPECT_GT(constraint.getDimension(), 0u);

  constraint::ConstraintInfo info{};
  LcpBuffers buffers;
  prepareConstraintInfo(
      info,
      buffers,
      constraint.getDimension(),
      1.0 / skeleton->getTimeStep(),
      constraint::ConstraintPhase::Velocity,
      false);
  constraint.getInformation(&info);

  constraint.applyUnitImpulse(0);
  skeleton->computeForwardDynamics();
  skeleton->computeImpulseForwardDynamics();
  std::vector<double> vel(constraint.getDimension(), 0.0);
  constraint.getVelocityChange(vel.data(), true);

  std::vector<double> lambda(constraint.getDimension(), 0.05);
  constraint.applyImpulse(lambda.data());
  constraint.excite();
  constraint.unexcite();
}
