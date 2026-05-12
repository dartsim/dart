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

#include "dart/constraint/constraint_solver.hpp"
#include "dart/constraint/coupler_constraint.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/joint.hpp"
#include "dart/dynamics/mimic_dof_properties.hpp"
#include "dart/dynamics/revolute_joint.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/dynamics/universal_joint.hpp"
#include "dart/simulation/world.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include <cmath>

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
  using CouplerConstraint::uniteSkeletons;
  using CouplerConstraint::update;

  dynamics::SkeletonPtr exposedGetRootSkeleton() const
  {
    return getRootSkeleton();
  }
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

struct SingleDofCouplerSetup
{
  dynamics::SkeletonPtr referenceSkeleton;
  dynamics::SkeletonPtr dependentSkeleton;
  dynamics::RevoluteJoint* referenceJoint{nullptr};
  dynamics::RevoluteJoint* dependentJoint{nullptr};
  std::vector<dynamics::MimicDofProperties> mimicProps;
};

dynamics::BodyNode::Properties createBodyProperties()
{
  dynamics::BodyNode::Properties bodyProps;
  bodyProps.mInertia.setMass(1.0);
  return bodyProps;
}

void configureMimicProperty(
    dynamics::MimicDofProperties& mimicProp,
    const dynamics::Joint* referenceJoint,
    std::size_t referenceDof,
    double multiplier = 1.0,
    double offset = 0.0)
{
  mimicProp.mReferenceJoint = referenceJoint;
  mimicProp.mReferenceDofIndex = referenceDof;
  mimicProp.mMultiplier = multiplier;
  mimicProp.mOffset = offset;
  mimicProp.mConstraintType = dynamics::MimicConstraintType::Coupler;
}

SingleDofCouplerSetup createSingleDofCouplerSetup(
    const std::string& name, bool sameSkeleton = false)
{
  SingleDofCouplerSetup setup;
  const auto bodyProps = createBodyProperties();

  dynamics::RevoluteJoint::Properties refProps;
  refProps.mName = name + "_reference_joint";
  refProps.mAxis = Eigen::Vector3d::UnitZ();

  dynamics::RevoluteJoint::Properties depProps;
  depProps.mName = name + "_dependent_joint";
  depProps.mAxis = Eigen::Vector3d::UnitZ();

  if (sameSkeleton) {
    setup.referenceSkeleton = dynamics::Skeleton::create(name + "_skeleton");
    setup.dependentSkeleton = setup.referenceSkeleton;

    auto refPair = setup.referenceSkeleton
                       ->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
                           nullptr, refProps, bodyProps);
    auto depPair = setup.referenceSkeleton
                       ->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
                           refPair.second, depProps, bodyProps);
    setup.referenceJoint = refPair.first;
    setup.dependentJoint = depPair.first;
  } else {
    setup.referenceSkeleton
        = dynamics::Skeleton::create(name + "_reference_skeleton");
    setup.dependentSkeleton
        = dynamics::Skeleton::create(name + "_dependent_skeleton");

    auto refPair = setup.referenceSkeleton
                       ->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
                           nullptr, refProps, bodyProps);
    auto depPair = setup.dependentSkeleton
                       ->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
                           nullptr, depProps, bodyProps);
    setup.referenceJoint = refPair.first;
    setup.dependentJoint = depPair.first;
  }

  setup.dependentJoint->setActuatorType(dynamics::Joint::MIMIC);
  setup.mimicProps.resize(1);
  configureMimicProperty(
      setup.mimicProps[0], setup.referenceJoint, 0, 1.0, 0.0);

  return setup;
}

SingleDofCouplerSetup createDependentCouplerForReference(
    const std::string& name, const dynamics::RevoluteJoint* referenceJoint)
{
  SingleDofCouplerSetup setup;
  setup.referenceJoint = const_cast<dynamics::RevoluteJoint*>(referenceJoint);
  setup.referenceSkeleton = std::const_pointer_cast<dynamics::Skeleton>(
      referenceJoint->getSkeleton());
  setup.dependentSkeleton
      = dynamics::Skeleton::create(name + "_dependent_skeleton");

  dynamics::RevoluteJoint::Properties depProps;
  depProps.mName = name + "_dependent_joint";
  depProps.mAxis = Eigen::Vector3d::UnitZ();
  auto depPair = setup.dependentSkeleton
                     ->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
                         nullptr, depProps, createBodyProperties());
  setup.dependentJoint = depPair.first;
  setup.dependentJoint->setActuatorType(dynamics::Joint::MIMIC);

  setup.mimicProps.resize(1);
  configureMimicProperty(
      setup.mimicProps[0], setup.referenceJoint, 0, 1.0, 0.0);

  return setup;
}

} // namespace

TEST(CouplerConstraint, TypeAndConstraintForceMixingAccessors)
{
  auto setup = createSingleDofCouplerSetup("coupler_type");
  ExposedCouplerConstraint constraint(setup.dependentJoint, setup.mimicProps);

  EXPECT_EQ(
      constraint.getType(), constraint::CouplerConstraint::getStaticType());
  EXPECT_EQ(constraint.exposedGetRootSkeleton(), setup.dependentSkeleton);

  const double prevCfm
      = constraint::CouplerConstraint::getConstraintForceMixing();
  constraint::CouplerConstraint::setConstraintForceMixing(0.0);
  EXPECT_DOUBLE_EQ(
      constraint::CouplerConstraint::getConstraintForceMixing(), 1e-9);

  constraint::CouplerConstraint::setConstraintForceMixing(3e-6);
  EXPECT_DOUBLE_EQ(
      constraint::CouplerConstraint::getConstraintForceMixing(), 3e-6);

  constraint::CouplerConstraint::setConstraintForceMixing(prevCfm);
}

TEST(CouplerConstraint, UpdateFallbackLimitsAndWarmStart)
{
  auto setup = createSingleDofCouplerSetup("coupler_fallback");
  setup.dependentSkeleton->setTimeStep(0.01);
  setup.referenceJoint->setPosition(0, 10.0);
  setup.dependentJoint->setPosition(0, 0.0);
  setup.dependentJoint->setVelocity(0, 0.0);
  setup.dependentJoint->setVelocityLowerLimit(
      0, -std::numeric_limits<double>::infinity());
  setup.dependentJoint->setVelocityUpperLimit(
      0, std::numeric_limits<double>::infinity());
  setup.dependentJoint->setForceLowerLimit(
      0, -std::numeric_limits<double>::infinity());
  setup.dependentJoint->setForceUpperLimit(
      0, std::numeric_limits<double>::infinity());

  ExposedCouplerConstraint constraint(setup.dependentJoint, setup.mimicProps);
  constraint.update();
  ASSERT_TRUE(constraint.isActive());
  ASSERT_EQ(constraint.getDimension(), 1u);

  constraint::ConstraintInfo info{};
  LcpBuffers buffers;
  prepareConstraintInfo(
      info,
      buffers,
      constraint.getDimension(),
      1.0 / setup.dependentSkeleton->getTimeStep(),
      constraint::ConstraintPhase::Velocity,
      false);
  constraint.getInformation(&info);

  EXPECT_DOUBLE_EQ(buffers.b[0], 50.0);
  EXPECT_DOUBLE_EQ(buffers.lo[0], -8.0);
  EXPECT_DOUBLE_EQ(buffers.hi[0], 8.0);
  EXPECT_DOUBLE_EQ(buffers.x[0], 0.0);

  std::vector<double> lambda(constraint.getDimension(), 0.25);
  constraint.applyImpulse(lambda.data());

  constraint.update();
  LcpBuffers warmStartBuffers;
  prepareConstraintInfo(
      info,
      warmStartBuffers,
      constraint.getDimension(),
      1.0 / setup.dependentSkeleton->getTimeStep(),
      constraint::ConstraintPhase::Velocity,
      false);
  constraint.getInformation(&info);

  EXPECT_DOUBLE_EQ(warmStartBuffers.x[0], 0.25);
}

TEST(CouplerConstraint, InactiveDofsAreSkippedAcrossCallbacks)
{
  auto referenceSkeleton
      = dynamics::Skeleton::create("coupler_inactive_reference");
  auto dependentSkeleton
      = dynamics::Skeleton::create("coupler_inactive_dependent");
  const auto bodyProps = createBodyProperties();

  dynamics::RevoluteJoint::Properties refProps;
  refProps.mName = "reference_joint";
  refProps.mAxis = Eigen::Vector3d::UnitZ();
  auto refPair
      = referenceSkeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr, refProps, bodyProps);

  auto depPair
      = dependentSkeleton->createJointAndBodyNodePair<dynamics::UniversalJoint>(
          nullptr, dynamics::UniversalJoint::Properties(), bodyProps);

  auto* referenceJoint = refPair.first;
  auto* dependentJoint = depPair.first;
  ASSERT_NE(referenceJoint, nullptr);
  ASSERT_NE(dependentJoint, nullptr);

  referenceJoint->setPosition(0, 0.4);
  dependentJoint->setPosition(1, 0.0);
  dependentJoint->setActuatorType(1, dynamics::Joint::MIMIC);

  std::vector<dynamics::MimicDofProperties> mimicProps(2);
  configureMimicProperty(mimicProps[0], referenceJoint, 0, 1.0, 0.0);
  configureMimicProperty(mimicProps[1], referenceJoint, 0, 1.0, 0.0);

  ExposedCouplerConstraint constraint(dependentJoint, mimicProps);
  constraint.update();
  ASSERT_TRUE(constraint.isActive());
  ASSERT_EQ(constraint.getDimension(), 1u);

  constraint::ConstraintInfo info{};
  LcpBuffers buffers;
  prepareConstraintInfo(
      info,
      buffers,
      constraint.getDimension(),
      1.0 / dependentSkeleton->getTimeStep(),
      constraint::ConstraintPhase::Velocity,
      false);
  constraint.getInformation(&info);
  EXPECT_TRUE(std::isfinite(buffers.b[0]));

  constraint.applyUnitImpulse(0);
  std::vector<double> vel(constraint.getDimension(), 0.0);
  constraint.getVelocityChange(vel.data(), false);
  EXPECT_TRUE(std::isfinite(vel[0]));

  constraint.excite();
  EXPECT_TRUE(referenceSkeleton->isImpulseApplied());
  EXPECT_TRUE(dependentSkeleton->isImpulseApplied());
  constraint.unexcite();
  EXPECT_FALSE(referenceSkeleton->isImpulseApplied());
  EXPECT_FALSE(dependentSkeleton->isImpulseApplied());

  std::vector<double> lambda(constraint.getDimension(), 0.125);
  constraint.applyImpulse(lambda.data());
  EXPECT_DOUBLE_EQ(dependentJoint->getConstraintImpulse(0), 0.0);
  EXPECT_DOUBLE_EQ(dependentJoint->getConstraintImpulse(1), 0.125);
  EXPECT_DOUBLE_EQ(referenceJoint->getConstraintImpulse(0), -0.125);
}

TEST(CouplerConstraint, UniteSkeletonsSkipsNonReactiveAndNonMimicJoints)
{
  auto nonReactiveDependent
      = createSingleDofCouplerSetup("coupler_non_reactive_dependent");
  nonReactiveDependent.dependentSkeleton->setMobile(false);
  ExposedCouplerConstraint inactiveDependentConstraint(
      nonReactiveDependent.dependentJoint, nonReactiveDependent.mimicProps);
  inactiveDependentConstraint.uniteSkeletons();
  EXPECT_EQ(
      nonReactiveDependent.referenceSkeleton->mUnionRootSkeleton.lock(),
      nonReactiveDependent.referenceSkeleton);
  EXPECT_EQ(
      nonReactiveDependent.dependentSkeleton->mUnionRootSkeleton.lock(),
      nonReactiveDependent.dependentSkeleton);

  auto nonReactiveReference
      = createSingleDofCouplerSetup("coupler_non_reactive_reference");
  nonReactiveReference.referenceSkeleton->setMobile(false);
  ExposedCouplerConstraint inactiveReferenceConstraint(
      nonReactiveReference.dependentJoint, nonReactiveReference.mimicProps);
  inactiveReferenceConstraint.uniteSkeletons();
  EXPECT_EQ(
      nonReactiveReference.referenceSkeleton->mUnionRootSkeleton.lock(),
      nonReactiveReference.referenceSkeleton);
  EXPECT_EQ(
      nonReactiveReference.dependentSkeleton->mUnionRootSkeleton.lock(),
      nonReactiveReference.dependentSkeleton);

  auto forceActuator = createSingleDofCouplerSetup("coupler_force_actuator");
  forceActuator.dependentJoint->setActuatorType(dynamics::Joint::FORCE);
  ExposedCouplerConstraint forceActuatorConstraint(
      forceActuator.dependentJoint, forceActuator.mimicProps);
  forceActuatorConstraint.uniteSkeletons();
  EXPECT_EQ(
      forceActuator.referenceSkeleton->mUnionRootSkeleton.lock(),
      forceActuator.referenceSkeleton);
  EXPECT_EQ(
      forceActuator.dependentSkeleton->mUnionRootSkeleton.lock(),
      forceActuator.dependentSkeleton);
}

TEST(CouplerConstraint, UniteSkeletonsHandlesSameAndCrossSkeletonRoots)
{
  auto sameSkeleton = createSingleDofCouplerSetup("coupler_same", true);
  ExposedCouplerConstraint sameSkeletonConstraint(
      sameSkeleton.dependentJoint, sameSkeleton.mimicProps);
  sameSkeletonConstraint.uniteSkeletons();
  EXPECT_EQ(
      sameSkeleton.dependentSkeleton->mUnionRootSkeleton.lock(),
      sameSkeleton.dependentSkeleton);

  auto crossSkeleton = createSingleDofCouplerSetup("coupler_cross");
  ExposedCouplerConstraint crossSkeletonConstraint(
      crossSkeleton.dependentJoint, crossSkeleton.mimicProps);
  crossSkeletonConstraint.uniteSkeletons();
  EXPECT_EQ(
      crossSkeleton.referenceSkeleton->mUnionRootSkeleton.lock(),
      crossSkeleton.dependentSkeleton);

  auto seed = createSingleDofCouplerSetup("coupler_seed");
  ExposedCouplerConstraint seedConstraint(seed.dependentJoint, seed.mimicProps);
  seedConstraint.uniteSkeletons();
  ASSERT_EQ(
      seed.referenceSkeleton->mUnionRootSkeleton.lock(),
      seed.dependentSkeleton);

  auto dependentToLargerReference = createDependentCouplerForReference(
      "coupler_dependent_to_larger_reference", seed.referenceJoint);
  ExposedCouplerConstraint largerReferenceConstraint(
      dependentToLargerReference.dependentJoint,
      dependentToLargerReference.mimicProps);
  largerReferenceConstraint.uniteSkeletons();
  EXPECT_EQ(
      dependentToLargerReference.dependentSkeleton->mUnionRootSkeleton.lock(),
      seed.dependentSkeleton);
}

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

TEST(CouplerConstraint, WorldStepMaintainsMimicRelation)
{
  auto world = simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  auto skeleton = dynamics::Skeleton::create("coupler_world");
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

  joint1->setPosition(0, 0.6);
  joint2->setPosition(0, -0.4);
  joint2->setActuatorType(dynamics::Joint::MIMIC);

  std::vector<dynamics::MimicDofProperties> mimicProps(1);
  mimicProps[0].mReferenceJoint = joint1;
  mimicProps[0].mReferenceDofIndex = 0;
  mimicProps[0].mMultiplier = 2.0;
  mimicProps[0].mOffset = -0.1;
  mimicProps[0].mConstraintType = dynamics::MimicConstraintType::Coupler;

  auto constraint
      = std::make_shared<constraint::CouplerConstraint>(joint2, mimicProps);
  world->addSkeleton(skeleton);
  world->getConstraintSolver()->addConstraint(constraint);

  const double initialExpected
      = joint1->getPosition(0) * mimicProps[0].mMultiplier
        + mimicProps[0].mOffset;
  const double initialError
      = std::abs(joint2->getPosition(0) - initialExpected);
  EXPECT_GT(initialError, 0.0);

  for (int i = 0; i < 100; ++i) {
    world->step();
  }

  const double finalExpected
      = joint1->getPosition(0) * mimicProps[0].mMultiplier
        + mimicProps[0].mOffset;
  const double finalError = std::abs(joint2->getPosition(0) - finalExpected);
  EXPECT_LT(finalError, initialError);
  EXPECT_LT(finalError, 0.25);
}
