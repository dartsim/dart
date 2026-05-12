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

#include <dart/simulation/world.hpp>

#include <dart/constraint/constraint_solver.hpp>
#include <dart/constraint/weld_joint_constraint.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <vector>

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
    ConstraintInfo& info,
    LcpBuffers& buffers,
    std::size_t dim,
    double invTimeStep)
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
  info.phase = ConstraintPhase::Velocity;
  info.useSplitImpulse = false;
}

class ExposedWeldJointConstraint final : public WeldJointConstraint
{
public:
  using WeldJointConstraint::applyImpulse;
  using WeldJointConstraint::applyUnitImpulse;
  using WeldJointConstraint::excite;
  using WeldJointConstraint::getInformation;
  using WeldJointConstraint::getVelocityChange;
  using WeldJointConstraint::isActive;
  using WeldJointConstraint::unexcite;
  using WeldJointConstraint::uniteSkeletons;
  using WeldJointConstraint::update;
  using WeldJointConstraint::WeldJointConstraint;

  SkeletonPtr exposedGetRootSkeleton() const
  {
    return getRootSkeleton();
  }
};

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

TEST(WeldJointConstraint, OneBodyCallbackPipeline)
{
  auto skel = createFreeSkeleton("one_body_weld_callbacks");
  auto* body = skel->getBodyNode(0);
  ASSERT_NE(body, nullptr);

  ExposedWeldJointConstraint constraint(body);
  constraint.update();
  ASSERT_TRUE(constraint.isActive());
  ASSERT_EQ(constraint.getDimension(), 6u);
  EXPECT_EQ(constraint.exposedGetRootSkeleton(), skel);

  ConstraintInfo info{};
  LcpBuffers buffers;
  prepareConstraintInfo(info, buffers, constraint.getDimension(), 1000.0);
  constraint.getInformation(&info);
  EXPECT_TRUE(
      (Eigen::Map<Eigen::Matrix<double, 6, 1>>(buffers.b.data()).allFinite()));

  constraint.applyUnitImpulse(0);
  skel->computeForwardDynamics();
  constraint.excite();
  EXPECT_TRUE(skel->isImpulseApplied());

  std::vector<double> vel(constraint.getDimension(), 0.0);
  constraint.getVelocityChange(vel.data(), true);
  EXPECT_TRUE(
      (Eigen::Map<Eigen::Matrix<double, 6, 1>>(vel.data()).allFinite()));

  std::vector<double> lambda = {0.1, -0.2, 0.3, -0.4, 0.5, -0.6};
  constraint.applyImpulse(lambda.data());
  constraint.unexcite();
  EXPECT_FALSE(skel->isImpulseApplied());

  LcpBuffers warmStartBuffers;
  prepareConstraintInfo(
      info, warmStartBuffers, constraint.getDimension(), 1000.0);
  constraint.getInformation(&info);
  for (std::size_t i = 0; i < lambda.size(); ++i) {
    EXPECT_DOUBLE_EQ(warmStartBuffers.x[i], lambda[i]);
  }
}

TEST(WeldJointConstraint, TwoBodyCallbackPipelineAndUnion)
{
  auto skel1 = createFreeSkeleton("two_body_weld_callbacks_1");
  auto skel2 = createFreeSkeleton("two_body_weld_callbacks_2");
  auto* body1 = skel1->getBodyNode(0);
  auto* body2 = skel2->getBodyNode(0);
  ASSERT_NE(body1, nullptr);
  ASSERT_NE(body2, nullptr);

  ExposedWeldJointConstraint constraint(body1, body2);
  constraint.update();
  ASSERT_TRUE(constraint.isActive());

  ConstraintInfo info{};
  LcpBuffers buffers;
  prepareConstraintInfo(info, buffers, constraint.getDimension(), 1000.0);
  constraint.getInformation(&info);
  EXPECT_TRUE(
      (Eigen::Map<Eigen::Matrix<double, 6, 1>>(buffers.b.data()).allFinite()));

  constraint.applyUnitImpulse(0);
  skel1->computeForwardDynamics();
  skel2->computeForwardDynamics();
  constraint.excite();
  EXPECT_TRUE(skel1->isImpulseApplied());
  EXPECT_TRUE(skel2->isImpulseApplied());

  std::vector<double> vel(constraint.getDimension(), 0.0);
  constraint.getVelocityChange(vel.data(), true);
  EXPECT_TRUE(
      (Eigen::Map<Eigen::Matrix<double, 6, 1>>(vel.data()).allFinite()));

  std::vector<double> lambda = {0.05, 0.04, -0.03, 0.02, -0.01, 0.005};
  constraint.applyImpulse(lambda.data());
  constraint.unexcite();
  EXPECT_FALSE(skel1->isImpulseApplied());
  EXPECT_FALSE(skel2->isImpulseApplied());

  constraint.uniteSkeletons();
  EXPECT_EQ(skel2->mUnionRootSkeleton.lock(), skel1);
}

TEST(WeldJointConstraint, NonReactiveFirstBodyUsesReactiveSecondBody)
{
  auto skel = Skeleton::create("non_reactive_first_weld_body");
  auto [weldJoint, body1] = skel->createJointAndBodyNodePair<WeldJoint>();
  auto [freeJoint, body2] = skel->createJointAndBodyNodePair<FreeJoint>(body1);
  (void)weldJoint;
  (void)freeJoint;
  ASSERT_NE(body1, nullptr);
  ASSERT_NE(body2, nullptr);
  ASSERT_FALSE(body1->isReactive());
  ASSERT_TRUE(body2->isReactive());

  ExposedWeldJointConstraint constraint(body1, body2);
  EXPECT_TRUE(constraint.isActive());
  EXPECT_EQ(constraint.exposedGetRootSkeleton(), skel);
  constraint.update();

  constraint.applyUnitImpulse(0);
  skel->computeForwardDynamics();

  constraint.excite();
  EXPECT_TRUE(skel->isImpulseApplied());
  constraint.unexcite();
  EXPECT_FALSE(skel->isImpulseApplied());

  std::vector<double> lambda = {0.01, 0.02, 0.03, 0.04, 0.05, 0.06};
  constraint.applyImpulse(lambda.data());
  EXPECT_TRUE(body2->getConstraintImpulse().allFinite());

  constraint.uniteSkeletons();
  EXPECT_EQ(skel->mUnionRootSkeleton.lock(), skel);
}

TEST(WeldJointConstraint, SimulateWithTwoBodies)
{
  auto world = World::create();

  auto skel1 = createFreeSkeleton("skel1");
  auto skel2 = createFreeSkeleton("skel2");
  world->addSkeleton(skel1);
  world->addSkeleton(skel2);

  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);

  skel2->setPosition(3, 1.0);

  auto constraint = std::make_shared<WeldJointConstraint>(body1, body2);
  world->getConstraintSolver()->addConstraint(constraint);

  for (int i = 0; i < 100; ++i) {
    world->step();
  }

  Eigen::Isometry3d relTransform
      = body1->getTransform().inverse() * body2->getTransform();
  Eigen::Isometry3d expectedRelTransform = constraint->getRelativeTransform();

  Eigen::Vector3d translationError
      = relTransform.translation() - expectedRelTransform.translation();
  EXPECT_LT(translationError.norm(), 3.0);
}

TEST(WeldJointConstraint, SimulateWithOneBody)
{
  auto world = World::create();

  auto skel = createFreeSkeleton("skel");
  world->addSkeleton(skel);

  auto body = skel->getBodyNode(0);

  skel->setPosition(3, 0.5);
  skel->setPosition(4, 0.5);

  auto constraint = std::make_shared<WeldJointConstraint>(body);
  world->getConstraintSolver()->addConstraint(constraint);

  Eigen::Isometry3d initialPose = body->getTransform();

  for (int i = 0; i < 100; ++i) {
    world->step();
  }

  Eigen::Isometry3d finalPose = body->getTransform();
  Eigen::Vector3d poseError
      = finalPose.translation() - initialPose.translation();

  EXPECT_LT(poseError.norm(), 1.0);
}

TEST(WeldJointConstraint, RemoveConstraint)
{
  auto world = World::create();

  auto skel1 = createFreeSkeleton("skel1");
  auto skel2 = createFreeSkeleton("skel2");
  world->addSkeleton(skel1);
  world->addSkeleton(skel2);

  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);

  auto constraint = std::make_shared<WeldJointConstraint>(body1, body2);

  world->getConstraintSolver()->addConstraint(constraint);
  EXPECT_EQ(world->getConstraintSolver()->getNumConstraints(), 1u);

  world->getConstraintSolver()->removeConstraint(constraint);
  EXPECT_EQ(world->getConstraintSolver()->getNumConstraints(), 0u);
}

TEST(WeldJointConstraint, SameSkeletonTwoBodies)
{
  auto world = World::create();

  auto skel = Skeleton::create("skel");
  auto [joint1, body1] = skel->createJointAndBodyNodePair<FreeJoint>();
  auto [joint2, body2] = skel->createJointAndBodyNodePair<FreeJoint>(body1);

  world->addSkeleton(skel);

  auto constraint = std::make_shared<WeldJointConstraint>(body1, body2);

  world->getConstraintSolver()->addConstraint(constraint);

  for (int i = 0; i < 10; ++i) {
    world->step();
  }

  EXPECT_EQ(constraint->getBodyNode1(), body1);
  EXPECT_EQ(constraint->getBodyNode2(), body2);
}

TEST(WeldJointConstraint, MultipleConstraints)
{
  auto world = World::create();

  auto skel1 = createFreeSkeleton("skel1");
  auto skel2 = createFreeSkeleton("skel2");
  auto skel3 = createFreeSkeleton("skel3");
  world->addSkeleton(skel1);
  world->addSkeleton(skel2);
  world->addSkeleton(skel3);

  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);
  auto body3 = skel3->getBodyNode(0);

  auto constraint1 = std::make_shared<WeldJointConstraint>(body1, body2);
  auto constraint2 = std::make_shared<WeldJointConstraint>(body2, body3);

  world->getConstraintSolver()->addConstraint(constraint1);
  world->getConstraintSolver()->addConstraint(constraint2);

  EXPECT_EQ(world->getConstraintSolver()->getNumConstraints(), 2u);

  for (int i = 0; i < 10; ++i) {
    world->step();
  }

  world->getConstraintSolver()->removeAllConstraints();
  EXPECT_EQ(world->getConstraintSolver()->getNumConstraints(), 0u);
}

TEST(WeldJointConstraint, RelativeTransformMaintained)
{
  auto world = World::create();

  auto skel1 = createFreeSkeleton("skel1");
  auto skel2 = createFreeSkeleton("skel2");
  world->addSkeleton(skel1);
  world->addSkeleton(skel2);

  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);

  skel2->setPosition(3, 2.0);
  skel2->setPosition(4, 1.0);

  auto constraint = std::make_shared<WeldJointConstraint>(body1, body2);

  Eigen::Isometry3d relativeTransform = Eigen::Isometry3d::Identity();
  relativeTransform.translation() = Eigen::Vector3d(2.0, 1.0, 0.0);
  constraint->setRelativeTransform(relativeTransform);

  world->getConstraintSolver()->addConstraint(constraint);

  for (int i = 0; i < 50; ++i) {
    world->step();
  }

  Eigen::Isometry3d storedRelTransform = constraint->getRelativeTransform();
  EXPECT_TRUE(storedRelTransform.isApprox(relativeTransform));
}

TEST(WeldJointConstraint, WorldStepMaintainsRelativeTransform)
{
  auto world = World::create();
  world->setGravity(Eigen::Vector3d::Zero());

  auto skel1 = createFreeSkeleton("skel1");
  auto skel2 = createFreeSkeleton("skel2");
  world->addSkeleton(skel1);
  world->addSkeleton(skel2);

  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);

  skel2->setPosition(0, 0.3);
  skel2->setPosition(3, 1.2);
  skel2->setPosition(4, -0.4);

  auto constraint = std::make_shared<WeldJointConstraint>(body1, body2);
  Eigen::Isometry3d desired = Eigen::Isometry3d::Identity();
  desired.linear()
      = Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitY()).toRotationMatrix();
  desired.translation() = Eigen::Vector3d(0.4, -0.2, 0.1);
  constraint->setRelativeTransform(desired);

  world->getConstraintSolver()->addConstraint(constraint);

  skel2->setPosition(3, skel2->getPosition(3) + 0.3);
  skel2->setPosition(4, skel2->getPosition(4) - 0.2);

  Eigen::Isometry3d initialRel
      = body1->getTransform().inverse() * body2->getTransform();
  Eigen::Isometry3d initialError = desired.inverse() * initialRel;
  const double initialTransError = initialError.translation().norm();
  EXPECT_GT(initialTransError, 0.0);

  for (int i = 0; i < 200; ++i) {
    world->step();
  }

  Eigen::Isometry3d finalRel
      = body1->getTransform().inverse() * body2->getTransform();
  Eigen::Isometry3d finalError = desired.inverse() * finalRel;
  const double finalTransError = finalError.translation().norm();
  const double finalAngleError = Eigen::AngleAxisd(finalError.linear()).angle();

  EXPECT_LT(finalTransError, initialTransError);
  EXPECT_FALSE(finalAngleError != finalAngleError);
}
