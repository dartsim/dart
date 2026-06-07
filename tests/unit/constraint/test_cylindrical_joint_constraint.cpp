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

#include <dart/all.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <cmath>

using namespace dart;
using namespace dart::constraint;
using namespace dart::dynamics;
using namespace dart::simulation;

namespace {

struct FreeBody
{
  SkeletonPtr skeleton;
  FreeJoint* joint;
  BodyNode* body;
};

struct TwoFreeBodies
{
  SkeletonPtr skeleton;
  FreeJoint* joint1;
  BodyNode* body1;
  FreeJoint* joint2;
  BodyNode* body2;
};

struct WeldAndFreeBodies
{
  SkeletonPtr skeleton;
  WeldJoint* weldJoint;
  BodyNode* staticBody;
  FreeJoint* freeJoint;
  BodyNode* reactiveBody;
};

struct FreeAndWeldRootBodies
{
  SkeletonPtr skeleton;
  FreeJoint* freeJoint;
  BodyNode* reactiveBody;
  WeldJoint* weldJoint;
  BodyNode* staticBody;
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

FreeBody createFreeBody(const std::string& name)
{
  FreeBody result;
  result.skeleton = Skeleton::create(name);
  auto pair = result.skeleton->createJointAndBodyNodePair<FreeJoint>();
  result.joint = pair.first;
  result.body = pair.second;
  result.body->setMass(1.0);
  return result;
}

TwoFreeBodies createTwoFreeBodies(const std::string& name)
{
  TwoFreeBodies result;
  result.skeleton = Skeleton::create(name);

  auto rootPair = result.skeleton->createJointAndBodyNodePair<FreeJoint>();
  result.joint1 = rootPair.first;
  result.body1 = rootPair.second;
  result.body1->setMass(1.0);

  auto childPair = result.body1->createChildJointAndBodyNodePair<FreeJoint>();
  result.joint2 = childPair.first;
  result.body2 = childPair.second;
  result.body2->setMass(1.0);

  return result;
}

WeldAndFreeBodies createWeldAndFreeBodies(const std::string& name)
{
  WeldAndFreeBodies result;
  result.skeleton = Skeleton::create(name);

  auto weldPair = result.skeleton->createJointAndBodyNodePair<WeldJoint>();
  result.weldJoint = weldPair.first;
  result.staticBody = weldPair.second;
  result.staticBody->setMass(1.0);

  auto freePair
      = result.staticBody->createChildJointAndBodyNodePair<FreeJoint>();
  result.freeJoint = freePair.first;
  result.reactiveBody = freePair.second;
  result.reactiveBody->setMass(1.0);

  return result;
}

FreeAndWeldRootBodies createFreeAndWeldRootBodies(const std::string& name)
{
  FreeAndWeldRootBodies result;
  result.skeleton = Skeleton::create(name);

  auto freePair = result.skeleton->createJointAndBodyNodePair<FreeJoint>();
  result.freeJoint = freePair.first;
  result.reactiveBody = freePair.second;
  result.reactiveBody->setMass(1.0);

  auto weldPair = result.skeleton->createJointAndBodyNodePair<WeldJoint>();
  result.weldJoint = weldPair.first;
  result.staticBody = weldPair.second;
  result.staticBody->setMass(1.0);

  return result;
}

void setTransform(FreeJoint* joint, const Eigen::Isometry3d& transform)
{
  joint->setPositions(FreeJoint::convertToPositions(transform));
}

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
}

double vectorNorm(const std::vector<double>& values, std::size_t begin, int dim)
{
  double squaredNorm = 0.0;
  for (int i = 0; i < dim; ++i) {
    squaredNorm += values[begin + i] * values[begin + i];
  }

  return std::sqrt(squaredNorm);
}

class ExposedCylindricalJointConstraint final
  : public CylindricalJointConstraint
{
public:
  using CylindricalJointConstraint::applyImpulse;
  using CylindricalJointConstraint::applyUnitImpulse;
  using CylindricalJointConstraint::CylindricalJointConstraint;
  using CylindricalJointConstraint::excite;
  using CylindricalJointConstraint::getInformation;
  using CylindricalJointConstraint::getVelocityChange;
  using CylindricalJointConstraint::isActive;
  using CylindricalJointConstraint::unexcite;
  using CylindricalJointConstraint::uniteSkeletons;
  using CylindricalJointConstraint::update;

  SkeletonPtr exposedGetRootSkeleton() const
  {
    return getRootSkeleton();
  }
};

} // namespace

TEST(CylindricalJointConstraint, CreateAndType)
{
  auto body1 = createFreeBody("body1");
  auto body2 = createFreeBody("body2");

  auto constraint = std::make_shared<CylindricalJointConstraint>(
      body1.body,
      body2.body,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d::UnitZ());

  EXPECT_EQ(constraint->getBodyNode1(), body1.body);
  EXPECT_EQ(constraint->getBodyNode2(), body2.body);
  EXPECT_EQ(constraint->getDimension(), 4u);
  EXPECT_EQ(constraint->getType(), CylindricalJointConstraint::getStaticType());
}

TEST(CylindricalJointConstraint, RowsLeaveAxisTranslationAndRotationFree)
{
  auto freeBody = createFreeBody("single_body");

  ExposedCylindricalJointConstraint constraint(
      freeBody.body, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ());

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  transform.translation() = Eigen::Vector3d(0.0, 0.0, 1.0);
  setTransform(freeBody.joint, transform);

  constraint.update();

  ConstraintInfo info;
  LcpBuffers buffers;
  prepareConstraintInfo(info, buffers, constraint.getDimension(), 1000.0);
  constraint.getInformation(&info);

  EXPECT_NEAR(vectorNorm(buffers.b, 0, 2), 0.0, 1e-9);
  EXPECT_NEAR(vectorNorm(buffers.b, 2, 2), 0.0, 1e-9);

  transform.translation() = Eigen::Vector3d(0.4, 0.0, 1.0);
  setTransform(freeBody.joint, transform);
  constraint.update();
  prepareConstraintInfo(info, buffers, constraint.getDimension(), 1000.0);
  constraint.getInformation(&info);

  EXPECT_GT(vectorNorm(buffers.b, 0, 2), 1e-3);
  EXPECT_NEAR(vectorNorm(buffers.b, 2, 2), 0.0, 1e-9);

  transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()).toRotationMatrix();
  transform.translation() = Eigen::Vector3d::Zero();
  setTransform(freeBody.joint, transform);
  constraint.update();
  prepareConstraintInfo(info, buffers, constraint.getDimension(), 1000.0);
  constraint.getInformation(&info);

  EXPECT_NEAR(vectorNorm(buffers.b, 0, 2), 0.0, 1e-9);
  EXPECT_GT(vectorNorm(buffers.b, 2, 2), 1e-3);
}

TEST(CylindricalJointConstraint, XAlignedAxisUsesStablePerpendicularBasis)
{
  auto freeBody = createFreeBody("x_axis_body");

  ExposedCylindricalJointConstraint constraint(
      freeBody.body, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.75, 0.0, 0.0);
  setTransform(freeBody.joint, transform);

  constraint.update();

  ConstraintInfo info;
  LcpBuffers buffers;
  prepareConstraintInfo(info, buffers, constraint.getDimension(), 1000.0);
  constraint.getInformation(&info);

  EXPECT_NEAR(vectorNorm(buffers.b, 0, 2), 0.0, 1e-9);

  transform.translation() = Eigen::Vector3d(0.75, 0.2, -0.3);
  setTransform(freeBody.joint, transform);
  constraint.update();
  prepareConstraintInfo(info, buffers, constraint.getDimension(), 1000.0);
  constraint.getInformation(&info);

  EXPECT_GT(vectorNorm(buffers.b, 0, 2), 1e-3);
}

TEST(CylindricalJointConstraint, CallbackPipelineAndWarmStart)
{
  auto body1 = createFreeBody("callbacks_1");
  auto body2 = createFreeBody("callbacks_2");

  ExposedCylindricalJointConstraint constraint(
      body1.body,
      body2.body,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d::UnitZ());
  constraint.update();

  ASSERT_TRUE(constraint.isActive());
  ASSERT_EQ(constraint.getDimension(), 4u);
  EXPECT_EQ(constraint.exposedGetRootSkeleton(), body1.skeleton);

  ConstraintInfo info;
  LcpBuffers buffers;
  prepareConstraintInfo(info, buffers, constraint.getDimension(), 1000.0);
  constraint.getInformation(&info);
  for (double value : buffers.b) {
    EXPECT_TRUE(std::isfinite(value));
  }

  constraint.applyUnitImpulse(0);
  body1.skeleton->computeForwardDynamics();
  body2.skeleton->computeForwardDynamics();
  constraint.excite();
  EXPECT_TRUE(body1.skeleton->isImpulseApplied());
  EXPECT_TRUE(body2.skeleton->isImpulseApplied());

  std::vector<double> vel(constraint.getDimension(), 0.0);
  constraint.getVelocityChange(vel.data(), true);
  for (double value : vel) {
    EXPECT_TRUE(std::isfinite(value));
  }

  std::vector<double> lambda = {0.1, -0.2, 0.3, -0.4};
  constraint.applyImpulse(lambda.data());
  constraint.unexcite();
  EXPECT_FALSE(body1.skeleton->isImpulseApplied());
  EXPECT_FALSE(body2.skeleton->isImpulseApplied());

  prepareConstraintInfo(info, buffers, constraint.getDimension(), 1000.0);
  constraint.getInformation(&info);
  EXPECT_DOUBLE_EQ(buffers.x[0], 0.1);
  EXPECT_DOUBLE_EQ(buffers.x[1], -0.2);
  EXPECT_DOUBLE_EQ(buffers.x[2], 0.3);
  EXPECT_DOUBLE_EQ(buffers.x[3], -0.4);

  constraint.uniteSkeletons();
  EXPECT_EQ(body2.skeleton->mUnionRootSkeleton.lock(), body1.skeleton);
}

TEST(CylindricalJointConstraint, SingleBodyCallbacksUseWorldAxisFallback)
{
  auto body = createFreeBody("single_body_callbacks");

  ExposedCylindricalJointConstraint constraint(
      body.body, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  transform.translation() = Eigen::Vector3d(0.0, 0.0, 1.0);
  setTransform(body.joint, transform);

  constraint.update();

  ConstraintInfo info;
  LcpBuffers buffers;
  prepareConstraintInfo(info, buffers, constraint.getDimension(), 1000.0);
  constraint.getInformation(&info);

  EXPECT_NEAR(vectorNorm(buffers.b, 0, 2), 0.0, 1e-9);
  EXPECT_NEAR(vectorNorm(buffers.b, 2, 2), 0.0, 1e-9);

  constraint.applyUnitImpulse(3);
  body.skeleton->computeForwardDynamics();
  constraint.excite();
  EXPECT_TRUE(body.skeleton->isImpulseApplied());

  std::vector<double> vel(constraint.getDimension(), 0.0);
  constraint.getVelocityChange(vel.data(), true);
  for (double value : vel) {
    EXPECT_TRUE(std::isfinite(value));
  }

  std::vector<double> lambda = {0.4, -0.3, 0.2, -0.1};
  constraint.applyImpulse(lambda.data());
  constraint.unexcite();
  EXPECT_FALSE(body.skeleton->isImpulseApplied());

  prepareConstraintInfo(info, buffers, constraint.getDimension(), 1000.0);
  constraint.getInformation(&info);
  EXPECT_DOUBLE_EQ(buffers.x[0], 0.4);
  EXPECT_DOUBLE_EQ(buffers.x[1], -0.3);
  EXPECT_DOUBLE_EQ(buffers.x[2], 0.2);
  EXPECT_DOUBLE_EQ(buffers.x[3], -0.1);
}

TEST(CylindricalJointConstraint, SingleBodyUniteSkeletonsIsNoOp)
{
  auto body = createFreeBody("single_body_unite");

  ExposedCylindricalJointConstraint constraint(
      body.body, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ());

  EXPECT_TRUE(constraint.isActive());
  constraint.uniteSkeletons();
  EXPECT_EQ(body.skeleton->mUnionRootSkeleton.lock(), body.skeleton);
}

TEST(CylindricalJointConstraint, StaticBody1UsesBody2ReactiveBranches)
{
  auto body1 = createFreeBody("static_callbacks_1");
  auto body2 = createFreeBody("static_callbacks_2");
  body1.skeleton->setMobile(false);

  ExposedCylindricalJointConstraint constraint(
      body1.body,
      body2.body,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d::UnitZ());
  constraint.update();

  ASSERT_FALSE(body1.body->isReactive());
  ASSERT_TRUE(body2.body->isReactive());
  EXPECT_TRUE(constraint.isActive());
  EXPECT_EQ(constraint.exposedGetRootSkeleton(), body2.skeleton);

  constraint.applyUnitImpulse(1);
  body2.skeleton->computeForwardDynamics();
  constraint.excite();
  EXPECT_FALSE(body1.skeleton->isImpulseApplied());
  EXPECT_TRUE(body2.skeleton->isImpulseApplied());

  std::vector<double> vel(constraint.getDimension(), 0.0);
  constraint.getVelocityChange(vel.data(), true);
  for (double value : vel) {
    EXPECT_TRUE(std::isfinite(value));
  }

  constraint.uniteSkeletons();
  constraint.unexcite();
  EXPECT_FALSE(body1.skeleton->isImpulseApplied());
  EXPECT_FALSE(body2.skeleton->isImpulseApplied());
}

TEST(CylindricalJointConstraint, NonReactiveFirstBodyUsesReactiveSecondBody)
{
  auto bodies = createWeldAndFreeBodies("non_reactive_first_cylindrical_body");

  ASSERT_NE(bodies.staticBody, nullptr);
  ASSERT_NE(bodies.reactiveBody, nullptr);
  ASSERT_FALSE(bodies.staticBody->isReactive());
  ASSERT_TRUE(bodies.reactiveBody->isReactive());

  ExposedCylindricalJointConstraint constraint(
      bodies.staticBody,
      bodies.reactiveBody,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d::UnitZ());
  EXPECT_TRUE(constraint.isActive());
  EXPECT_EQ(constraint.exposedGetRootSkeleton(), bodies.skeleton);
  constraint.update();

  constraint.applyUnitImpulse(0);
  bodies.skeleton->computeForwardDynamics();

  constraint.excite();
  EXPECT_TRUE(bodies.skeleton->isImpulseApplied());

  std::vector<double> vel(constraint.getDimension(), 0.0);
  constraint.getVelocityChange(vel.data(), true);
  for (double value : vel) {
    EXPECT_TRUE(std::isfinite(value));
  }

  std::vector<double> lambda = {0.01, 0.02, 0.03, 0.04};
  constraint.applyImpulse(lambda.data());
  EXPECT_TRUE(bodies.reactiveBody->getConstraintImpulse().allFinite());

  constraint.unexcite();
  EXPECT_FALSE(bodies.skeleton->isImpulseApplied());

  constraint.uniteSkeletons();
  EXPECT_EQ(bodies.skeleton->mUnionRootSkeleton.lock(), bodies.skeleton);
}

TEST(CylindricalJointConstraint, ReactiveFirstBodyIgnoresStaticSecondBody)
{
  auto bodies = createFreeAndWeldRootBodies("reactive_first_cylindrical_body");

  ASSERT_NE(bodies.reactiveBody, nullptr);
  ASSERT_NE(bodies.staticBody, nullptr);
  ASSERT_TRUE(bodies.reactiveBody->isReactive());
  ASSERT_FALSE(bodies.staticBody->isReactive());

  ExposedCylindricalJointConstraint constraint(
      bodies.reactiveBody,
      bodies.staticBody,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d::UnitZ());
  EXPECT_TRUE(constraint.isActive());
  EXPECT_EQ(constraint.exposedGetRootSkeleton(), bodies.skeleton);
  constraint.update();

  constraint.applyUnitImpulse(1);
  bodies.skeleton->computeForwardDynamics();

  constraint.excite();
  EXPECT_TRUE(bodies.skeleton->isImpulseApplied());

  std::vector<double> vel(constraint.getDimension(), 0.0);
  constraint.getVelocityChange(vel.data(), true);
  for (double value : vel) {
    EXPECT_TRUE(std::isfinite(value));
  }

  std::vector<double> lambda = {-0.01, 0.02, -0.03, 0.04};
  constraint.applyImpulse(lambda.data());
  EXPECT_TRUE(bodies.reactiveBody->getConstraintImpulse().allFinite());

  constraint.unexcite();
  EXPECT_FALSE(bodies.skeleton->isImpulseApplied());

  constraint.uniteSkeletons();
  EXPECT_EQ(bodies.skeleton->mUnionRootSkeleton.lock(), bodies.skeleton);
}

TEST(CylindricalJointConstraint, SameSkeletonBodiesShareImpulseSolve)
{
  auto bodies = createTwoFreeBodies("same_skeleton_callbacks");

  ExposedCylindricalJointConstraint constraint(
      bodies.body1,
      bodies.body2,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d::UnitZ());
  constraint.update();

  ASSERT_TRUE(bodies.body1->isReactive());
  ASSERT_TRUE(bodies.body2->isReactive());

  constraint.applyUnitImpulse(2);
  bodies.skeleton->computeForwardDynamics();
  constraint.excite();
  EXPECT_TRUE(bodies.skeleton->isImpulseApplied());

  std::vector<double> vel(constraint.getDimension(), 0.0);
  constraint.getVelocityChange(vel.data(), false);
  for (double value : vel) {
    EXPECT_TRUE(std::isfinite(value));
  }

  constraint.uniteSkeletons();
  constraint.unexcite();
  EXPECT_FALSE(bodies.skeleton->isImpulseApplied());
}

TEST(CylindricalJointConstraint, StaticBodiesAreInactive)
{
  auto singleBody = createFreeBody("static_single_body");
  singleBody.skeleton->setMobile(false);

  ExposedCylindricalJointConstraint singleBodyConstraint(
      singleBody.body, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ());
  EXPECT_FALSE(singleBodyConstraint.isActive());
  singleBodyConstraint.uniteSkeletons();

  auto body1 = createFreeBody("static_body_1");
  auto body2 = createFreeBody("static_body_2");
  body1.skeleton->setMobile(false);
  body2.skeleton->setMobile(false);

  ExposedCylindricalJointConstraint twoBodyConstraint(
      body1.body,
      body2.body,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d::UnitZ());
  EXPECT_FALSE(twoBodyConstraint.isActive());
  twoBodyConstraint.uniteSkeletons();
}
