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

#include <algorithm>

#include <cmath>

namespace {

using dart::dynamics::BodyNode;
using dart::dynamics::EulerJoint;
using dart::dynamics::FreeJoint;
using dart::dynamics::RevoluteJoint;
using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;
using dart::dynamics::TranslationalJoint;

double finiteDifferenceStep(double value)
{
  return 1e-6 * std::max(1.0, std::abs(value));
}

SkeletonPtr makeTestSkeleton()
{
  SkeletonPtr skeleton = Skeleton::create("test");

  BodyNode::Properties bodyProps;
  bodyProps.mName = "root";

  TranslationalJoint::Properties transProps;
  transProps.mName = "trans";
  auto rootPair
      = skeleton->createJointAndBodyNodePair<TranslationalJoint, BodyNode>(
          nullptr, transProps, bodyProps);
  BodyNode* root = rootPair.second;
  root->setName("root_body");

  EulerJoint::Properties eulerProps;
  eulerProps.mName = "euler";
  BodyNode::Properties childProps;
  childProps.mName = "child_body";
  skeleton->createJointAndBodyNodePair<EulerJoint, BodyNode>(
      root, eulerProps, childProps);

  return skeleton;
}

SkeletonPtr createChainSkeleton(const std::string& name, int numBodies)
{
  auto skeleton = Skeleton::create(name);
  BodyNode* parent = nullptr;

  for (int i = 0; i < numBodies; ++i) {
    BodyNode::Properties bodyProps;
    bodyProps.mName = "body" + std::to_string(i);

    if (i == 0) {
      auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>(
          nullptr, FreeJoint::Properties(), bodyProps);
      parent = pair.second;
    } else {
      RevoluteJoint::Properties jointProps;
      jointProps.mName = "joint" + std::to_string(i);
      jointProps.mAxis = Eigen::Vector3d::UnitZ();
      auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>(
          parent, jointProps, bodyProps);
      parent = pair.second;
    }
  }

  return skeleton;
}

Eigen::Matrix4d numericalFirstDerivative(
    const SkeletonPtr& skeleton,
    BodyNode* body,
    const Eigen::VectorXd& basePositions,
    std::size_t dependentIndex,
    double step)
{
  const std::size_t skeletonIndex
      = body->getDependentGenCoordIndex(dependentIndex);

  Eigen::VectorXd posPlus = basePositions;
  Eigen::VectorXd posMinus = basePositions;
  posPlus[skeletonIndex] += step;
  posMinus[skeletonIndex] -= step;

  skeleton->setPositions(posPlus);
  const Eigen::Matrix4d T_plus = body->getWorldTransform().matrix();

  skeleton->setPositions(posMinus);
  const Eigen::Matrix4d T_minus = body->getWorldTransform().matrix();

  skeleton->setPositions(basePositions);
  return (T_plus - T_minus) / (2.0 * step);
}

Eigen::Matrix4d numericalSecondDerivative(
    const SkeletonPtr& skeleton,
    BodyNode* body,
    const Eigen::VectorXd& basePositions,
    std::size_t dependentIndexI,
    std::size_t dependentIndexJ,
    double stepI,
    double stepJ)
{
  const std::size_t skeletonIndexI
      = body->getDependentGenCoordIndex(dependentIndexI);
  const std::size_t skeletonIndexJ
      = body->getDependentGenCoordIndex(dependentIndexJ);

  skeleton->setPositions(basePositions);
  const Eigen::Matrix4d T0 = body->getWorldTransform().matrix();

  if (dependentIndexI == dependentIndexJ) {
    Eigen::VectorXd posPlus = basePositions;
    Eigen::VectorXd posMinus = basePositions;
    posPlus[skeletonIndexI] += stepI;
    posMinus[skeletonIndexI] -= stepI;

    skeleton->setPositions(posPlus);
    const Eigen::Matrix4d T_plus = body->getWorldTransform().matrix();

    skeleton->setPositions(posMinus);
    const Eigen::Matrix4d T_minus = body->getWorldTransform().matrix();

    skeleton->setPositions(basePositions);
    return (T_plus - 2.0 * T0 + T_minus) / (stepI * stepI);
  }

  Eigen::VectorXd posPP = basePositions;
  Eigen::VectorXd posPM = basePositions;
  Eigen::VectorXd posMP = basePositions;
  Eigen::VectorXd posMM = basePositions;

  posPP[skeletonIndexI] += stepI;
  posPP[skeletonIndexJ] += stepJ;

  posPM[skeletonIndexI] += stepI;
  posPM[skeletonIndexJ] -= stepJ;

  posMP[skeletonIndexI] -= stepI;
  posMP[skeletonIndexJ] += stepJ;

  posMM[skeletonIndexI] -= stepI;
  posMM[skeletonIndexJ] -= stepJ;

  skeleton->setPositions(posPP);
  const Eigen::Matrix4d T_pp = body->getWorldTransform().matrix();

  skeleton->setPositions(posPM);
  const Eigen::Matrix4d T_pm = body->getWorldTransform().matrix();

  skeleton->setPositions(posMP);
  const Eigen::Matrix4d T_mp = body->getWorldTransform().matrix();

  skeleton->setPositions(posMM);
  const Eigen::Matrix4d T_mm = body->getWorldTransform().matrix();

  skeleton->setPositions(basePositions);
  return (T_pp - T_pm - T_mp + T_mm) / (4.0 * stepI * stepJ);
}

} // namespace

TEST(BodyNodeDerivatives, FirstOrderMatchesFiniteDifference)
{
  const SkeletonPtr skeleton = makeTestSkeleton();
  ASSERT_EQ(skeleton->getNumDofs(), 6u);

  Eigen::VectorXd positions = Eigen::VectorXd::Random(skeleton->getNumDofs());
  positions *= 0.2;
  skeleton->setPositions(positions);

  const double tolerance = 1e-6;

  std::vector<BodyNode*> bodies
      = {skeleton->getRootBodyNode(),
         skeleton->getRootBodyNode()->getChildBodyNode(0)};

  for (BodyNode* body : bodies) {
    const std::size_t depDofs = body->getNumDependentGenCoords();
    for (std::size_t i = 0; i < depDofs; ++i) {
      const std::size_t skeletonIndex = body->getDependentGenCoordIndex(i);
      const double step = finiteDifferenceStep(positions[skeletonIndex]);

      const Eigen::Matrix4d analytic = body->getWorldTransformDerivative(i);
      const Eigen::Matrix4d numeric
          = numericalFirstDerivative(skeleton, body, positions, i, step);

      const double error = (analytic - numeric).lpNorm<Eigen::Infinity>();
      EXPECT_LT(error, tolerance) << "First derivative mismatch at index " << i
                                  << " for body " << body->getName();
    }
  }
}

TEST(BodyNodeDerivatives, SecondOrderMatchesFiniteDifference)
{
  const SkeletonPtr skeleton = makeTestSkeleton();
  ASSERT_EQ(skeleton->getNumDofs(), 6u);

  Eigen::VectorXd positions = Eigen::VectorXd::Random(skeleton->getNumDofs());
  positions *= 0.2;
  skeleton->setPositions(positions);

  BodyNode* body = skeleton->getRootBodyNode()->getChildBodyNode(0);
  ASSERT_NE(body, nullptr);

  const std::size_t depDofs = body->getNumDependentGenCoords();
  const double tolerance = 2e-4;

  for (std::size_t i = 0; i < depDofs; ++i) {
    const std::size_t skeletonIndexI = body->getDependentGenCoordIndex(i);
    const double stepI = finiteDifferenceStep(positions[skeletonIndexI]);
    for (std::size_t j = 0; j < depDofs; ++j) {
      const std::size_t skeletonIndexJ = body->getDependentGenCoordIndex(j);
      const double stepJ = finiteDifferenceStep(positions[skeletonIndexJ]);

      const Eigen::Matrix4d analytic
          = body->getWorldTransformSecondDerivative(i, j);
      const Eigen::Matrix4d numeric = numericalSecondDerivative(
          skeleton, body, positions, i, j, stepI, stepJ);

      const double error = (analytic - numeric).lpNorm<Eigen::Infinity>();
      EXPECT_LT(error, tolerance)
          << "Second derivative mismatch at indices (" << i << ", " << j
          << ") for body " << body->getName();
    }
  }
}

TEST(BodyNodeDerivatives, JacobianUpdates)
{
  const SkeletonPtr skeleton = createChainSkeleton("jac_chain", 3);
  ASSERT_EQ(skeleton->getNumDofs(), 8u);

  Eigen::VectorXd positions = Eigen::VectorXd::Random(skeleton->getNumDofs());
  Eigen::VectorXd velocities = Eigen::VectorXd::Random(skeleton->getNumDofs());
  skeleton->setPositions(positions * 0.1);
  skeleton->setVelocities(velocities * 0.1);

  BodyNode* body = skeleton->getBodyNode(2);
  ASSERT_NE(body, nullptr);

  const auto jacobian = body->getJacobian();
  EXPECT_EQ(
      jacobian.cols(), static_cast<int>(body->getNumDependentGenCoords()));

  const auto worldJacobian = body->getWorldJacobian();
  EXPECT_EQ(worldJacobian.cols(), jacobian.cols());

  const auto spatialDeriv = body->getJacobianSpatialDeriv();
  EXPECT_EQ(spatialDeriv.cols(), jacobian.cols());

  const auto classicDeriv = body->getJacobianClassicDeriv();
  EXPECT_EQ(classicDeriv.cols(), jacobian.cols());
}

TEST(BodyNodeDerivatives, ForcesAndImpulses)
{
  auto skeleton = Skeleton::create("forces");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* body = pair.second;
  auto* joint = pair.first;

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = Eigen::AngleAxisd(dart::math::half_pi, Eigen::Vector3d::UnitZ())
                    .toRotationMatrix();
  tf.translation() = Eigen::Vector3d(0.2, -0.1, 0.3);
  joint->setTransform(tf);

  body->clearExternalForces();
  body->addExtForce(
      Eigen::Vector3d(1.0, 0.5, -0.2),
      Eigen::Vector3d(0.1, 0.0, 0.2),
      false,
      false);
  body->addExtTorque(Eigen::Vector3d(0.1, -0.2, 0.3), false);

  const Eigen::Vector6d extLocal = body->getExternalForceLocal();
  EXPECT_TRUE(extLocal.array().isFinite().all());
  EXPECT_GT(extLocal.norm(), 0.0);

  const Eigen::Vector6d extGlobal = body->getExternalForceGlobal();
  EXPECT_TRUE(extGlobal.array().isFinite().all());
  EXPECT_GT(extGlobal.norm(), 0.0);

  body->clearConstraintImpulse();
  body->addConstraintImpulse(
      Eigen::Vector3d(0.3, 0.2, -0.1),
      Eigen::Vector3d(0.0, 0.1, 0.0),
      false,
      false);
  EXPECT_TRUE(body->getConstraintImpulse().array().isFinite().all());
  EXPECT_GT(body->getConstraintImpulse().norm(), 0.0);

  body->clearPositionConstraintImpulse();
  body->addPositionConstraintImpulse(Eigen::Vector6d::Ones());
  EXPECT_FALSE(body->getPositionConstraintImpulse().isZero());

  const Eigen::Vector6d& bodyForce = body->getBodyForce();
  EXPECT_TRUE(bodyForce.array().isFinite().all());
}

TEST(BodyNodeDerivatives, MoveCopyAgreementFailures)
{
  auto skelA = createChainSkeleton("skelA", 2);
  auto skelB = createChainSkeleton("skelB", 2);

  BodyNode* bodyA = skelA->getBodyNode("body1");
  BodyNode* bodyB = skelB->getBodyNode("body1");
  ASSERT_NE(bodyA, nullptr);
  ASSERT_NE(bodyB, nullptr);

  EXPECT_FALSE(bodyA->moveTo(skelB, bodyA));

  const auto copyResult = bodyA->copyTo(skelB, bodyA, false);
  EXPECT_EQ(copyResult.first, nullptr);
  EXPECT_EQ(copyResult.second, nullptr);

  SkeletonPtr nullSkeleton;
  EXPECT_FALSE(bodyA->moveTo(nullSkeleton, bodyB));
}

TEST(BodyNodeDerivatives, DerivativesAndSimulationPaths)
{
  auto skeleton = createChainSkeleton("dyn", 2);
  ASSERT_EQ(skeleton->getNumDofs(), 7u);

  Eigen::VectorXd positions = Eigen::VectorXd::Zero(7);
  positions << 0.1, -0.2, 0.3, 0.0, 0.2, -0.1, 0.4;
  Eigen::VectorXd velocities = Eigen::VectorXd::Constant(7, 0.05);
  Eigen::VectorXd accelerations = Eigen::VectorXd::Constant(7, -0.02);

  skeleton->setPositions(positions);
  skeleton->setVelocities(velocities);
  skeleton->setAccelerations(accelerations);

  BodyNode* body = skeleton->getBodyNode("body1");
  ASSERT_NE(body, nullptr);

  const auto& dT0 = body->getWorldTransformDerivative(0);
  const auto& dT1 = body->getWorldTransformDerivative(1);
  const auto& dT01 = body->getWorldTransformSecondDerivative(0, 1);
  EXPECT_TRUE(dT0.array().isFinite().all());
  EXPECT_TRUE(dT1.array().isFinite().all());
  EXPECT_TRUE(dT01.array().isFinite().all());

  EXPECT_TRUE(body->getRelativeSpatialVelocity().array().isFinite().all());
  EXPECT_TRUE(body->getRelativeSpatialAcceleration().array().isFinite().all());
  EXPECT_TRUE(body->getPrimaryRelativeAcceleration().array().isFinite().all());
  EXPECT_TRUE(body->getPartialAcceleration().array().isFinite().all());

  const auto& jac = body->getJacobian();
  const auto& worldJac = body->getWorldJacobian();
  const auto& jacDeriv = body->getJacobianSpatialDeriv();
  const auto& jacClassic = body->getJacobianClassicDeriv();
  EXPECT_EQ(jac.rows(), 6);
  EXPECT_EQ(worldJac.rows(), 6);
  EXPECT_EQ(jacDeriv.rows(), 6);
  EXPECT_EQ(jacClassic.rows(), 6);

  body->dirtyTransform();
  body->dirtyVelocity();
  body->dirtyAcceleration();
  EXPECT_TRUE(body->getWorldTransform().matrix().array().isFinite().all());

  auto world = dart::simulation::World::create();
  world->addSkeleton(skeleton);
  world->step();
  EXPECT_TRUE(body->getBodyForce().array().isFinite().all());
}

TEST(BodyNodeDerivatives, NodeStatePropertyCoverage)
{
  auto skeleton = Skeleton::create("node_state_props");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* bodyA = rootPair.second;
  auto childPair = bodyA->createChildJointAndBodyNodePair<RevoluteJoint>();
  auto* bodyB = childPair.second;

  auto shape
      = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d::Ones());
  bodyA->createShapeNodeWith<dart::dynamics::VisualAspect>(shape);
  bodyB->createEndEffector("ee");

  auto statesA = bodyA->getAllNodeStates();
  auto statesB = bodyB->getAllNodeStates();

  bodyA->setAllNodeStates(statesA);
  bodyB->setAllNodeStates(statesB);
  bodyA->setAllNodeStates(statesB);
  bodyB->setAllNodeStates(statesA);

  auto propsA = bodyA->getAllNodeProperties();
  auto propsB = bodyB->getAllNodeProperties();

  bodyA->setAllNodeProperties(propsA);
  bodyB->setAllNodeProperties(propsB);
  bodyA->setAllNodeProperties(propsB);
  bodyB->setAllNodeProperties(propsA);

  auto aspectState = bodyA->getAspectState();
  aspectState.mFext = Eigen::Vector6d::Ones();
  bodyA->setAspectState(aspectState);

  bodyA->copy(*bodyA);
  bodyA->setName(bodyA->getName());

  const auto nodes = bodyA->getNodes();
  const BodyNode* constBody = bodyA;
  const auto constNodes = constBody->getNodes();
  EXPECT_EQ(nodes.size(), constNodes.size());
}
