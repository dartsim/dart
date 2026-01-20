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

#include <dart/All.hpp>

#include <gtest/gtest.h>

#include <algorithm>

#include <cmath>

namespace {

using dart::dynamics::BodyNode;
using dart::dynamics::EulerJoint;
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
