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

#include "TestHelpers.hpp"
#include "dart/common/Console.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/dynamics/EndEffector.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/HierarchicalIK.hpp"
#include "dart/dynamics/InverseKinematics.hpp"
#include "dart/dynamics/PrismaticJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/math/Constants.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/math/MathTypes.hpp"
#include "dart/optimizer/Problem.hpp"
#include "dart/optimizer/Solver.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace Eigen;
using namespace dart;
using namespace dart::dynamics;

//==============================================================================
TEST(InverseKinematics, SolveForFreeJoint)
{
  // Very simple test of InverseKinematics module, applied to a FreeJoint to
  // ensure that the target is reachable

  SkeletonPtr skel = Skeleton::create();
  skel->createJointAndBodyNodePair<FreeJoint>();

  std::shared_ptr<InverseKinematics> ik = skel->getBodyNode(0)->getIK(true);

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, 0.0, 0.8);
  tf.rotate(
      Eigen::AngleAxisd(math::constantsd::pi() / 8, Eigen::Vector3d(0, 1, 0)));
  ik->getTarget()->setTransform(tf);

  ik->getErrorMethod().setBounds(
      Eigen::Vector6d::Constant(-1e-8), Eigen::Vector6d::Constant(1e-8));

  ik->getSolver()->setNumMaxIterations(100);

  EXPECT_FALSE(equals(
      ik->getTarget()->getTransform().matrix(),
      skel->getBodyNode(0)->getTransform().matrix(),
      1e-1));

  EXPECT_TRUE(ik->getSolver()->solve());

  EXPECT_TRUE(equals(
      ik->getTarget()->getTransform().matrix(),
      skel->getBodyNode(0)->getTransform().matrix(),
      1e-8));
}

//==============================================================================
class FailingSolver : public optimizer::Solver
{
public:
  FailingSolver(double constant) : mConstant(constant)
  {
    // Do nothing
  }

  ~FailingSolver() override = default;

  bool solve() override
  {
    std::shared_ptr<optimizer::Problem> problem = mProperties.mProblem;
    if (nullptr == problem) {
      dtwarn << "[FailingSolver::solve] Attempting to solve a nullptr "
             << "problem! We will return false.\n";
      return false;
    }

    const auto dim = problem->getDimension();
    const Eigen::VectorXd wrongSolution
        = Eigen::VectorXd::Constant(static_cast<int>(dim), mConstant);
    problem->setOptimalSolution(wrongSolution);

    return false;
  }

  std::string getType() const override
  {
    return "FailingSolver";
  }

  std::shared_ptr<Solver> clone() const override
  {
    return std::make_shared<FailingSolver>(mConstant);
  }

protected:
  double mConstant;
};

//==============================================================================
TEST(InverseKinematics, DoNotApplySolutionOnFailure)
{
  SkeletonPtr skel = Skeleton::create();
  skel->createJointAndBodyNodePair<FreeJoint>();

  std::shared_ptr<InverseKinematics> ik = skel->getBodyNode(0)->getIK(true);
  ik->setSolver(std::make_shared<FailingSolver>(10));
  ik->getTarget()->setTransform(Eigen::Isometry3d::Identity());

  const auto dofs = static_cast<int>(skel->getNumDofs());
  skel->resetPositions();

  EXPECT_FALSE(ik->solveAndApply(false));
  EXPECT_TRUE(equals(skel->getPositions(), Eigen::VectorXd::Zero(dofs).eval()));

  EXPECT_FALSE(ik->solveAndApply(true));
  EXPECT_FALSE(
      equals(skel->getPositions(), Eigen::VectorXd::Zero(dofs).eval()));
}

//==============================================================================
TEST(InverseKinematics, CloneWithoutProblemObjective)
{
  SkeletonPtr skel = Skeleton::create();
  skel->createJointAndBodyNodePair<FreeJoint>();

  auto ik = skel->getBodyNode(0)->getIK(true);
  ik->getProblem()->clearObjective();
  auto ikClone = ik->clone(skel->getBodyNode(0));
  EXPECT_EQ(ikClone->getProblem()->getObjective(), nullptr);

  auto wholeBodyIk = skel->getIK(true);
  wholeBodyIk->getProblem()->clearObjective();
  auto wholeBodyClone = wholeBodyIk->clone(skel);
  EXPECT_EQ(wholeBodyClone->getProblem()->getObjective(), nullptr);
}

namespace {

//==============================================================================
class PrismaticIKCloneAnalytical : public InverseKinematics::Analytical
{
public:
  explicit PrismaticIKCloneAnalytical(
      InverseKinematics* ik, const Properties& properties = Properties())
    : Analytical(ik, "PrismaticIKCloneAnalytical", properties)
  {
  }

  std::unique_ptr<GradientMethod> clone(InverseKinematics* newIK) const override
  {
    return std::make_unique<PrismaticIKCloneAnalytical>(
        newIK, getAnalyticalProperties());
  }

  const std::vector<std::size_t>& getDofs() const override
  {
    return mDofs;
  }

  const std::vector<Solution>& computeSolutions(
      const Eigen::Isometry3d& desiredTf) override
  {
    mSolutions.clear();
    mSolutions.emplace_back(
        Eigen::VectorXd::Constant(1, desiredTf.translation().x()));
    checkSolutionJointLimits();
    return mSolutions;
  }

private:
  const std::vector<std::size_t> mDofs{1};
};

//==============================================================================
void checkPrismaticIKClone(bool useEndEffector, bool useAnalytical)
{
  auto original = Skeleton::create();
  original->createJointAndBodyNodePair<PrismaticJoint>().first->setAxis(
      Eigen::Vector3d::UnitY());
  auto pair = original->createJointAndBodyNodePair<PrismaticJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitX());
  original->setPosition(0, 0.2);
  original->setPosition(1, 0.3);
  const Eigen::VectorXd originalPositions = original->getPositions();

  JacobianNode* node = pair.second;
  if (useEndEffector)
    node = pair.second->createEndEffector("clone_ik");
  const Eigen::Isometry3d originalTransform = node->getWorldTransform();
  auto ik = node->getIK(true);
  if (useAnalytical)
    ik->setGradientMethod<PrismaticIKCloneAnalytical>();
  else
    ik->setGradientMethod<InverseKinematics::JacobianDLS>();
  ik->getErrorMethod().setBounds(
      Eigen::Vector6d::Constant(-1e-8), Eigen::Vector6d::Constant(1e-8));
  ik->getSolver()->setNumMaxIterations(100);

  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation().x() = 0.8;
  ik->getTarget()->setTransform(target);

  std::size_t originalTransformUpdates = 0;
  auto originalConnection = node->onTransformUpdated.connect(
      [&](const Entity*) { ++originalTransformUpdates; });

  auto clone = original->cloneSkeleton();
  JacobianNode* clonedNode = clone->getBodyNode(1);
  if (useEndEffector)
    clonedNode = clone->getEndEffector("clone_ik");
  ASSERT_NE(clonedNode, nullptr);
  auto clonedIK = clonedNode->getIK();
  ASSERT_NE(clonedIK, nullptr);
  EXPECT_NE(clonedIK, ik);
  EXPECT_EQ(clonedIK->getNode(), clonedNode);
  EXPECT_EQ(clonedIK->getGradientMethod().getIK(), clonedIK.get());
  EXPECT_EQ(clonedIK->getNode()->getSkeleton(), clone);
  EXPECT_EQ(clonedIK->getDofs(), ik->getDofs());
  EXPECT_EQ(clonedIK->getDofs(), std::vector<std::size_t>({1}));
  EXPECT_EQ(clonedIK->getDofMap(), std::vector<int>({0}));
  ASSERT_EQ(clonedNode->getNumDependentGenCoords(), 1u);
  EXPECT_EQ(clonedNode->getDependentDof(0), clone->getDof(1));
  EXPECT_NE(clonedNode->getDependentDof(0), node->getDependentDof(0));
  for (std::size_t i = 0; i < original->getNumDofs(); ++i) {
    EXPECT_EQ(clone->getDof(i)->getIndexInSkeleton(), i);
    EXPECT_NE(clone->getDof(i), original->getDof(i));
    EXPECT_EQ(clone->getDof(i)->getSkeleton(), clone);
  }
  EXPECT_TRUE(clone->getPositions().isApprox(originalPositions));
  if (useAnalytical) {
    ASSERT_NE(clonedIK->getAnalytical(), nullptr);
    EXPECT_NE(clonedIK->getAnalytical(), ik->getAnalytical());
    EXPECT_EQ(
        clonedIK->getAnalytical()->getDofs(), ik->getAnalytical()->getDofs());
    EXPECT_TRUE(clonedIK->getAnalytical()->getPositions().isApprox(
        Eigen::VectorXd::Constant(1, 0.3)));
  } else {
    EXPECT_EQ(clonedIK->getAnalytical(), nullptr);
  }

  ASSERT_TRUE(clonedIK->solveAndApply(false));
  EXPECT_NEAR(clone->getPosition(1), 0.8, 1e-7);
  EXPECT_DOUBLE_EQ(clone->getPosition(0), 0.2);
  EXPECT_TRUE(clonedNode->getWorldTransform().isApprox(target, 1e-7));
  EXPECT_TRUE(original->getPositions().isApprox(originalPositions));
  EXPECT_TRUE(node->getWorldTransform().isApprox(originalTransform));
  EXPECT_EQ(originalTransformUpdates, 0u);
}

} // namespace

//==============================================================================
TEST(InverseKinematics, CloneBodyNodeWithAnalytical)
{
  checkPrismaticIKClone(false, true);
}

//==============================================================================
TEST(InverseKinematics, CloneEndEffectorWithAnalytical)
{
  checkPrismaticIKClone(true, true);
}

//==============================================================================
TEST(InverseKinematics, CloneBodyNodeWithJacobianDLS)
{
  checkPrismaticIKClone(false, false);
}

//==============================================================================
TEST(InverseKinematics, CloneEndEffectorWithJacobianDLS)
{
  checkPrismaticIKClone(true, false);
}

//==============================================================================
TEST(InverseKinematics, CopyEndEffectorWithAnalytical)
{
  auto original = Skeleton::create();
  original->createJointAndBodyNodePair<PrismaticJoint>();
  auto pair = original->createJointAndBodyNodePair<PrismaticJoint>();
  auto* ee = pair.second->createEndEffector("copy_ik");
  ee->getIK(true)->setGradientMethod<PrismaticIKCloneAnalytical>();

  // The copied EndEffector's IK is cloned before its BodyNode joins a
  // Skeleton.
  auto destination = Skeleton::create("destination");
  const auto copied = pair.second->copyTo(destination, nullptr);
  ASSERT_NE(copied.second, nullptr);
  ASSERT_EQ(destination->getNumEndEffectors(), 1u);
  auto* copiedEE = destination->getEndEffector(0);
  ASSERT_NE(copiedEE->getIK(), nullptr);
  EXPECT_EQ(copiedEE->getIK()->getNode(), copiedEE);
  EXPECT_NE(copiedEE->getIK()->getAnalytical(), nullptr);
}
