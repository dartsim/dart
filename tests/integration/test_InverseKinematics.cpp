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
#include "dart/constraint/BalanceConstraint.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/HierarchicalIK.hpp"
#include "dart/dynamics/InverseKinematics.hpp"
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
TEST(InverseKinematics, DoNotApplySolutionWithoutSolver)
{
  auto skel = Skeleton::create();
  skel->createJointAndBodyNodePair<FreeJoint>();
  const Eigen::VectorXd original = Eigen::VectorXd::Constant(6, 0.25);
  skel->setPositions(original);

  const auto check = [&skel, &original](const auto& ik) {
    ik->setSolver(nullptr);
    testing::internal::CaptureStderr();
    EXPECT_FALSE(ik->solveAndApply());
    for (bool allowIncompleteResult : {false, true}) {
      EXPECT_FALSE(ik->solveAndApply(allowIncompleteResult));
      Eigen::VectorXd solution;
      EXPECT_FALSE(ik->solveAndApply(solution, allowIncompleteResult));
      EXPECT_EQ(solution.size(), 0);
      solution = Eigen::VectorXd::Zero(1);
      EXPECT_FALSE(ik->solveAndApply(solution, allowIncompleteResult));
      EXPECT_EQ(solution.size(), 1);
    }
    const auto warnings = testing::internal::GetCapturedStderr();
    EXPECT_NE(warnings.find("is a nullptr"), std::string::npos);
    EXPECT_EQ(warnings.find("setPositions"), std::string::npos);
    EXPECT_TRUE(equals(skel->getPositions(), original));
  };

  check(skel->getBodyNode(0)->getIK(true));
  check(skel->getIK(true));
}

//==============================================================================
TEST(BalanceConstraint, HierarchyClonePreservesSettings)
{
  auto source = Skeleton::create();
  source->createJointAndBodyNodePair<FreeJoint>();
  auto ik = source->getIK(true);
  using Balance = constraint::BalanceConstraint;
  auto balance = std::make_shared<Balance>(
      ik, Balance::SHIFT_COM, Balance::OPTIMIZE_BALANCE);
  balance->setName("configured balance");
  balance->setOptimizationTolerance(2e-5);
  balance->setPseudoInverseDamping(0.125);
  ik->getProblem()->setObjective(balance);

  auto target = Skeleton::create();
  target->createJointAndBodyNodePair<FreeJoint>();
  auto cloned = ik->clone(target);
  auto copy = std::dynamic_pointer_cast<Balance>(
      cloned->getProblem()->getObjective());
  ASSERT_NE(copy, nullptr);
  EXPECT_NE(copy, balance);
  EXPECT_EQ(copy->getName(), balance->getName());
  EXPECT_EQ(copy->getBalanceMethod(), balance->getBalanceMethod());
  EXPECT_EQ(copy->getErrorMethod(), balance->getErrorMethod());
  EXPECT_DOUBLE_EQ(
      copy->getOptimizationTolerance(), balance->getOptimizationTolerance());
  EXPECT_DOUBLE_EQ(
      copy->getPseudoInverseDamping(), balance->getPseudoInverseDamping());
}

//==============================================================================
TEST(InverseKinematics, ExpiredHierarchyConstraintGradientIsGuarded)
{
  optimizer::FunctionPtr constraint;
  {
    auto skel = Skeleton::create();
    skel->createJointAndBodyNodePair<FreeJoint>();
    constraint = skel->getIK(true)->getProblem()->getEqConstraint(0);
  }
  // The Skeleton, and with it its whole-body IK, no longer exist.
  ASSERT_NE(constraint, nullptr);
  const Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd grad = Eigen::VectorXd::Ones(6);
  Eigen::Map<Eigen::VectorXd> gradMap(grad.data(), grad.size());
#ifdef NDEBUG
  constraint->evalGradient(x, gradMap);
  EXPECT_TRUE(grad.isZero());
#elif GTEST_HAS_DEATH_TEST
  EXPECT_DEATH(constraint->evalGradient(x, gradMap), "");
#endif
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
