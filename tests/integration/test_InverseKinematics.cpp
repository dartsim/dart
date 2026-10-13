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
#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/dynamics/EndEffector.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/HierarchicalIK.hpp"
#include "dart/dynamics/InverseKinematics.hpp"
#include "dart/dynamics/PrismaticJoint.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/math/Constants.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/math/MathTypes.hpp"
#include "dart/optimizer/Function.hpp"
#include "dart/optimizer/Problem.hpp"
#include "dart/optimizer/Solver.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <algorithm>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <cmath>

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
TEST(HierarchicalIK, CloneWithoutSolver)
{
  auto source = Skeleton::create();
  source->createJointAndBodyNodePair<FreeJoint>();
  auto target = Skeleton::create();
  target->createJointAndBodyNodePair<FreeJoint>();

  const auto check = [&target](const auto& cloned) {
    ASSERT_NE(cloned, nullptr);
    EXPECT_EQ(cloned->getSkeleton(), target);
    EXPECT_EQ(cloned->getSolver(), nullptr);
    EXPECT_EQ(cloned->getObjective(), nullptr);
    EXPECT_EQ(cloned->getNullSpaceObjective(), nullptr);
  };

  auto wholeBody = source->getIK(true);
  wholeBody->setSolver(nullptr);
  check(wholeBody->clone(target));
  check(wholeBody->cloneWholeBodyIK(target));

  auto composite = CompositeIK::create(source);
  composite->setSolver(nullptr);
  check(composite->clone(target));
  check(composite->cloneCompositeIK(target));
}

//==============================================================================
TEST(WholeBodyIK, ClonePreservesSharedObjectives)
{
  auto source = Skeleton::create();
  source->createJointAndBodyNodePair<FreeJoint>();
  auto ik = source->getIK(true);
  auto objective = std::make_shared<optimizer::ModularFunction>();
  objective->setCostFunction(
      [](const Eigen::VectorXd& x) { return x.squaredNorm(); });
  objective->setGradientFunction(
      [](const Eigen::VectorXd& x, Eigen::Map<Eigen::VectorXd> grad) {
        grad = 2 * x;
      });
  auto nullSpaceObjective = std::make_shared<optimizer::ModularFunction>();
  nullSpaceObjective->setCostFunction(
      [](const Eigen::VectorXd& x) { return 2 * x.squaredNorm(); });
  nullSpaceObjective->setGradientFunction(
      [](const Eigen::VectorXd& x, Eigen::Map<Eigen::VectorXd> grad) {
        grad = 4 * x;
      });
  ik->setObjective(objective);
  ik->setNullSpaceObjective(nullSpaceObjective);

  auto target = Skeleton::create();
  target->createJointAndBodyNodePair<FreeJoint>();
  const Eigen::VectorXd x = Eigen::VectorXd::Constant(6, 0.25);
  const auto check = [&](const auto& cloned) {
    EXPECT_EQ(cloned->getObjective(), objective);
    EXPECT_EQ(cloned->getNullSpaceObjective(), nullSpaceObjective);
    EXPECT_TRUE(cloned->hasNullSpaceObjective());
    auto aggregate = cloned->getProblem()->getObjective();
    ASSERT_NE(aggregate, nullptr);
    EXPECT_DOUBLE_EQ(aggregate->eval(x), 3 * x.squaredNorm());
    Eigen::VectorXd grad = Eigen::VectorXd::Zero(6);
    aggregate->evalGradient(x, grad);
    EXPECT_TRUE(equals(grad, (6 * x).eval()));
  };
  check(ik->clone(target));
  check(ik->cloneWholeBodyIK(target));
}

//==============================================================================
TEST(WholeBodyIK, CloneRebindsObjectives)
{
  auto source = Skeleton::create();
  source->createJointAndBodyNodePair<FreeJoint>();
  const Eigen::VectorXd original = Eigen::VectorXd::Constant(6, -0.25);
  source->setPositions(original);
  auto ik = source->getIK(true);
  auto objective = std::make_shared<constraint::BalanceConstraint>(ik);
  auto nullSpaceObjective = std::make_shared<constraint::BalanceConstraint>(ik);
  ik->setObjective(objective);
  ik->setNullSpaceObjective(nullSpaceObjective);

  auto target = Skeleton::create();
  target->createJointAndBodyNodePair<FreeJoint>();
  const Eigen::VectorXd x = Eigen::VectorXd::Constant(6, 0.25);
  const auto check = [&](const auto& cloned) {
    EXPECT_NE(cloned->getObjective(), objective);
    EXPECT_NE(cloned->getNullSpaceObjective(), nullSpaceObjective);
    for (const auto& function :
         {cloned->getObjective(), cloned->getNullSpaceObjective()}) {
      ASSERT_NE(
          std::dynamic_pointer_cast<constraint::BalanceConstraint>(function),
          nullptr);
      target->resetPositions();
      function->eval(x);
      EXPECT_TRUE(equals(target->getPositions(), x));
      EXPECT_TRUE(equals(source->getPositions(), original));
    }
  };
  check(ik->clone(target));
  check(ik->cloneWholeBodyIK(target));
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

  // Solve for the parent Joint of the node's BodyNode, so that copies of this
  // solver name the copied DOF.
  const std::vector<std::size_t>& getDofs() const override
  {
    const Joint* joint = getIK()->getNode()->getBodyNode()->getParentJoint();
    mDofs.assign(1, joint->getIndexInSkeleton(0));
    return mDofs;
  }

  // Assumes a parent BodyNode frame with no x offset or rotation.
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
  mutable std::vector<std::size_t> mDofs;
};

//==============================================================================
// Solves for the DOFs that its IK module selects, so its copies follow the
// copied selection.
class SelectedDofsIKCloneAnalytical : public PrismaticIKCloneAnalytical
{
public:
  using PrismaticIKCloneAnalytical::PrismaticIKCloneAnalytical;

  std::unique_ptr<GradientMethod> clone(InverseKinematics* newIK) const override
  {
    return std::make_unique<SelectedDofsIKCloneAnalytical>(
        newIK, getAnalyticalProperties());
  }

  const std::vector<std::size_t>& getDofs() const override
  {
    return getIK()->getDofs();
  }

  // Moves the parent Joint of the node's BodyNode as the base class does, and
  // keeps the other selected DOFs where they are.
  const std::vector<Solution>& computeSolutions(
      const Eigen::Isometry3d& desiredTf) override
  {
    const JacobianNode* node = getIK()->getNode();
    const std::size_t x
        = node->getBodyNode()->getParentJoint()->getIndexInSkeleton(0);
    const std::vector<std::size_t>& dofs = getDofs();
    Eigen::VectorXd q = node->getSkeleton()->getPositions(dofs);
    for (std::size_t i = 0; i < dofs.size(); ++i) {
      if (dofs[i] == x)
        q[i] = desiredTf.translation().x();
    }
    mSolutions.clear();
    mSolutions.emplace_back(q);
    checkSolutionJointLimits();
    return mSolutions;
  }
};

//==============================================================================
// Solves for the DOFs that its node depends on, so its copies follow the copy.
// Only its DOFs matter here, so it finds no solutions.
class DependentDofsIKCloneAnalytical : public PrismaticIKCloneAnalytical
{
public:
  using PrismaticIKCloneAnalytical::PrismaticIKCloneAnalytical;

  std::unique_ptr<GradientMethod> clone(InverseKinematics* newIK) const override
  {
    return std::make_unique<DependentDofsIKCloneAnalytical>(
        newIK, getAnalyticalProperties());
  }

  const std::vector<std::size_t>& getDofs() const override
  {
    return getIK()->getNode()->getDependentGenCoordIndices();
  }

  const std::vector<Solution>& computeSolutions(
      const Eigen::Isometry3d&) override
  {
    mSolutions.clear();
    return mSolutions;
  }
};

//==============================================================================
// Stores the DOFs of a torso that turns about z and of an arm that slides along
// x from one unit off the torso's axis, as IkFast stores the DOFs of an arm,
// and solves for the position of a tool on the arm.
class TorsoArmIKCloneAnalytical : public PrismaticIKCloneAnalytical
{
public:
  using PrismaticIKCloneAnalytical::PrismaticIKCloneAnalytical;

  std::unique_ptr<GradientMethod> clone(InverseKinematics* newIK) const override
  {
    return std::make_unique<TorsoArmIKCloneAnalytical>(
        newIK, getAnalyticalProperties());
  }

  const std::vector<std::size_t>& getDofs() const override
  {
    return mTorsoArmDofs;
  }

  const std::vector<Solution>& computeSolutions(
      const Eigen::Isometry3d& desiredTf) override
  {
    const Eigen::Vector3d& position = desiredTf.translation();
    mSolutions.clear();
    mSolutions.emplace_back(Eigen::Vector2d(
        std::atan2(position.y(), position.x()),
        std::hypot(position.x(), position.y()) - 1.0));
    checkSolutionJointLimits();
    return mSolutions;
  }

private:
  const std::vector<std::size_t> mTorsoArmDofs{0, 1};
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

//==============================================================================
std::vector<double> entries(const Eigen::VectorXd& values)
{
  return std::vector<double>(values.data(), values.data() + values.size());
}

//==============================================================================
// Copies a chain of two prismatic joints whose end effector and ShapeNode IK
// modules select both DOFs. keepsRootDof says whether the copy keeps the root
// DOF, as a copy of the root Joint or as a DOF that the copy shares with the
// original in one Skeleton. The copied IK modules must select the kept DOFs,
// and solving the copied end effector IK must move only the copy.
void checkPrismaticIKCopy(
    bool useAnalytical,
    bool keepsRootDof,
    const std::function<SkeletonPtr(BodyNode* root, BodyNode* body)>& copy)
{
  auto original = Skeleton::create("original");
  auto root = original->createJointAndBodyNodePair<PrismaticJoint>();
  root.first->setAxis(Eigen::Vector3d::UnitY());
  auto pair = root.second->createChildJointAndBodyNodePair<PrismaticJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitX());
  original->setPositions(Eigen::Vector2d(0.2, 0.3));
  const Eigen::VectorXd originalPositions = original->getPositions();

  auto ik = pair.second->createEndEffector("copy_ik")->getIK(true);
  if (useAnalytical)
    ik->setGradientMethod<PrismaticIKCloneAnalytical>();
  // Weights and seeds apply by position in the selection. A zero weight holds
  // the root DOF, so a copy that drops it must not apply it to the DOF that it
  // keeps.
  const Eigen::Vector2d weights(0.0, 1.0);
  ik->getGradientMethod().setComponentWeights(weights);
  const Eigen::Vector2d seed(0.1, 0.6);
  ik->getProblem()->addSeed(seed);
  ik->getErrorMethod().setBounds(
      Eigen::Vector6d::Constant(-1e-8), Eigen::Vector6d::Constant(1e-8));
  auto& region
      = static_cast<InverseKinematics::TaskSpaceRegion&>(ik->getErrorMethod());
  const auto referenceFrame
      = SimpleFrame::createShared(Frame::World(), "reference");
  region.setComputeFromCenter(false);
  region.setReferenceFrame(referenceFrame);
  ik->getGradientMethod().setComponentWiseClamp(0.15);
  ik->setHierarchyLevel(2);
  ik->getSolver()->setNumMaxIterations(100);
  ASSERT_EQ(ik->getDofs(), std::vector<std::size_t>({0, 1}));
  ASSERT_EQ(ik->getDofMap(), std::vector<int>({0, 1}));
  const auto shapeIK
      = pair.second->createShapeNode(std::make_shared<SphereShape>(0.1))
            ->getIK(true);
  ASSERT_EQ(shapeIK->getDofs(), std::vector<std::size_t>({0, 1}));

  testing::internal::CaptureStderr();
  const SkeletonPtr destination = copy(root.second, pair.second);
  const std::string output = testing::internal::GetCapturedStderr();
  if (!useAnalytical) {
    EXPECT_TRUE(output.empty()) << output;
  }

  // The copied EndEffector follows those that the destination already had
  ASSERT_GT(destination->getNumEndEffectors(), 0u);
  EndEffector* copiedEE
      = destination->getEndEffector(destination->getNumEndEffectors() - 1);
  ASSERT_NE(copiedEE, ik->getNode());
  const auto& copiedIK = copiedEE->getIK();
  ASSERT_NE(copiedIK, nullptr);
  EXPECT_EQ(copiedIK->getNode(), copiedEE);

  BodyNode* copiedBody = copiedEE->getBodyNode();
  const std::size_t copiedDof
      = copiedBody->getParentJoint()->getIndexInSkeleton(0);
  std::vector<std::size_t> expectedDofs{copiedDof};
  Eigen::VectorXd expectedWeights = weights.tail<1>();
  Eigen::VectorXd expectedSeed = seed.tail<1>();
  if (keepsRootDof) {
    const Joint* rootJoint = copiedBody->getParentBodyNode()->getParentJoint();
    expectedDofs.insert(expectedDofs.begin(), rootJoint->getIndexInSkeleton(0));
    expectedWeights = weights;
    expectedSeed = seed;
  }

  ASSERT_EQ(copiedIK->getDofs(), expectedDofs);
  const std::vector<int>& dofMap = copiedIK->getDofMap();
  ASSERT_EQ(dofMap.size(), copiedEE->getNumDependentGenCoords());
  for (std::size_t i = 0; i < dofMap.size(); ++i) {
    const auto selected = std::find(
        expectedDofs.begin(),
        expectedDofs.end(),
        copiedEE->getDependentGenCoordIndex(i));
    EXPECT_EQ(
        dofMap[i],
        selected == expectedDofs.end() ? -1 : selected - expectedDofs.begin());
  }
  EXPECT_EQ(
      entries(copiedIK->getGradientMethod().getComponentWeights()),
      entries(expectedWeights));
  const auto& copiedSeeds = copiedIK->getProblem()->getSeeds();
  ASSERT_EQ(copiedSeeds.size(), 1u);
  EXPECT_EQ(entries(copiedSeeds[0]), entries(expectedSeed));
  EXPECT_EQ(copiedBody->getShapeNode(0)->getIK()->getDofs(), expectedDofs);
  EXPECT_EQ(copiedIK->getAnalytical(), nullptr);
  EXPECT_EQ(copiedIK->getGradientMethod().getMethodName(), "JacobianDLS");
  if (useAnalytical) {
    const std::string warning
        = "The copied IK module of the JacobianNode [" + copiedEE->getName()
          + "] uses JacobianDLS. Set the analytical solver up again for the "
            "copy.\n";
    const auto position = output.find(warning);
    ASSERT_NE(position, std::string::npos) << output;
    EXPECT_EQ(
        output.find(warning, position + warning.size()), std::string::npos)
        << output;
  }
  const InverseKinematics::ErrorMethod::Properties defaults;
  const auto& error = copiedIK->getErrorMethod();
  EXPECT_DOUBLE_EQ(error.getErrorLengthClamp(), defaults.mErrorLengthClamp);
  EXPECT_TRUE(error.getErrorWeights().isApprox(defaults.mErrorWeights));
  EXPECT_TRUE(
      error.getBounds().first.isApprox(ik->getErrorMethod().getBounds().first));
  EXPECT_TRUE(error.getBounds().second.isApprox(
      ik->getErrorMethod().getBounds().second));
  const auto* copiedRegion
      = dynamic_cast<const InverseKinematics::TaskSpaceRegion*>(&error);
  ASSERT_NE(copiedRegion, nullptr);
  EXPECT_FALSE(copiedRegion->isComputingFromCenter());
  EXPECT_EQ(copiedRegion->getReferenceFrame(), referenceFrame);
  EXPECT_DOUBLE_EQ(copiedIK->getGradientMethod().getComponentWiseClamp(), 0.15);
  EXPECT_EQ(copiedIK->getHierarchyLevel(), 2u);

  // Solving moves only the copied DOF.
  Eigen::Isometry3d target = copiedEE->getWorldTransform();
  target.translation().x() = 0.8;
  copiedIK->setTarget(
      SimpleFrame::createShared(Frame::World(), "copy_target", target));
  Eigen::VectorXd expectedPositions = destination->getPositions();
  ASSERT_TRUE(copiedIK->solveAndApply(false));
  EXPECT_TRUE(copiedEE->getWorldTransform().isApprox(target, 1e-7));
  expectedPositions[copiedDof] = destination->getPosition(copiedDof);
  EXPECT_TRUE(destination->getPositions().isApprox(expectedPositions));

  // A copy into the original Skeleton adds its DOFs after the original ones
  EXPECT_TRUE(original->getPositions().head(2).isApprox(originalPositions));
  EXPECT_EQ(ik->getDofs(), std::vector<std::size_t>({0, 1}));
  EXPECT_EQ(ik->getDofMap(), std::vector<int>({0, 1}));
  EXPECT_EQ(
      entries(ik->getGradientMethod().getComponentWeights()), entries(weights));
  ASSERT_EQ(ik->getProblem()->getSeeds().size(), 1u);
  EXPECT_EQ(entries(ik->getProblem()->getSeeds()[0]), entries(seed));
  EXPECT_EQ(shapeIK->getDofs(), std::vector<std::size_t>({0, 1}));
}

//==============================================================================
// Creates a Skeleton with two DOFs that do not move its BodyNodes along x
SkeletonPtr createSkeletonWithDofs()
{
  auto skeleton = Skeleton::create("destination");
  auto lift = skeleton->createJointAndBodyNodePair<PrismaticJoint>();
  lift.first->setAxis(Eigen::Vector3d::UnitY());
  auto slide = lift.second->createChildJointAndBodyNodePair<PrismaticJoint>();
  slide.first->setAxis(Eigen::Vector3d::UnitZ());
  skeleton->setPositions(Eigen::Vector2d(0.4, -0.25));
  return skeleton;
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
TEST(InverseKinematics, CopyEndEffectorIntoEmptySkeleton)
{
  for (const bool useAnalytical : {true, false}) {
    SCOPED_TRACE(useAnalytical ? "Analytical" : "JacobianDLS");
    checkPrismaticIKCopy(useAnalytical, false, [](BodyNode*, BodyNode* body) {
      auto destination = Skeleton::create("destination");
      body->copyTo(destination, nullptr);
      return destination;
    });
  }
}

//==============================================================================
TEST(InverseKinematics, CopyEndEffectorIntoSkeletonWithDofs)
{
  for (const bool useAnalytical : {true, false}) {
    SCOPED_TRACE(useAnalytical ? "Analytical" : "JacobianDLS");
    checkPrismaticIKCopy(useAnalytical, false, [](BodyNode*, BodyNode* body) {
      // The copied DOF follows the destination's DOFs, so its index shifts.
      auto destination = createSkeletonWithDofs();
      body->copyTo(destination, destination->getBodyNode(1));
      return destination;
    });
  }
}

//==============================================================================
TEST(InverseKinematics, CopyEndEffectorWithNewRootJoint)
{
  for (const bool useAnalytical : {true, false}) {
    SCOPED_TRACE(useAnalytical ? "Analytical" : "JacobianDLS");
    checkPrismaticIKCopy(useAnalytical, false, [](BodyNode* root, BodyNode*) {
      // The FreeJoint replaces the root Joint instead of copying it.
      auto destination = Skeleton::create("destination");
      root->copyTo<FreeJoint>(destination, nullptr);
      return destination;
    });
  }
}

//==============================================================================
TEST(InverseKinematics, CopyChainAsSkeleton)
{
  for (const bool useAnalytical : {true, false}) {
    SCOPED_TRACE(useAnalytical ? "Analytical" : "JacobianDLS");
    checkPrismaticIKCopy(useAnalytical, true, [](BodyNode* root, BodyNode*) {
      // Every copied DOF keeps its index.
      return root->copyAs("destination");
    });
  }
}

//==============================================================================
TEST(InverseKinematics, CopyChainIntoSkeletonWithDofs)
{
  for (const bool useAnalytical : {true, false}) {
    SCOPED_TRACE(useAnalytical ? "Analytical" : "JacobianDLS");
    checkPrismaticIKCopy(useAnalytical, true, [](BodyNode* root, BodyNode*) {
      // Every copied DOF follows the destination's DOFs.
      auto destination = createSkeletonWithDofs();
      root->copyTo(destination, destination->getBodyNode(1));
      return destination;
    });
  }
}

//==============================================================================
TEST(InverseKinematics, CopyEndEffectorWithinSkeleton)
{
  for (const bool useAnalytical : {true, false}) {
    SCOPED_TRACE(useAnalytical ? "Analytical" : "JacobianDLS");
    checkPrismaticIKCopy(
        useAnalytical, true, [](BodyNode* root, BodyNode* body) {
          // The copy keeps the root DOF that it shares with the original.
          body->copyTo(root);
          return root->getSkeleton();
        });
  }
}

//==============================================================================
TEST(InverseKinematics, CopyEndEffectorWithinSkeletonKeepsUnrelatedDof)
{
  for (const bool useAnalytical : {true, false}) {
    SCOPED_TRACE(useAnalytical ? "Analytical" : "JacobianDLS");
    auto skeleton = Skeleton::create("original");
    auto root = skeleton->createJointAndBodyNodePair<PrismaticJoint>();
    auto pair = root.second->createChildJointAndBodyNodePair<PrismaticJoint>();
    root.second->createChildJointAndBodyNodePair<PrismaticJoint>();

    // An IK module can select a DOF that its node does not depend on, such as
    // the DOF of another branch that a null-space objective drives, and an
    // analytical solver can solve for it too.
    auto ik = pair.second->createEndEffector("copy_ik")->getIK(true);
    ik->setDofs({0, 1, 2});
    if (useAnalytical)
      ik->setGradientMethod<SelectedDofsIKCloneAnalytical>();
    const Eigen::Vector3d weights(0.5, 1.0, 0.25);
    ik->getGradientMethod().setComponentWeights(weights);
    const Eigen::Vector3d seed(0.1, 0.6, -0.3);
    ik->getProblem()->addSeed(seed);

    // Analytical copies warn once and use the iterative solver.
    testing::internal::CaptureStderr();
    pair.second->copyTo(root.second);
    const std::string output = testing::internal::GetCapturedStderr();
    if (useAnalytical) {
      EXPECT_NE(output.find("uses JacobianDLS"), std::string::npos) << output;
    }
    if (!useAnalytical) {
      EXPECT_TRUE(output.empty()) << output;
    }

    // The copy keeps the DOFs that it shares with the original, with their
    // weights and seed entries, and selects its own DOF in place of the
    // original's.
    ASSERT_EQ(skeleton->getNumDofs(), 4u);
    EndEffector* copiedEE = skeleton->getEndEffector(1);
    EXPECT_FALSE(copiedEE->dependsOn(2));
    const auto& copiedIK = copiedEE->getIK();
    EXPECT_EQ(copiedIK->getDofs(), std::vector<std::size_t>({0, 3, 2}));
    EXPECT_EQ(
        entries(copiedIK->getGradientMethod().getComponentWeights()),
        entries(weights));
    const auto& copiedSeeds = copiedIK->getProblem()->getSeeds();
    ASSERT_EQ(copiedSeeds.size(), 1u);
    EXPECT_EQ(entries(copiedSeeds[0]), entries(seed));
    EXPECT_EQ(ik->getDofs(), std::vector<std::size_t>({0, 1, 2}));
    EXPECT_EQ(copiedIK->getAnalytical(), nullptr);
    EXPECT_EQ(copiedIK->getGradientMethod().getMethodName(), "JacobianDLS");
  }
}

//==============================================================================
TEST(InverseKinematics, CopyEndEffectorIntoEarlierTree)
{
  auto skeleton = Skeleton::create("original");
  BodyNode* first
      = skeleton->createJointAndBodyNodePair<PrismaticJoint>().second;
  auto second = skeleton->createJointAndBodyNodePair<PrismaticJoint>();
  auto pair = second.second->createChildJointAndBodyNodePair<PrismaticJoint>();
  auto ik = pair.second->createEndEffector("copy_ik")->getIK(true);
  ASSERT_EQ(ik->getDofs(), std::vector<std::size_t>({1, 2}));
  const std::vector<DegreeOfFreedom*> dofs = skeleton->getDofs();

  testing::internal::CaptureStderr();
  pair.second->copyTo(first);
  const std::string output = testing::internal::GetCapturedStderr();
  EXPECT_TRUE(output.empty()) << output;

  // The copy joins the first tree, but the Skeleton appends its DOF, so the
  // DOFs of the second tree keep their indices.
  EndEffector* copiedEE = skeleton->getEndEffector(1);
  ASSERT_EQ(copiedEE->getBodyNode()->getTreeIndex(), 0u);
  ASSERT_EQ(skeleton->getNumDofs(), 4u);
  for (std::size_t i = 0; i < dofs.size(); ++i)
    EXPECT_EQ(skeleton->getDof(i), dofs[i]) << i;

  // The copied IK module selects the DOF of the second tree's root, which it
  // shares with the original, and its own DOF in place of the original's.
  const auto selected = [&](const InverseKinematics& module) {
    std::vector<DegreeOfFreedom*> selection;
    for (const std::size_t dof : module.getDofs())
      selection.push_back(skeleton->getDof(dof));
    return selection;
  };
  EXPECT_EQ(
      selected(*copiedEE->getIK()),
      std::vector<DegreeOfFreedom*>(
          {dofs[1], copiedEE->getBodyNode()->getParentJoint()->getDof(0)}));
  EXPECT_EQ(selected(*ik), std::vector<DegreeOfFreedom*>({dofs[1], dofs[2]}));
}

//==============================================================================
TEST(InverseKinematics, CopyToolWithinSkeletonReplacesStaleAnalytical)
{
  // A torso that turns about z carries an arm that slides along x from one
  // unit off its axis, with a tool welded to the arm, and a fixed arm along y.
  auto skeleton = Skeleton::create("original");
  auto torso = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  torso.first->setAxis(Eigen::Vector3d::UnitZ());
  PrismaticJoint::Properties slide;
  slide.mAxis = Eigen::Vector3d::UnitX();
  slide.mT_ParentBodyToJoint.translation() = Eigen::Vector3d::UnitX();
  BodyNode* arm
      = torso.second->createChildJointAndBodyNodePair<PrismaticJoint>(slide)
            .second;
  BodyNode* tool = arm->createChildJointAndBodyNodePair<WeldJoint>().second;
  WeldJoint::Properties fixed;
  fixed.mT_ParentBodyToJoint.translation() = 2.0 * Eigen::Vector3d::UnitY();
  BodyNode* fixedArm
      = torso.second->createChildJointAndBodyNodePair<WeldJoint>(fixed).second;

  // Both end effectors select the torso and arm DOFs. One solver stores them,
  // as IkFast does, and the other solves for the DOFs that its node depends on.
  auto storedIK = tool->createEndEffector("stored")->getIK(true);
  storedIK->setDofs({0, 1});
  storedIK->setGradientMethod<TorsoArmIKCloneAnalytical>();
  auto dependentIK = tool->createEndEffector("dependent")->getIK(true);
  dependentIK->setDofs({0, 1});
  dependentIK->setGradientMethod<DependentDofsIKCloneAnalytical>();

  testing::internal::CaptureStderr();
  BodyNode* copy = tool->copyTo(fixedArm).second;
  const std::string warning = testing::internal::GetCapturedStderr();

  // The copies depend on the torso DOF alone, and each copied IK module keeps
  // both selected DOFs, which it shares with the original.
  EndEffector* storedCopy = copy->getEndEffector(0);
  EndEffector* dependentCopy = copy->getEndEffector(1);
  ASSERT_EQ(
      storedCopy->getDependentGenCoordIndices(), std::vector<std::size_t>({0}));
  EXPECT_EQ(storedCopy->getIK()->getDofs(), std::vector<std::size_t>({0, 1}));
  EXPECT_EQ(
      dependentCopy->getIK()->getDofs(), std::vector<std::size_t>({0, 1}));

  // Both copied modules use JacobianDLS and each warns once.
  for (EndEffector* ee : {storedCopy, dependentCopy}) {
    const std::string message
        = "The copied IK module of the JacobianNode [" + ee->getName()
          + "] uses JacobianDLS. Set the analytical solver up again for the "
            "copy.\n";
    const auto position = warning.find(message);
    ASSERT_NE(position, std::string::npos) << warning;
    EXPECT_EQ(
        warning.find(message, position + message.size()), std::string::npos)
        << warning;
    EXPECT_EQ(ee->getIK()->getAnalytical(), nullptr);
    EXPECT_EQ(ee->getIK()->getGradientMethod().getMethodName(), "JacobianDLS");
  }
  EXPECT_NE(storedIK->getAnalytical(), nullptr);
  EXPECT_NE(dependentIK->getAnalytical(), nullptr);

  // A quarter turn of the torso brings the copy to its target. The stale solver
  // would turn the torso half a turn instead, which aims the original's arm at
  // the target, and the copy would miss it.
  const auto& copiedIK = storedCopy->getIK();
  const Eigen::Isometry3d target
      = Eigen::Isometry3d(Eigen::AngleAxisd(
            math::constantsd::half_pi(), Eigen::Vector3d::UnitZ()))
        * storedCopy->getWorldTransform();
  copiedIK->setTarget(
      SimpleFrame::createShared(Frame::World(), "copy_target", target));
  copiedIK->getErrorMethod().setBounds(
      Eigen::Vector6d::Constant(-1e-8), Eigen::Vector6d::Constant(1e-8));
  copiedIK->getSolver()->setNumMaxIterations(100);
  EXPECT_TRUE(copiedIK->solveAndApply(false));
  EXPECT_TRUE(storedCopy->getWorldTransform().isApprox(target, 1e-7));
  EXPECT_NEAR(skeleton->getPosition(0), math::constantsd::half_pi(), 1e-7);
  EXPECT_EQ(skeleton->getPosition(1), 0.0);
}

//==============================================================================
TEST(InverseKinematics, CopyEndEffectorWithoutSelectedDofs)
{
  auto original = Skeleton::create("original");
  auto root = original->createJointAndBodyNodePair<PrismaticJoint>();
  auto weld = root.second->createChildJointAndBodyNodePair<WeldJoint>();
  auto ik = weld.second->createEndEffector("copy_ik")->getIK(true);
  ASSERT_EQ(ik->getDofs(), std::vector<std::size_t>({0}));

  // The copy leaves out the only DOF that the IK module selected.
  auto destination = Skeleton::create("destination");
  testing::internal::CaptureStderr();
  weld.second->copyTo(destination, nullptr);
  const std::string warning = testing::internal::GetCapturedStderr();
  EXPECT_NE(
      warning.find("has none of the DOFs that its IK module selected"),
      std::string::npos)
      << warning;
  const auto& copiedIK = destination->getEndEffector("copy_ik")->getIK();
  ASSERT_NE(copiedIK, nullptr);
  EXPECT_TRUE(copiedIK->getDofs().empty());
  EXPECT_EQ(ik->getDofs(), std::vector<std::size_t>({0}));
}
