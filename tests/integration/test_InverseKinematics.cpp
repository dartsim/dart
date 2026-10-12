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
#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/math/Constants.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/math/MathTypes.hpp"
#include "dart/optimizer/Problem.hpp"
#include "dart/optimizer/Solver.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <functional>
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
// Keeps naming the DOF with Skeleton index 1 wherever it is copied, as a
// solver with a fixed DOF map does.
class FixedDofIKCloneAnalytical : public PrismaticIKCloneAnalytical
{
public:
  using PrismaticIKCloneAnalytical::PrismaticIKCloneAnalytical;

  std::unique_ptr<GradientMethod> clone(InverseKinematics* newIK) const override
  {
    return std::make_unique<FixedDofIKCloneAnalytical>(
        newIK, getAnalyticalProperties());
  }

  const std::vector<std::size_t>& getDofs() const override
  {
    return mFixedDofs;
  }

private:
  const std::vector<std::size_t> mFixedDofs{1};
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
// Copies part of a chain of two prismatic joints whose end effector IK uses
// both DOFs. Each copy must leave out the root Joint, so the copied IK keeps
// only the copy of the end effector's own DOF.
void checkPrismaticIKCopy(
    bool useAnalytical,
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
  // the root DOF, so the copy must not apply it to the DOF that it keeps.
  const Eigen::Vector2d weights(0.0, 1.0);
  ik->getGradientMethod().setComponentWeights(weights);
  const Eigen::Vector2d seed(0.1, 0.6);
  ik->getProblem()->addSeed(seed);
  ik->getErrorMethod().setBounds(
      Eigen::Vector6d::Constant(-1e-8), Eigen::Vector6d::Constant(1e-8));
  ik->getSolver()->setNumMaxIterations(100);
  ASSERT_EQ(ik->getDofs(), std::vector<std::size_t>({0, 1}));
  ASSERT_EQ(ik->getDofMap(), std::vector<int>({0, 1}));

  const SkeletonPtr destination = copy(root.second, pair.second);
  ASSERT_NE(destination, original);
  EndEffector* copiedEE = destination->getEndEffector("copy_ik");
  ASSERT_NE(copiedEE, nullptr);
  const auto& copiedIK = copiedEE->getIK();
  ASSERT_NE(copiedIK, nullptr);
  EXPECT_EQ(copiedIK->getNode(), copiedEE);

  const std::size_t copiedDof
      = copiedEE->getBodyNode()->getParentJoint()->getIndexInSkeleton(0);
  ASSERT_EQ(copiedIK->getDofs(), std::vector<std::size_t>({copiedDof}));
  const std::vector<int>& dofMap = copiedIK->getDofMap();
  ASSERT_EQ(dofMap.size(), copiedEE->getNumDependentGenCoords());
  for (std::size_t i = 0; i < dofMap.size(); ++i) {
    EXPECT_EQ(
        dofMap[i],
        copiedEE->getDependentGenCoordIndex(i) == copiedDof ? 0 : -1);
  }
  const Eigen::VectorXd& copiedWeights
      = copiedIK->getGradientMethod().getComponentWeights();
  ASSERT_EQ(copiedWeights.size(), 1);
  EXPECT_EQ(copiedWeights[0], weights[1]);
  const auto& copiedSeeds = copiedIK->getProblem()->getSeeds();
  ASSERT_EQ(copiedSeeds.size(), 1u);
  ASSERT_EQ(copiedSeeds[0].size(), 1);
  EXPECT_EQ(copiedSeeds[0][0], seed[1]);
  if (useAnalytical) {
    ASSERT_NE(copiedIK->getAnalytical(), nullptr);
    EXPECT_EQ(copiedIK->getAnalytical()->getDofs(), copiedIK->getDofs());
  }

  // The analytical solver only moves the DOF if its DOF map is valid.
  Eigen::Isometry3d target = copiedEE->getWorldTransform();
  target.translation().x() = 0.8;
  copiedIK->setTarget(
      SimpleFrame::createShared(Frame::World(), "copy_target", target));
  Eigen::VectorXd expectedPositions = destination->getPositions();
  ASSERT_TRUE(copiedIK->solveAndApply(false));
  EXPECT_TRUE(copiedEE->getWorldTransform().isApprox(target, 1e-7));
  expectedPositions[copiedDof] = destination->getPosition(copiedDof);
  EXPECT_TRUE(destination->getPositions().isApprox(expectedPositions));

  EXPECT_TRUE(original->getPositions().isApprox(originalPositions));
  EXPECT_EQ(ik->getDofs(), std::vector<std::size_t>({0, 1}));
  EXPECT_EQ(ik->getDofMap(), std::vector<int>({0, 1}));
  EXPECT_TRUE(ik->getGradientMethod().getComponentWeights() == weights);
  ASSERT_EQ(ik->getProblem()->getSeeds().size(), 1u);
  EXPECT_TRUE(ik->getProblem()->getSeeds()[0] == seed);
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
    checkPrismaticIKCopy(useAnalytical, [](BodyNode*, BodyNode* body) {
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
    checkPrismaticIKCopy(useAnalytical, [](BodyNode*, BodyNode* body) {
      // The copied DOF follows the destination's DOFs, so its index shifts.
      auto destination = Skeleton::create("destination");
      auto lift = destination->createJointAndBodyNodePair<PrismaticJoint>();
      lift.first->setAxis(Eigen::Vector3d::UnitY());
      auto slide
          = lift.second->createChildJointAndBodyNodePair<PrismaticJoint>();
      slide.first->setAxis(Eigen::Vector3d::UnitZ());
      destination->setPositions(Eigen::Vector2d(0.4, -0.25));
      body->copyTo(destination, slide.second);
      return destination;
    });
  }
}

//==============================================================================
TEST(InverseKinematics, CopyEndEffectorWithMissingAnalyticalDof)
{
  auto original = Skeleton::create("original");
  auto root = original->createJointAndBodyNodePair<PrismaticJoint>();
  auto pair = root.second->createChildJointAndBodyNodePair<PrismaticJoint>();
  auto ik = pair.second->createEndEffector("copy_ik")->getIK(true);
  ik->setGradientMethod<FixedDofIKCloneAnalytical>();

  // The destination has no DOF with index 1, so rebuilding the copy's
  // analytical DOF map can only warn about it.
  auto destination = Skeleton::create("destination");
  pair.second->copyTo(destination, nullptr);
  ASSERT_EQ(destination->getNumDofs(), 1u);
  const auto& copiedIK = destination->getEndEffector("copy_ik")->getIK();
  ASSERT_NE(copiedIK, nullptr);
  EXPECT_EQ(copiedIK->getDofs(), std::vector<std::size_t>({0}));
  ASSERT_NE(copiedIK->getAnalytical(), nullptr);
  EXPECT_EQ(
      copiedIK->getAnalytical()->getDofs(), std::vector<std::size_t>({1}));
}

//==============================================================================
TEST(InverseKinematics, CopyEndEffectorWithNewRootJoint)
{
  for (const bool useAnalytical : {true, false}) {
    SCOPED_TRACE(useAnalytical ? "Analytical" : "JacobianDLS");
    checkPrismaticIKCopy(useAnalytical, [](BodyNode* root, BodyNode*) {
      // The FreeJoint replaces the root Joint instead of copying it.
      auto destination = Skeleton::create("destination");
      root->copyTo<FreeJoint>(destination, nullptr);
      return destination;
    });
  }
}
