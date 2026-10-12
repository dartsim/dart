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
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/math/Constants.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/math/MathTypes.hpp"
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
// Stores the index of the DOF that it solves for, as IkFast does, so its copies
// keep naming the DOF with Skeleton index 1.
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
  ik->getSolver()->setNumMaxIterations(100);
  ASSERT_EQ(ik->getDofs(), std::vector<std::size_t>({0, 1}));
  ASSERT_EQ(ik->getDofMap(), std::vector<int>({0, 1}));
  const auto shapeIK
      = pair.second->createShapeNode(std::make_shared<SphereShape>(0.1))
            ->getIK(true);
  ASSERT_EQ(shapeIK->getDofs(), std::vector<std::size_t>({0, 1}));

  // Binding the copied IK modules to the copy reports nothing
  testing::internal::CaptureStderr();
  const SkeletonPtr destination = copy(root.second, pair.second);
  const std::string output = testing::internal::GetCapturedStderr();
  EXPECT_TRUE(output.empty()) << output;

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
  if (useAnalytical) {
    ASSERT_NE(copiedIK->getAnalytical(), nullptr);
    EXPECT_EQ(
        copiedIK->getAnalytical()->getDofs(),
        std::vector<std::size_t>({copiedDof}));
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
  auto skeleton = Skeleton::create("original");
  auto root = skeleton->createJointAndBodyNodePair<PrismaticJoint>();
  auto pair = root.second->createChildJointAndBodyNodePair<PrismaticJoint>();
  root.second->createChildJointAndBodyNodePair<PrismaticJoint>();

  // An IK module can select a DOF that its node does not depend on, such as
  // the DOF of another branch that a null-space objective drives.
  auto ik = pair.second->createEndEffector("copy_ik")->getIK(true);
  ik->setDofs({0, 1, 2});
  const Eigen::Vector3d weights(0.5, 1.0, 0.25);
  ik->getGradientMethod().setComponentWeights(weights);
  const Eigen::Vector3d seed(0.1, 0.6, -0.3);
  ik->getProblem()->addSeed(seed);

  testing::internal::CaptureStderr();
  pair.second->copyTo(root.second);
  const std::string output = testing::internal::GetCapturedStderr();
  EXPECT_TRUE(output.empty()) << output;

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
TEST(InverseKinematics, CopyEndEffectorWithStoredAnalyticalDofs)
{
  auto original = Skeleton::create("original");
  auto root = original->createJointAndBodyNodePair<PrismaticJoint>();
  root.first->setAxis(Eigen::Vector3d::UnitY());
  auto pair = root.second->createChildJointAndBodyNodePair<PrismaticJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitX());
  auto ik = pair.second->createEndEffector("copy_ik")->getIK(true);
  ik->setGradientMethod<FixedDofIKCloneAnalytical>();
  ik->getErrorMethod().setBounds(
      Eigen::Vector6d::Constant(-1e-8), Eigen::Vector6d::Constant(1e-8));
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation().x() = 0.8;

  // The stored index still names the copied DOF while copyAs() keeps it.
  testing::internal::CaptureStderr();
  const SkeletonPtr sameIndices = root.second->copyAs("same_indices");
  const std::string output = testing::internal::GetCapturedStderr();
  EXPECT_TRUE(output.empty()) << output;
  const auto& sameIK = sameIndices->getEndEffector("copy_ik")->getIK();
  ASSERT_NE(sameIK->getAnalytical(), nullptr);
  EXPECT_EQ(sameIK->getAnalytical()->getDofs(), std::vector<std::size_t>({1}));
  sameIK->setTarget(
      SimpleFrame::createShared(Frame::World(), "copy_target", target));
  EXPECT_TRUE(sameIK->solveAndApply(false));
  EXPECT_NEAR(sameIndices->getPosition(1), 0.8, 1e-7);

  // Otherwise the stored index names another DOF or none, so the copy solves
  // with JacobianDLS instead, and only the copied DOF moves.
  const auto checkReplacedSolver = [&](const SkeletonPtr& destination,
                                       const std::string& warning,
                                       std::size_t copiedDof) {
    EXPECT_NE(
        warning.find(
            "solves for the DOFs [1], but the copies of the DOFs that it "
            "solved for are ["
            + std::to_string(copiedDof)
            + "]. A solver that stores DOF indices, such as IkFast, keeps the "
              "indices of the original Skeleton, so the copied IK module uses "
              "JacobianDLS instead."),
        std::string::npos)
        << warning;
    const auto& copiedIK = destination->getEndEffector("copy_ik")->getIK();
    ASSERT_NE(copiedIK, nullptr);
    EXPECT_EQ(copiedIK->getAnalytical(), nullptr);
    EXPECT_EQ(copiedIK->getGradientMethod().getMethodName(), "JacobianDLS");
    copiedIK->setTarget(
        SimpleFrame::createShared(Frame::World(), "copy_target", target));
    Eigen::VectorXd expectedPositions = destination->getPositions();
    EXPECT_TRUE(copiedIK->solveAndApply(false));
    EXPECT_TRUE(
        copiedIK->getNode()->getWorldTransform().isApprox(target, 1e-7));
    expectedPositions[copiedDof] = 0.8;
    EXPECT_TRUE(destination->getPositions().isApprox(expectedPositions, 1e-7))
        << destination->getPositions().transpose();
  };

  // Under a base, the copied DOF has index 2, and index 1 names the copy of
  // the root DOF.
  auto shiftedIndices = Skeleton::create("shifted_indices");
  shiftedIndices->createJointAndBodyNodePair<PrismaticJoint>();
  testing::internal::CaptureStderr();
  root.second->copyTo(shiftedIndices, shiftedIndices->getBodyNode(0));
  checkReplacedSolver(
      shiftedIndices, testing::internal::GetCapturedStderr(), 2);
  EXPECT_EQ(
      shiftedIndices->getEndEffector("copy_ik")->getIK()->getDofs(),
      std::vector<std::size_t>({1, 2}));

  // A copy of the end effector's BodyNode alone has no DOF with index 1
  auto missingIndex = Skeleton::create("missing_index");
  testing::internal::CaptureStderr();
  pair.second->copyTo(missingIndex, nullptr);
  checkReplacedSolver(missingIndex, testing::internal::GetCapturedStderr(), 0);
  ASSERT_EQ(missingIndex->getNumDofs(), 1u);

  // The original keeps its analytical solver and positions
  ASSERT_NE(ik->getAnalytical(), nullptr);
  EXPECT_EQ(ik->getAnalytical()->getDofs(), std::vector<std::size_t>({1}));
  EXPECT_TRUE(original->getPositions().isZero());
}

//==============================================================================
TEST(InverseKinematics, CopyEndEffectorWithSelectedAnalyticalDofs)
{
  auto original = Skeleton::create("original");
  auto root = original->createJointAndBodyNodePair<PrismaticJoint>();
  root.first->setAxis(Eigen::Vector3d::UnitY());
  auto pair = root.second->createChildJointAndBodyNodePair<PrismaticJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitX());
  auto ik = pair.second->createEndEffector("copy_ik")->getIK(true);
  ik->setGradientMethod<SelectedDofsIKCloneAnalytical>();
  ik->getErrorMethod().setBounds(
      Eigen::Vector6d::Constant(-1e-8), Eigen::Vector6d::Constant(1e-8));

  // The copy shifts the selected DOFs, and the copied solver solves for the
  // copied selection, so the copy keeps it.
  auto destination = createSkeletonWithDofs();
  testing::internal::CaptureStderr();
  root.second->copyTo(destination, destination->getBodyNode(1));
  const std::string output = testing::internal::GetCapturedStderr();
  EXPECT_TRUE(output.empty()) << output;
  const auto& copiedIK = destination->getEndEffector("copy_ik")->getIK();
  ASSERT_NE(copiedIK->getAnalytical(), nullptr);
  EXPECT_EQ(
      copiedIK->getAnalytical()->getDofs(), std::vector<std::size_t>({2, 3}));

  Eigen::Isometry3d target = copiedIK->getNode()->getWorldTransform();
  target.translation().x() = 0.8;
  copiedIK->setTarget(
      SimpleFrame::createShared(Frame::World(), "copy_target", target));
  Eigen::VectorXd expectedPositions = destination->getPositions();
  expectedPositions[3] = 0.8;
  EXPECT_TRUE(copiedIK->solveAndApply(false));
  EXPECT_TRUE(destination->getPositions().isApprox(expectedPositions, 1e-7))
      << destination->getPositions().transpose();
  EXPECT_TRUE(original->getPositions().isZero());
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
