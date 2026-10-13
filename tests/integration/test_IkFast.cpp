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

#include <dart/config.hpp>

#include <dart/utils/urdf/DartLoader.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Frame.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/IkFast.hpp>
#include <dart/dynamics/InverseKinematics.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/SharedLibraryIkFast.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/WeldJoint.hpp>

#include <dart/math/Constants.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <cstddef>

using namespace dart;

//==============================================================================
TEST(IkFast, WrapCyclicSolution)
{
  const auto pi = math::constantsd::pi();

  double sol;

  // Invalid bounds (lb > ub)
  EXPECT_FALSE(dynamics::wrapCyclicSolution(0, 10, -10, sol));

  // Current value is in the limits, but solution is lesser than lower limit.
  // Expect valid solution that is the cloest to the current value.
  sol = -3 * pi;
  EXPECT_TRUE(dynamics::wrapCyclicSolution(-pi / 2, -pi, +pi, sol));
  EXPECT_DOUBLE_EQ(sol, -pi);
  sol = -3 * pi;
  EXPECT_TRUE(
      dynamics::wrapCyclicSolution(-pi / 2, -(3.0 / 4.0) * pi, +pi, sol));
  EXPECT_DOUBLE_EQ(sol, +pi);
  sol = -3 * pi;
  EXPECT_FALSE(
      dynamics::wrapCyclicSolution(-pi / 2, -0.9 * pi, +0.9 * pi, sol));

  // Current value is in the limits, but solution is greater than upper limit.
  // Expect valid solution that is the cloest to the current value.
  sol = -3 * pi;
  EXPECT_TRUE(dynamics::wrapCyclicSolution(+pi / 2, -pi, +pi, sol));
  EXPECT_DOUBLE_EQ(sol, +pi);
  sol = -3 * pi;
  EXPECT_TRUE(
      dynamics::wrapCyclicSolution(+pi / 2, -pi, +(3.0 / 4.0) * pi, sol));
  EXPECT_DOUBLE_EQ(sol, -pi);
  sol = -3 * pi;
  EXPECT_FALSE(
      dynamics::wrapCyclicSolution(+pi / 2, -0.9 * pi, +0.9 * pi, sol));

  // Both current value and solution are lesser than lower limit.
  // Expect least valid solution.
  sol = -9 * pi;
  EXPECT_TRUE(dynamics::wrapCyclicSolution(-5 * pi, -4 * pi, +4 * pi, sol));
  EXPECT_DOUBLE_EQ(sol, -3 * pi);
  sol = -9 * pi;
  EXPECT_FALSE(dynamics::wrapCyclicSolution(-5 * pi, -4 * pi, -3.1 * pi, sol));

  // Both current value and solution are greater than upper limit.
  // Expect greatest valid solution.
  sol = +9 * pi;
  EXPECT_TRUE(dynamics::wrapCyclicSolution(+5 * pi, -4 * pi, +4 * pi, sol));
  EXPECT_DOUBLE_EQ(sol, +3 * pi);
  sol = +9 * pi;
  EXPECT_FALSE(dynamics::wrapCyclicSolution(+5 * pi, +3.1 * pi, +4 * pi, sol));
}

//==============================================================================
TEST(IkFast, FailedToLoadSharedLibrary)
{
  auto skel = dynamics::Skeleton::create();
  ASSERT_NE(skel, nullptr);

  auto bodyNode
      = skel->createJointAndBodyNodePair<dynamics::FreeJoint>().second;

  auto ee = bodyNode->createEndEffector("ee");
  ASSERT_NE(ee, nullptr);
  auto ik = ee->createIK();
  ASSERT_NE(ik, nullptr);
  auto ikfast = ik->setGradientMethod<dynamics::SharedLibraryIkFast>(
      "doesn't exist", std::vector<std::size_t>(), std::vector<std::size_t>());
  EXPECT_EQ(ikfast.isConfigured(), false);
}

//==============================================================================
TEST(IkFast, LoadWamArmIk)
{
  utils::DartLoader urdfParser;
  urdfParser.addPackageDirectory(
      "herb_description", DART_DATA_PATH "/urdf/wam");
  auto wam = urdfParser.parseSkeleton(DART_DATA_PATH "/urdf/wam/wam.urdf");
  ASSERT_NE(wam, nullptr);

  auto wam7 = wam->getBodyNode("/wam7");
  ASSERT_NE(wam7, nullptr);
  auto ee = wam7->createEndEffector("ee");
  auto ik = ee->createIK();
  auto targetFrame
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  targetFrame->setRotation(Eigen::Matrix3d::Identity());

  ik->setTarget(targetFrame);
  ik->setHierarchyLevel(1);
  const std::string libName = DART_TEST_IKFAST_LIBRARY;
  std::vector<std::size_t> ikFastDofs{0, 1, 3, 4, 5, 6};
  std::vector<std::size_t> ikFastFreeDofs{2};
  ik->setGradientMethod<dynamics::SharedLibraryIkFast>(
      libName, ikFastDofs, ikFastFreeDofs);
  auto analytical = ik->getAnalytical();
  ASSERT_NE(analytical, nullptr);
  EXPECT_EQ(analytical->getDofs().size(), 6);

  auto ikfast = dynamic_cast<dynamics::SharedLibraryIkFast*>(analytical);
  ASSERT_NE(ikfast, nullptr);
  EXPECT_EQ(ikfast->getNumJoints2(), 7);
  EXPECT_EQ(ikfast->getNumFreeParameters2(), 1);
  EXPECT_EQ(ikfast->getIkType2(), dynamics::IkFast::IkType::TRANSFORM_6D);
  EXPECT_EQ(ikfast->getIkFastVersion2(), "71");

  targetFrame->setTranslation(Eigen::Vector3d(0, 0, 0.5));
  auto solutions = ikfast->getSolutions(targetFrame->getTransform());
  EXPECT_TRUE(!solutions.empty());

  const auto dofs = ikfast->getDofs();

  for (const auto& solution : solutions) {
    ASSERT_EQ(solution.mConfig.size(), 6);

    if (solution.mValidity != InverseKinematics::Analytical::VALID)
      continue;

    wam->setPositions(dofs, solution.mConfig);
    Eigen::Isometry3d newTf = ee->getTransform();
    EXPECT_TRUE(equals(targetFrame->getTransform(), newTf, 1e-2));
  }
}

//==============================================================================
TEST(IkFast, CopyWamArmIk)
{
  utils::DartLoader urdfParser;
  urdfParser.addPackageDirectory(
      "herb_description", DART_DATA_PATH "/urdf/wam");
  auto wam = urdfParser.parseSkeleton(DART_DATA_PATH "/urdf/wam/wam.urdf");
  ASSERT_NE(wam, nullptr);
  ASSERT_EQ(wam->getNumDofs(), 7u);

  auto ik = wam->getBodyNode("/wam7")->createEndEffector("ee")->createIK();
  const std::vector<std::size_t> ikFastDofs{0, 1, 3, 4, 5, 6};
  ik->setGradientMethod<dynamics::SharedLibraryIkFast>(
      DART_TEST_IKFAST_LIBRARY, ikFastDofs, std::vector<std::size_t>{2});

  // A whole-skeleton clone keeps the identical chain and its analytical solver.
  const auto clone = wam->cloneSkeleton("wam_clone");
  auto clonedEE = clone->getEndEffector("ee");
  ASSERT_NE(clonedEE, nullptr);
  auto clonedIkFast
      = dynamic_cast<dynamics::IkFast*>(clonedEE->getIK()->getAnalytical());
  ASSERT_NE(clonedIkFast, nullptr);
  EXPECT_TRUE(clonedIkFast->isConfigured());
  EXPECT_EQ(clonedIkFast->getDofs(), ikFastDofs);

  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = Eigen::Vector3d(0, 0, 0.5);
  const auto solutions = clonedIkFast->getSolutions(target);
  std::size_t numValid = 0;
  for (const auto& solution : solutions) {
    if (solution.mValidity != InverseKinematics::Analytical::VALID)
      continue;
    ++numValid;
    clone->setPositions(ikFastDofs, solution.mConfig);
    EXPECT_TRUE(equals(target, clonedEE->getWorldTransform(), 1e-2));
  }
  EXPECT_GT(numValid, 0u);

  const auto copyWithJacobianDLS = [](const auto& copy) {
    testing::internal::CaptureStderr();
    dynamics::EndEffector* ee = copy();
    const std::string warning = testing::internal::GetCapturedStderr();
    EXPECT_NE(ee, nullptr);
    if (ee != nullptr) {
      const std::string message
          = "The copied IK module of the JacobianNode [" + ee->getName()
            + "] uses JacobianDLS. Set the analytical solver up again for the "
              "copy.\n";
      const auto position = warning.find(message);
      EXPECT_NE(position, std::string::npos) << warning;
      if (position != std::string::npos) {
        EXPECT_EQ(
            warning.find(message, position + message.size()), std::string::npos)
            << warning;
      }
      const auto& copiedIK = ee->getIK();
      EXPECT_EQ(copiedIK->getAnalytical(), nullptr);
      EXPECT_EQ(copiedIK->getGradientMethod().getMethodName(), "JacobianDLS");
      const InverseKinematics::ErrorMethod::Properties defaults;
      EXPECT_DOUBLE_EQ(
          copiedIK->getErrorMethod().getErrorLengthClamp(),
          defaults.mErrorLengthClamp);
      EXPECT_TRUE(copiedIK->getErrorMethod().getErrorWeights().isApprox(
          defaults.mErrorWeights));
    }
    return ee;
  };

  // Solving the copy reaches a target near the copied arm and moves only the
  // copied arm, which is what its IK module selects.
  const auto solveCopy = [](dynamics::EndEffector* ee) {
    ASSERT_NE(ee, nullptr);
    const auto& copiedIK = ee->getIK();
    const auto skeleton = ee->getSkeleton();
    const std::vector<std::size_t> dofs = copiedIK->getDofs();
    ASSERT_EQ(dofs.size(), 7u);
    Eigen::VectorXd goal(7);
    goal << 0.3, 0.5, 0.2, 1.2, -0.4, 0.6, 0.2;
    skeleton->setPositions(dofs, goal);
    const Eigen::Isometry3d target = ee->getWorldTransform();
    skeleton->setPositions(dofs, goal - Eigen::VectorXd::Constant(7, 0.05));
    copiedIK->setTarget(dynamics::SimpleFrame::createShared(
        dynamics::Frame::World(), "copy_target", target));
    Eigen::VectorXd expected = skeleton->getPositions();
    EXPECT_TRUE(copiedIK->solveAndApply(false));
    EXPECT_TRUE(equals(target, ee->getWorldTransform(), 1e-5));
    const Eigen::VectorXd positions = skeleton->getPositions();
    for (const std::size_t dof : dofs)
      expected[dof] = positions[dof];
    EXPECT_TRUE(positions.isApprox(expected));
  };

  dynamics::SkeletonPtr copy;
  solveCopy(copyWithJacobianDLS([&] {
    copy = wam->getRootBodyNode()->copyAs("wam_copy");
    return copy->getEndEffector("ee");
  }));

  // A welded base changes the chain without changing any DOF indices.
  auto withFixedBase = dynamics::Skeleton::create("wam_with_fixed_base");
  dynamics::WeldJoint::Properties fixed;
  fixed.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(1.0, -0.5, 0.2);
  fixed.mT_ParentBodyToJoint.linear()
      = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  auto fixedBase
      = withFixedBase->createJointAndBodyNodePair<dynamics::WeldJoint>(
          nullptr, fixed);
  solveCopy(copyWithJacobianDLS([&] {
    wam->getRootBodyNode()->copyTo(withFixedBase, fixedBase.second);
    return withFixedBase->getEndEffector("ee");
  }));
  EXPECT_EQ(
      withFixedBase->getEndEffector("ee")->getIK()->getDofs(), ik->getDofs());

  const Eigen::VectorXd wamPositions = wam->getPositions();

  // Under a FreeJoint, the copied WAM DOFs follow the FreeJoint's DOFs
  auto withBase = dynamics::Skeleton::create("wam_with_base");
  auto base = withBase->createJointAndBodyNodePair<dynamics::FreeJoint>();
  solveCopy(copyWithJacobianDLS([&] {
    wam->getRootBodyNode()->copyTo(withBase, base.second);
    return withBase->getEndEffector("ee");
  }));
  EXPECT_EQ(
      withBase->getEndEffector("ee")->getIK()->getDofs(),
      std::vector<std::size_t>({6, 7, 8, 9, 10, 11, 12}));

  // A copy within the WAM's Skeleton selects the new DOFs.
  solveCopy(copyWithJacobianDLS([&] {
    wam->getRootBodyNode()->copyTo(nullptr);
    return wam->getEndEffector(1);
  }));

  // A copy of the last link alone lacks most of the DOFs of IkFast
  dynamics::SkeletonPtr lastLink;
  copyWithJacobianDLS([&] {
    lastLink = wam->getBodyNode("/wam7")->copyAs("wam7_copy");
    return lastLink->getEndEffector("ee");
  });

  // A tool welded to the arm and copied onto another branch of the WAM's
  // Skeleton depends on none of the DOFs of IkFast.
  auto tool = wam->getBodyNode("/wam7")
                  ->createChildJointAndBodyNodePair<dynamics::WeldJoint>()
                  .second;
  tool->createEndEffector("tool")
      ->createIK()
      ->setGradientMethod<dynamics::SharedLibraryIkFast>(
          DART_TEST_IKFAST_LIBRARY, ikFastDofs, std::vector<std::size_t>{2});
  auto branch = wam->getRootBodyNode()
                    ->createChildJointAndBodyNodePair<dynamics::RevoluteJoint>()
                    .second;
  copyWithJacobianDLS(
      [&] { return tool->copyTo(branch).second->getEndEffector(0); });

  EXPECT_TRUE(wam->getPositions().head(7).isApprox(wamPositions));
  EXPECT_EQ(ik->getAnalytical()->getDofs(), ikFastDofs);
}
