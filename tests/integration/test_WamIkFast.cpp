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

#include "demo_models/WamIkFast.hpp"

#include <dart/config.hpp>

#include <dart/utils/urdf/DartLoader.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/EndEffector.hpp>
#include <dart/dynamics/IkFast.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

namespace {

//==============================================================================
dart::dynamics::SkeletonPtr loadWamIkFastRobot()
{
  dart::utils::DartLoader loader;
  loader.addPackageDirectory("herb_description", DART_DATA_PATH "/urdf/wam");
  return loader.parseSkeleton(DART_DATA_PATH "/urdf/wam/wam.urdf");
}

} // namespace

//==============================================================================
TEST(WamIkFastTest, SolvesReachablePose)
{
  auto wam = loadWamIkFastRobot();
  ASSERT_NE(wam, nullptr);
  auto* ee = wam->getBodyNode("/wam7")->createEndEffector("ee");
  auto ik = ee->getIK(true);
  dart_demos::setWamIkFast(*ik);
  auto* solver = dynamic_cast<dart::dynamics::IkFast*>(ik->getAnalytical());
  ASSERT_NE(solver, nullptr);
  EXPECT_TRUE(solver->isConfigured());
  EXPECT_EQ(solver->getDofs(), (std::vector<std::size_t>{0, 1, 3, 4, 5, 6}));
  EXPECT_EQ(solver->getFreeDofs(), (std::vector<std::size_t>{2}));
  EXPECT_EQ(solver->getNumJoints2(), 7u);
  EXPECT_EQ(solver->getNumFreeParameters2(), 1u);
  EXPECT_EQ(solver->getIkType2(), dart::dynamics::IkFast::TRANSFORM_6D);
  EXPECT_EQ(solver->getIkFastVersion2(), "71");
  EXPECT_EQ(solver->getKinematicsHash2(), "268c2c509bc1bb657da055f0ef2eb7e1");

  Eigen::VectorXd targetPose(7);
  targetPose << 0.1, 0.05, 0.05, 0.1, 0.02, 0.05, 0.01;
  wam->setPositions(targetPose);
  const Eigen::Isometry3d target = ee->getTransform();
  wam->setPositions(Eigen::VectorXd::Zero(7));
  wam->getDof(2)->setPosition(targetPose[2]);

  const auto solutions = solver->getSolutions(target);
  ASSERT_FALSE(solutions.empty());
  std::size_t validSolutions = 0;
  for (const auto& solution : solutions) {
    ASSERT_EQ(solution.mConfig.size(), 6);
    if (solution.mValidity
        != dart::dynamics::InverseKinematics::Analytical::VALID)
      continue;
    ++validSolutions;
    wam->setPositions(solver->getDofs(), solution.mConfig);
    // Generated ideal rotations differ from rounded URDF rpy by a few
    // microradians.
    EXPECT_TRUE(ee->getTransform().matrix().isApprox(target.matrix(), 1e-5));
    EXPECT_DOUBLE_EQ(wam->getDof(2)->getPosition(), targetPose[2]);
  }
  EXPECT_GT(validSolutions, 0u);

  wam->setPositions(Eigen::VectorXd::Zero(7));
  wam->getDof(2)->setPosition(targetPose[2]);
  ik->getTarget()->setTransform(target);
  EXPECT_TRUE(ik->solveAndApply(true));
  EXPECT_TRUE(ee->getTransform().matrix().isApprox(target.matrix(), 1e-6));
}

//==============================================================================
TEST(WamIkFastTest, PreservesBodyEndpointWithSceneEndEffectorOffset)
{
  auto wam = loadWamIkFastRobot();
  ASSERT_NE(wam, nullptr);
  auto* body = wam->getBodyNode("/wam7");
  auto* ee = body->createEndEffector("ee");
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(0.0, 0.0, -0.09);
  ee->setDefaultRelativeTransform(offset, true);
  auto ik = ee->getIK(true);
  dart_demos::setWamIkFast(*ik);
  auto* solver = ik->getAnalytical();
  ASSERT_NE(solver, nullptr);

  Eigen::VectorXd targetPose(7);
  targetPose << 0.3, -0.6, 0.4, 1.1, -0.3, 0.5, 0.2;
  wam->setPositions(targetPose);
  const Eigen::Isometry3d target = ee->getTransform();
  wam->setPositions(Eigen::VectorXd::Zero(7));
  wam->getDof(2)->setPosition(targetPose[2]);
  const auto solutions = solver->getSolutions(target);
  ASSERT_FALSE(solutions.empty());
  ASSERT_EQ(
      solutions.front().mValidity,
      dart::dynamics::InverseKinematics::Analytical::VALID);
  wam->setPositions(solver->getDofs(), solutions.front().mConfig);
  // The source solver targets /wam7; its hand transform remains an extra
  // offset.
  EXPECT_TRUE(body->getTransform().matrix().isApprox(target.matrix(), 1e-5));
  EXPECT_TRUE(
      ee->getTransform().matrix().isApprox((target * offset).matrix(), 1e-5));

  wam->setPositions(Eigen::VectorXd::Zero(7));
  wam->getDof(2)->setPosition(targetPose[2]);
  ik->getTarget()->setTransform(target);
  EXPECT_FALSE(ik->solveAndApply(true));
  EXPECT_TRUE(body->getTransform().matrix().isApprox(target.matrix(), 1e-5));
  EXPECT_TRUE(
      ee->getTransform().matrix().isApprox((target * offset).matrix(), 1e-5));
  EXPECT_NEAR(
      (ee->getTransform().translation() - target.translation()).norm(),
      0.09,
      1e-5);
}

//==============================================================================
TEST(WamIkFastTest, ClonePreservesMethodAndGradientProperties)
{
  auto wam = loadWamIkFastRobot();
  ASSERT_NE(wam, nullptr);
  auto* ee = wam->getBodyNode("/wam7")->createEndEffector("ee");
  dart_demos::setWamIkFast(*ee->getIK(true));
  ee->getIK()->getGradientMethod().setComponentWiseClamp(0.031);

  auto clone = wam->cloneSkeleton();
  ASSERT_NE(clone, nullptr);
  auto cloneIk = clone->getEndEffector("ee")->getIK();
  ASSERT_NE(cloneIk, nullptr);
  auto* solver
      = dynamic_cast<dart::dynamics::IkFast*>(cloneIk->getAnalytical());
  ASSERT_NE(solver, nullptr);
  EXPECT_TRUE(solver->isConfigured());
  EXPECT_EQ(solver->getDofs(), (std::vector<std::size_t>{0, 1, 3, 4, 5, 6}));
  EXPECT_EQ(solver->getMethodName(), "IKFast");
  EXPECT_DOUBLE_EQ(solver->getComponentWiseClamp(), 0.031);
  EXPECT_EQ(solver->getFreeDofs(), (std::vector<std::size_t>{2}));
}

//==============================================================================
TEST(WamIkFastTest, RejectsIncompatibleNodeAndDofMap)
{
  auto wam = loadWamIkFastRobot();
  ASSERT_NE(wam, nullptr);
  auto* lastBody = wam->getBodyNode("/wam7");
  EXPECT_THROW(
      dart_demos::setWamIkFast(*lastBody->getIK(true)), std::invalid_argument);
  auto* wrongEe = wam->getBodyNode("/wam6")->createEndEffector("wrong");
  EXPECT_THROW(
      dart_demos::setWamIkFast(*wrongEe->getIK(true)), std::invalid_argument);
  auto* ee = lastBody->createEndEffector("ee");
  wam->getDof(2)->setName("renamed");
  EXPECT_THROW(
      dart_demos::setWamIkFast(*ee->getIK(true)), std::invalid_argument);
}
