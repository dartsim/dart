/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include <sstream>

#include <gtest/gtest.h>

#include "dart/dart.hpp"
#include "dart/test/math/GTestUtils.hpp"

using namespace dart;
using namespace dart::dynamics;

//==============================================================================
TEST(IkFast, WrapCyclicSolution) {
  const auto pi = math::constantsd::pi();

  double sol;

  // Invalid bounds (lb > ub)
  EXPECT_FALSE(dynamics::wrapCyclicSolution(0, 10, -10, sol));

  // Current value is in the lmits, but solution is lesser than lower limit.
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

  // Current value is in the lmits, but solution is greater than upper limit.
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
TEST(IkFast, FailedToLoadSharedLibrary) {
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
TEST(IkFast, LoadWamArmIk) {
  io::DartLoader urdfParser;
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
  std::stringstream ss;
  ss << DART_SHARED_LIB_PREFIX << "GeneratedWamIkFast";
#if (DART_OS_LINUX || DART_OS_MACOS) && !NDEBUG
  ss << "d";
#endif
  ss << "." << DART_SHARED_LIB_EXTENSION;
  std::string libName = ss.str();
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
    EXPECT_TRUE(test::equals(targetFrame->getTransform(), newTf, 1e-2));
  }
}
