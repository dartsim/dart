/*
 * Copyright (c) 2011-2017, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <gtest/gtest.h>
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include "TestHelpers.hpp"

using namespace dart;

//==============================================================================
TEST(IkFast, FailedToLoadSharedLibrary)
{
  auto skel = dynamics::Skeleton::create();
  EXPECT_NE(skel, nullptr);

  auto bodyNode
      = skel->createJointAndBodyNodePair<dynamics::FreeJoint>().second;

  auto ee = bodyNode->createEndEffector("ee");
  auto ik = ee->createIK();
  auto ikfast
      = ik->setGradientMethod<dynamics::SharedLibraryIkFast>(
        "doesn't exist",
        std::vector<std::size_t>(),
        std::vector<std::size_t>());
  EXPECT_EQ(ikfast.isConfigured(), false);
}

//==============================================================================
dynamics::SkeletonPtr loadWamArm()
{
  utils::DartLoader urdfParser;
  urdfParser.addPackageDirectory("herb_description", DART_DATA_PATH"/urdf/wam");
  auto wamArm = urdfParser.parseSkeleton(DART_DATA_PATH"/urdf/wam/wam.urdf");
  EXPECT_NE(wamArm, nullptr);

  return wamArm;
}

//==============================================================================
void loadWamArmIkFastSolver(InverseKinematics& ik)
{
  std::string libName = "libGeneratedWamIkFast";
#if (DART_OS_LINUX || DART_OS_MACOS) && !NDEBUG
  libName += "d";
#endif
#if DART_OS_LINUX
  libName += ".so";
#elif DART_OS_MACOS
  libName += ".dylib";
#elif DART_OS_WINDOWS
  libName += ".dll";
#endif
  std::vector<std::size_t> ikFastDofs{0, 1, 3, 4, 5, 6};
  std::vector<std::size_t> ikFastFreeDofs{2};
  ik.setGradientMethod<dynamics::SharedLibraryIkFast>(
      libName, ikFastDofs, ikFastFreeDofs);
}

//==============================================================================
TEST(IkFast, LoadWamArmIk)
{
  auto wam = loadWamArm();

  auto wam7 = wam->getBodyNode("/wam7");
  auto ee = wam7->createEndEffector("ee");
  auto ik = ee->createIK();
  auto targetFrame
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  targetFrame->setRotation(Eigen::Matrix3d::Identity());

  ik->setTarget(targetFrame);
  ik->setHierarchyLevel(1);

  loadWamArmIkFastSolver(*ik);

  auto analytical = ik->getAnalytical();
  EXPECT_NE(analytical, nullptr);
  EXPECT_EQ(analytical->getDofs().size(), 6);

  auto ikfast = dynamic_cast<dynamics::SharedLibraryIkFast*>(analytical);
  EXPECT_NE(ikfast, nullptr);

  targetFrame->setTranslation(Eigen::Vector3d(0, 0, 0.5));
  auto solutions = ikfast->getSolutions(targetFrame->getTransform());
  EXPECT_TRUE(!solutions.empty());

  const auto dofs = ikfast->getDofs();

  for (const auto& solution : solutions)
  {
    EXPECT_EQ(solution.mConfig.size(), 6);

    if (solution.mValidity != InverseKinematics::Analytical::VALID)
      continue;

    wam->setPositions(dofs, solution.mConfig);
    Eigen::Isometry3d newTf = ee->getTransform();
    EXPECT_TRUE(equals(targetFrame->getTransform(), newTf, 1e-2));
  }
}

//==============================================================================
void verifySolution(
    const dynamics::InverseKinematics& ik,
    const dynamics::BodyNode& bodyNode)
{
  std::cout << "target:\n"
            << ik.getTarget()->getTransform().matrix() << "\n";
  std::cout << "actual:\n"
            << bodyNode.getTransform().matrix() << "\n";

  EXPECT_TRUE(equals(ik.getTarget()->getTransform().matrix(),
                     bodyNode.getTransform().matrix(), 1e-4));
}

//==============================================================================
TEST(IkFast, SimpleBenchmark)
{
  auto wamArm1 = loadWamArm();
  auto wamArm2 = loadWamArm();

  auto wamArm1_wam7 = wamArm1->getBodyNode("/wam7");
  auto wamArm2_wam7 = wamArm2->getBodyNode("/wam7");

  const auto numDofs = wamArm2_wam7->getNumDependentGenCoords();
  std::cout << "numDofs: " << numDofs << "\n";

  auto ee1 = wamArm1_wam7->createEndEffector("ee");
  auto ee2 = wamArm2_wam7->createEndEffector("ee");
  auto ik1 = ee1->createIK();
  auto ik2 = ee2->createIK();
  auto targetFrame1
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  ik1->setHierarchyLevel(1);
  ik2->setHierarchyLevel(1);

  loadWamArmIkFastSolver(*ik1);

  ik2->getErrorMethod().setBounds(Eigen::VectorXd::Constant(numDofs, -1e-8),
                                  Eigen::VectorXd::Constant(numDofs,  1e-8));
  ik2->getSolver()->setNumMaxIterations(100);

  targetFrame1->setRotation(Eigen::Matrix3d::Identity());
  targetFrame1->setTranslation(Eigen::Vector3d(0, 0, 0.5));

//  ik1->setTarget(targetFrame1);
  ik2->setTarget(targetFrame1);

  ik1->solve();
  ik2->solve();

  verifySolution(*ik1, *wamArm1_wam7);
  verifySolution(*ik2, *wamArm2_wam7);
}
