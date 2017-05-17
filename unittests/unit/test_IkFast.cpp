/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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
      = ik->setGradientMethod<dynamics::SharedLibraryIkFast>("name doesn't exist");
  EXPECT_EQ(ikfast.isGood(), false);
}

//==============================================================================
TEST(IkFast, LoadWamArmIk)
{
  utils::DartLoader urdfParser;
  urdfParser.addPackageDirectory("herb_description", DART_DATA_PATH"/urdf/wam");
  auto wam = urdfParser.parseSkeleton(DART_DATA_PATH"/urdf/wam/wam.urdf");
  EXPECT_NE(wam, nullptr);

  auto wam7 = wam->getBodyNode("/wam7");
  auto ee = wam7->createEndEffector("ee");
  auto ik = ee->createIK();
  auto targetFrame
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  targetFrame->setRotation(Eigen::Matrix3d::Identity());

  ik->setTarget(targetFrame);
  ik->setHierarchyLevel(1);
  std::string libName = "libwamIk";
#if DART_OS_LINUX && !NDEBUG
  libName += "d";
#endif
  auto ikfastGradientMethod
      = ik->setGradientMethod<dynamics::SharedLibraryIkFast>(libName);
  EXPECT_EQ(ikfastGradientMethod.getDofs().size(), 6);

  targetFrame->setTranslation(Eigen::Vector3d(0, 0, 0.5));
  ik->solve();
//  auto solved = ik->solve();
//  if (solved)
//    EXPECT_TRUE(!ik->getSolutions().empty());
}
