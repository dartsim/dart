/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart/All.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace collision;
using namespace dynamics;

namespace {

constexpr double kDistanceTol = 1e-6;

} // namespace

//==============================================================================
TEST(DartDistance, SphereSphereDistance)
{
  auto detector = DARTCollisionDetector::create();

  auto frame1 = SimpleFrame::createShared(Frame::World());
  auto frame2 = SimpleFrame::createShared(Frame::World());

  frame1->setShape(std::make_shared<SphereShape>(1.0));
  frame2->setShape(std::make_shared<SphereShape>(0.5));

  frame1->setTranslation(Eigen::Vector3d::Zero());
  frame2->setTranslation(Eigen::Vector3d(3.0, 0.0, 0.0));

  auto group = detector->createCollisionGroup(frame1.get(), frame2.get());

  DistanceOption option(true, 0.0, nullptr);
  DistanceResult result;

  const double distance = group->distance(option, &result);
  EXPECT_NEAR(distance, 1.5, kDistanceTol);
  EXPECT_NEAR(result.minDistance, 1.5, kDistanceTol);
  EXPECT_NEAR(result.unclampedMinDistance, 1.5, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_TRUE(result.nearestPoint1.isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_TRUE(result.nearestPoint2.isApprox(Eigen::Vector3d(2.5, 0.0, 0.0)));

  DistanceOption clampOption(true, 2.0, nullptr);
  result.clear();
  const double clamped = group->distance(clampOption, &result);
  EXPECT_NEAR(clamped, 2.0, kDistanceTol);
  EXPECT_NEAR(result.minDistance, 2.0, kDistanceTol);
  EXPECT_NEAR(result.unclampedMinDistance, 1.5, kDistanceTol);
}

//==============================================================================
TEST(DartDistance, BoxPlaneDistance)
{
  auto detector = DARTCollisionDetector::create();

  auto boxFrame = SimpleFrame::createShared(Frame::World());
  auto planeFrame = SimpleFrame::createShared(Frame::World());

  boxFrame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(2.0, 2.0, 2.0)));
  planeFrame->setShape(std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));

  boxFrame->setTranslation(Eigen::Vector3d(0.0, 0.0, 2.0));

  auto group = detector->createCollisionGroup(boxFrame.get(), planeFrame.get());

  DistanceOption option(true, 0.0, nullptr);
  DistanceResult result;

  const double distance = group->distance(option, &result);
  EXPECT_NEAR(distance, 1.0, kDistanceTol);
  EXPECT_NEAR(result.minDistance, 1.0, kDistanceTol);
  EXPECT_TRUE(result.found());
}

//==============================================================================
TEST(DartDistance, SphereInsideBoxDistance)
{
  auto detector = DARTCollisionDetector::create();

  auto boxFrame = SimpleFrame::createShared(Frame::World());
  auto sphereFrame = SimpleFrame::createShared(Frame::World());

  boxFrame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(2.0, 2.0, 2.0)));
  sphereFrame->setShape(std::make_shared<SphereShape>(0.5));

  boxFrame->setTranslation(Eigen::Vector3d::Zero());
  sphereFrame->setTranslation(Eigen::Vector3d::Zero());

  auto group = detector->createCollisionGroup(boxFrame.get(), sphereFrame.get());

  DistanceOption option(false, 0.0, nullptr);
  DistanceResult result;

  const double distance = group->distance(option, &result);
  EXPECT_NEAR(distance, -1.5, kDistanceTol);
  EXPECT_NEAR(result.minDistance, -1.5, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_TRUE(result.nearestPoint1.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(result.nearestPoint2.isApprox(Eigen::Vector3d::Zero()));
}
