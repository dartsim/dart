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

#include <cmath>

#include <gtest/gtest.h>

using namespace dart;
using namespace collision;
using namespace dynamics;

namespace {

constexpr double kDistanceTol = 1e-6;
constexpr double kPi = 3.141592653589793;

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
TEST(DartDistance, SphereSphereOverlapDistance)
{
  auto detector = DARTCollisionDetector::create();

  auto frame1 = SimpleFrame::createShared(Frame::World());
  auto frame2 = SimpleFrame::createShared(Frame::World());

  frame1->setShape(std::make_shared<SphereShape>(1.0));
  frame2->setShape(std::make_shared<SphereShape>(1.0));

  frame1->setTranslation(Eigen::Vector3d::Zero());
  frame2->setTranslation(Eigen::Vector3d(1.0, 0.0, 0.0));

  auto group = detector->createCollisionGroup(frame1.get(), frame2.get());

  DistanceOption option(true, -5.0, nullptr);
  DistanceResult result;

  const double distance = group->distance(option, &result);
  EXPECT_NEAR(distance, -1.0, kDistanceTol);
  EXPECT_NEAR(result.minDistance, -1.0, kDistanceTol);
  EXPECT_NEAR(result.unclampedMinDistance, -1.0, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_TRUE(result.nearestPoint1.isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_TRUE(result.nearestPoint2.isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
}

//==============================================================================
TEST(DartDistance, SpherePlaneDistance)
{
  auto detector = DARTCollisionDetector::create();

  auto sphereFrame = SimpleFrame::createShared(Frame::World());
  auto planeFrame = SimpleFrame::createShared(Frame::World());

  sphereFrame->setShape(std::make_shared<SphereShape>(0.5));
  planeFrame->setShape(std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));

  sphereFrame->setTranslation(Eigen::Vector3d(0.0, 0.0, 2.0));

  auto group = detector->createCollisionGroup(
      sphereFrame.get(), planeFrame.get());

  DistanceOption option(true, 0.0, nullptr);
  DistanceResult result;

  const double distance = group->distance(option, &result);
  EXPECT_NEAR(distance, 1.5, kDistanceTol);
  EXPECT_NEAR(result.minDistance, 1.5, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_TRUE(result.nearestPoint1.isApprox(Eigen::Vector3d(0.0, 0.0, 1.5)));
  EXPECT_TRUE(result.nearestPoint2.isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
}

//==============================================================================
TEST(DartDistance, SphereRotatedPlaneDistance)
{
  auto detector = DARTCollisionDetector::create();

  auto sphereFrame = SimpleFrame::createShared(Frame::World());
  auto planeFrame = SimpleFrame::createShared(Frame::World());

  sphereFrame->setShape(std::make_shared<SphereShape>(0.5));
  planeFrame->setShape(std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));
  planeFrame->setRotation(
      Eigen::AngleAxisd(0.5 * kPi, Eigen::Vector3d::UnitY()).toRotationMatrix());

  sphereFrame->setTranslation(Eigen::Vector3d(2.0, 0.0, 0.0));

  auto group = detector->createCollisionGroup(
      sphereFrame.get(), planeFrame.get());

  DistanceOption option(true, 0.0, nullptr);
  DistanceResult result;

  const double distance = group->distance(option, &result);
  EXPECT_NEAR(distance, 1.5, kDistanceTol);
  EXPECT_NEAR(result.minDistance, 1.5, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_TRUE(result.nearestPoint1.isApprox(Eigen::Vector3d(1.5, 0.0, 0.0)));
  EXPECT_TRUE(result.nearestPoint2.isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
}

//==============================================================================
TEST(DartDistance, SphereBoxDistance)
{
  auto detector = DARTCollisionDetector::create();

  auto sphereFrame = SimpleFrame::createShared(Frame::World());
  auto boxFrame = SimpleFrame::createShared(Frame::World());

  sphereFrame->setShape(std::make_shared<SphereShape>(0.5));
  boxFrame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(2.0, 2.0, 2.0)));

  sphereFrame->setTranslation(Eigen::Vector3d(3.0, 0.0, 0.0));
  boxFrame->setTranslation(Eigen::Vector3d::Zero());

  auto group = detector->createCollisionGroup(
      sphereFrame.get(), boxFrame.get());

  DistanceOption option(true, 0.0, nullptr);
  DistanceResult result;

  const double distance = group->distance(option, &result);
  EXPECT_NEAR(distance, 1.5, kDistanceTol);
  EXPECT_NEAR(result.minDistance, 1.5, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_TRUE(result.nearestPoint1.isApprox(Eigen::Vector3d(2.5, 0.0, 0.0)));
  EXPECT_TRUE(result.nearestPoint2.isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));
}

//==============================================================================
TEST(DartDistance, SphereBoxRotatedDistance)
{
  auto detector = DARTCollisionDetector::create();

  auto sphereFrame = SimpleFrame::createShared(Frame::World());
  auto boxFrame = SimpleFrame::createShared(Frame::World());

  sphereFrame->setShape(std::make_shared<SphereShape>(0.5));
  boxFrame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(2.0, 2.0, 2.0)));

  boxFrame->setRotation(
      Eigen::AngleAxisd(0.25 * kPi, Eigen::Vector3d::UnitZ()).toRotationMatrix());
  sphereFrame->setTranslation(Eigen::Vector3d(3.0, 0.0, 0.0));

  auto group = detector->createCollisionGroup(
      sphereFrame.get(), boxFrame.get());

  DistanceOption option(true, 0.0, nullptr);
  DistanceResult result;

  const double root2 = std::sqrt(2.0);
  const double expected = 3.0 - root2 - 0.5;
  const double distance = group->distance(option, &result);
  EXPECT_NEAR(distance, expected, kDistanceTol);
  EXPECT_NEAR(result.minDistance, expected, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_TRUE(result.nearestPoint1.isApprox(
      Eigen::Vector3d(2.5, 0.0, 0.0), kDistanceTol));
  EXPECT_TRUE(result.nearestPoint2.isApprox(
      Eigen::Vector3d(root2, 0.0, 0.0), kDistanceTol));
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
  EXPECT_TRUE(result.nearestPoint1.isApprox(Eigen::Vector3d(0.0, 0.0, 1.0)));
  EXPECT_TRUE(result.nearestPoint2.isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
}

//==============================================================================
TEST(DartDistance, BoxRotatedPlaneDistance)
{
  auto detector = DARTCollisionDetector::create();

  auto boxFrame = SimpleFrame::createShared(Frame::World());
  auto planeFrame = SimpleFrame::createShared(Frame::World());

  boxFrame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(2.0, 2.0, 2.0)));
  planeFrame->setShape(std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));

  boxFrame->setRotation(
      Eigen::AngleAxisd(0.25 * kPi, Eigen::Vector3d::UnitY()).toRotationMatrix());
  boxFrame->setTranslation(Eigen::Vector3d(0.0, 0.0, 2.0));

  auto group = detector->createCollisionGroup(boxFrame.get(), planeFrame.get());

  DistanceOption option(true, 0.0, nullptr);
  DistanceResult result;

  const double root2 = std::sqrt(2.0);
  const double expected = 2.0 - root2;
  const double distance = group->distance(option, &result);
  EXPECT_NEAR(distance, expected, kDistanceTol);
  EXPECT_NEAR(result.minDistance, expected, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_TRUE(result.nearestPoint1.isApprox(
      Eigen::Vector3d(0.0, 1.0, expected), kDistanceTol));
  EXPECT_TRUE(result.nearestPoint2.isApprox(
      Eigen::Vector3d(0.0, 1.0, 0.0), kDistanceTol));
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
  EXPECT_NEAR(distance, 0.0, kDistanceTol);
  EXPECT_NEAR(result.minDistance, 0.0, kDistanceTol);
  EXPECT_NEAR(result.unclampedMinDistance, -1.5, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_TRUE(result.nearestPoint1.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(result.nearestPoint2.isApprox(Eigen::Vector3d::Zero()));
}

//==============================================================================
TEST(DartDistance, CylinderPlaneDistance)
{
  auto detector = DARTCollisionDetector::create();

  auto cylinderFrame = SimpleFrame::createShared(Frame::World());
  auto planeFrame = SimpleFrame::createShared(Frame::World());

  cylinderFrame->setShape(std::make_shared<CylinderShape>(0.5, 2.0));
  planeFrame->setShape(
      std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));

  cylinderFrame->setTranslation(Eigen::Vector3d(0.0, 0.0, 2.0));

  auto group = detector->createCollisionGroup(
      cylinderFrame.get(), planeFrame.get());

  DistanceOption option(true, 0.0, nullptr);
  DistanceResult result;

  const double distance = group->distance(option, &result);
  EXPECT_NEAR(distance, 1.0, kDistanceTol);
  EXPECT_NEAR(result.minDistance, 1.0, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_TRUE(result.nearestPoint1.isApprox(Eigen::Vector3d(0.0, 0.0, 1.0)));
  EXPECT_TRUE(result.nearestPoint2.isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
}

//==============================================================================
TEST(DartDistance, CylinderPlaneOverlapDistance)
{
  auto detector = DARTCollisionDetector::create();

  auto cylinderFrame = SimpleFrame::createShared(Frame::World());
  auto planeFrame = SimpleFrame::createShared(Frame::World());

  cylinderFrame->setShape(std::make_shared<CylinderShape>(0.5, 2.0));
  planeFrame->setShape(
      std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));

  cylinderFrame->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.2));

  auto group = detector->createCollisionGroup(
      cylinderFrame.get(), planeFrame.get());

  DistanceOption option(true, -5.0, nullptr);
  DistanceResult result;

  const double distance = group->distance(option, &result);
  EXPECT_NEAR(distance, -0.8, kDistanceTol);
  EXPECT_NEAR(result.minDistance, -0.8, kDistanceTol);
  EXPECT_NEAR(result.unclampedMinDistance, -0.8, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_TRUE(result.nearestPoint1.isApprox(
      Eigen::Vector3d(0.0, 0.0, -0.8), kDistanceTol));
  EXPECT_TRUE(result.nearestPoint2.isApprox(
      Eigen::Vector3d(0.0, 0.0, 0.0), kDistanceTol));
}

//==============================================================================
TEST(DartDistance, CylinderRotatedPlaneDistance)
{
  auto detector = DARTCollisionDetector::create();

  auto cylinderFrame = SimpleFrame::createShared(Frame::World());
  auto planeFrame = SimpleFrame::createShared(Frame::World());

  cylinderFrame->setShape(std::make_shared<CylinderShape>(0.5, 2.0));
  planeFrame->setShape(
      std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));

  cylinderFrame->setRotation(
      Eigen::AngleAxisd(0.5 * kPi, Eigen::Vector3d::UnitY()).toRotationMatrix());
  cylinderFrame->setTranslation(Eigen::Vector3d(0.0, 0.0, 2.0));

  auto group = detector->createCollisionGroup(
      cylinderFrame.get(), planeFrame.get());

  DistanceOption option(true, 0.0, nullptr);
  DistanceResult result;

  const double distance = group->distance(option, &result);
  EXPECT_NEAR(distance, 1.5, kDistanceTol);
  EXPECT_NEAR(result.minDistance, 1.5, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_TRUE(result.nearestPoint1.isApprox(
      Eigen::Vector3d(1.0, 0.0, 1.5), kDistanceTol));
  EXPECT_TRUE(result.nearestPoint2.isApprox(
      Eigen::Vector3d(1.0, 0.0, 0.0), kDistanceTol));
}

//==============================================================================
TEST(DartDistance, SphereCylinderDistance)
{
  auto detector = DARTCollisionDetector::create();

  auto sphereFrame = SimpleFrame::createShared(Frame::World());
  auto cylinderFrame = SimpleFrame::createShared(Frame::World());

  sphereFrame->setShape(std::make_shared<SphereShape>(0.5));
  cylinderFrame->setShape(std::make_shared<CylinderShape>(1.0, 2.0));

  sphereFrame->setTranslation(Eigen::Vector3d(3.0, 0.0, 0.0));
  cylinderFrame->setTranslation(Eigen::Vector3d::Zero());

  auto group = detector->createCollisionGroup(
      sphereFrame.get(), cylinderFrame.get());

  DistanceOption option(true, 0.0, nullptr);
  DistanceResult result;

  const double distance = group->distance(option, &result);
  EXPECT_NEAR(distance, 1.5, kDistanceTol);
  EXPECT_NEAR(result.minDistance, 1.5, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_TRUE(result.nearestPoint1.isApprox(Eigen::Vector3d(2.5, 0.0, 0.0)));
  EXPECT_TRUE(result.nearestPoint2.isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));
}

//==============================================================================
TEST(DartDistance, BoxBoxDistance)
{
  auto detector = DARTCollisionDetector::create();

  auto frame1 = SimpleFrame::createShared(Frame::World());
  auto frame2 = SimpleFrame::createShared(Frame::World());

  frame1->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(2.0, 2.0, 2.0)));
  frame2->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(2.0, 2.0, 2.0)));

  frame1->setTranslation(Eigen::Vector3d::Zero());
  frame2->setTranslation(Eigen::Vector3d(3.0, 0.0, 0.0));

  auto group = detector->createCollisionGroup(frame1.get(), frame2.get());

  DistanceOption option(true, 0.0, nullptr);
  DistanceResult result;

  const double distance = group->distance(option, &result);
  EXPECT_NEAR(distance, 1.0, kDistanceTol);
  EXPECT_NEAR(result.minDistance, 1.0, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_TRUE(result.nearestPoint1.isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_TRUE(result.nearestPoint2.isApprox(Eigen::Vector3d(2.0, 0.0, 0.0)));
}

//==============================================================================
TEST(DartDistance, BoxBoxDiagonalDistance)
{
  auto detector = DARTCollisionDetector::create();

  auto frame1 = SimpleFrame::createShared(Frame::World());
  auto frame2 = SimpleFrame::createShared(Frame::World());

  frame1->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(2.0, 2.0, 2.0)));
  frame2->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(2.0, 2.0, 2.0)));

  frame1->setTranslation(Eigen::Vector3d::Zero());
  frame2->setTranslation(Eigen::Vector3d(3.0, 4.0, 0.0));

  auto group = detector->createCollisionGroup(frame1.get(), frame2.get());

  DistanceOption option(true, 0.0, nullptr);
  DistanceResult result;

  const double expected = std::sqrt(5.0);
  const double distance = group->distance(option, &result);
  EXPECT_NEAR(distance, expected, kDistanceTol);
  EXPECT_NEAR(result.minDistance, expected, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_TRUE(result.nearestPoint1.isApprox(
      Eigen::Vector3d(1.0, 1.0, 0.0), kDistanceTol));
  EXPECT_TRUE(result.nearestPoint2.isApprox(
      Eigen::Vector3d(2.0, 3.0, 0.0), kDistanceTol));
}

//==============================================================================
TEST(DartDistance, BoxBoxOverlapDistance)
{
  auto detector = DARTCollisionDetector::create();

  auto frame1 = SimpleFrame::createShared(Frame::World());
  auto frame2 = SimpleFrame::createShared(Frame::World());

  frame1->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(2.0, 2.0, 2.0)));
  frame2->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(2.0, 2.0, 2.0)));

  frame1->setTranslation(Eigen::Vector3d::Zero());
  frame2->setTranslation(Eigen::Vector3d(0.5, 0.0, 0.0));

  auto group = detector->createCollisionGroup(frame1.get(), frame2.get());

  DistanceOption option(true, -5.0, nullptr);
  DistanceResult result;

  const double distance = group->distance(option, &result);
  EXPECT_NEAR(distance, -1.5, kDistanceTol);
  EXPECT_NEAR(result.minDistance, -1.5, kDistanceTol);
  EXPECT_NEAR(result.unclampedMinDistance, -1.5, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_TRUE(result.nearestPoint1.isApprox(
      Eigen::Vector3d(1.0, 0.0, 0.0), kDistanceTol));
  EXPECT_TRUE(result.nearestPoint2.isApprox(
      Eigen::Vector3d(-0.5, 0.0, 0.0), kDistanceTol));
}

//==============================================================================
TEST(DartDistance, BoxCylinderDistance)
{
  auto detector = DARTCollisionDetector::create();

  auto boxFrame = SimpleFrame::createShared(Frame::World());
  auto cylinderFrame = SimpleFrame::createShared(Frame::World());

  boxFrame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(2.0, 2.0, 2.0)));
  cylinderFrame->setShape(std::make_shared<CylinderShape>(1.0, 2.0));

  boxFrame->setTranslation(Eigen::Vector3d::Zero());
  cylinderFrame->setTranslation(Eigen::Vector3d(3.0, 0.0, 0.0));

  auto group = detector->createCollisionGroup(
      boxFrame.get(), cylinderFrame.get());

  DistanceOption option(true, 0.0, nullptr);
  DistanceResult result;

  const double distance = group->distance(option, &result);
  EXPECT_NEAR(distance, 1.0, kDistanceTol);
  EXPECT_NEAR(result.minDistance, 1.0, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_TRUE(result.nearestPoint1.isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_TRUE(result.nearestPoint2.isApprox(Eigen::Vector3d(2.0, 0.0, 0.0)));
}

//==============================================================================
TEST(DartDistance, CylinderCylinderDistance)
{
  auto detector = DARTCollisionDetector::create();

  auto frame1 = SimpleFrame::createShared(Frame::World());
  auto frame2 = SimpleFrame::createShared(Frame::World());

  frame1->setShape(std::make_shared<CylinderShape>(1.0, 2.0));
  frame2->setShape(std::make_shared<CylinderShape>(1.0, 2.0));

  frame1->setTranslation(Eigen::Vector3d::Zero());
  frame2->setTranslation(Eigen::Vector3d(3.0, 0.0, 0.0));

  auto group = detector->createCollisionGroup(frame1.get(), frame2.get());

  DistanceOption option(true, 0.0, nullptr);
  DistanceResult result;

  const double distance = group->distance(option, &result);
  EXPECT_NEAR(distance, 1.0, kDistanceTol);
  EXPECT_NEAR(result.minDistance, 1.0, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_TRUE(result.nearestPoint1.isApprox(
      Eigen::Vector3d(1.0, 0.0, 0.0), kDistanceTol));
  EXPECT_TRUE(result.nearestPoint2.isApprox(
      Eigen::Vector3d(2.0, 0.0, 0.0), kDistanceTol));
}

//==============================================================================
TEST(DartDistance, CylinderCylinderDiagonalDistance)
{
  auto detector = DARTCollisionDetector::create();

  auto frame1 = SimpleFrame::createShared(Frame::World());
  auto frame2 = SimpleFrame::createShared(Frame::World());

  frame1->setShape(std::make_shared<CylinderShape>(0.5, 2.0));
  frame2->setShape(std::make_shared<CylinderShape>(0.5, 2.0));

  frame1->setTranslation(Eigen::Vector3d::Zero());
  frame2->setTranslation(Eigen::Vector3d(2.0, 2.0, 0.0));

  auto group = detector->createCollisionGroup(frame1.get(), frame2.get());

  DistanceOption option(true, 0.0, nullptr);
  DistanceResult result;

  const double expected = std::sqrt(8.0) - 1.0;
  const double distance = group->distance(option, &result);
  EXPECT_NEAR(distance, expected, kDistanceTol);
  EXPECT_NEAR(result.minDistance, expected, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_TRUE(result.nearestPoint1.isApprox(
      Eigen::Vector3d(0.5 / std::sqrt(2.0), 0.5 / std::sqrt(2.0), 0.0),
      kDistanceTol));
  EXPECT_TRUE(result.nearestPoint2.isApprox(
      Eigen::Vector3d(2.0 - 0.5 / std::sqrt(2.0),
                      2.0 - 0.5 / std::sqrt(2.0),
                      0.0),
      kDistanceTol));
}

//==============================================================================
TEST(DartDistance, GroupGroupDistance)
{
  auto detector = DARTCollisionDetector::create();

  auto sphereFrame1 = SimpleFrame::createShared(Frame::World());
  auto boxFrame = SimpleFrame::createShared(Frame::World());
  auto sphereFrame2 = SimpleFrame::createShared(Frame::World());

  sphereFrame1->setShape(std::make_shared<SphereShape>(0.5));
  boxFrame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0)));
  sphereFrame2->setShape(std::make_shared<SphereShape>(0.5));

  sphereFrame1->setTranslation(Eigen::Vector3d::Zero());
  boxFrame->setTranslation(Eigen::Vector3d(5.0, 0.0, 0.0));
  sphereFrame2->setTranslation(Eigen::Vector3d(2.0, 0.0, 0.0));

  auto group1 = detector->createCollisionGroup(
      sphereFrame1.get(), boxFrame.get());
  auto group2 = detector->createCollisionGroup(sphereFrame2.get());

  DistanceOption option(true, 0.0, nullptr);
  DistanceResult result;

  const double distance = group1->distance(group2.get(), option, &result);
  EXPECT_NEAR(distance, 1.0, kDistanceTol);
  EXPECT_NEAR(result.minDistance, 1.0, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_EQ(result.shapeFrame1, sphereFrame1.get());
  EXPECT_EQ(result.shapeFrame2, sphereFrame2.get());
  EXPECT_TRUE(result.nearestPoint1.isApprox(
      Eigen::Vector3d(0.5, 0.0, 0.0), kDistanceTol));
  EXPECT_TRUE(result.nearestPoint2.isApprox(
      Eigen::Vector3d(1.5, 0.0, 0.0), kDistanceTol));
}

//==============================================================================
TEST(DartDistance, GroupGroupDistanceWithFilter)
{
  auto detector = DARTCollisionDetector::create();

  auto sphereFrame1 = SimpleFrame::createShared(Frame::World());
  auto boxFrame = SimpleFrame::createShared(Frame::World());
  auto sphereFrame2 = SimpleFrame::createShared(Frame::World());

  sphereFrame1->setShape(std::make_shared<SphereShape>(0.5));
  boxFrame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0)));
  sphereFrame2->setShape(std::make_shared<SphereShape>(0.5));

  sphereFrame1->setTranslation(Eigen::Vector3d::Zero());
  boxFrame->setTranslation(Eigen::Vector3d(5.0, 0.0, 0.0));
  sphereFrame2->setTranslation(Eigen::Vector3d(2.0, 0.0, 0.0));

  auto group1 = detector->createCollisionGroup(
      sphereFrame1.get(), boxFrame.get());
  auto group2 = detector->createCollisionGroup(sphereFrame2.get());

  struct ShapeFrameDistanceFilter final : DistanceFilter
  {
    const dynamics::ShapeFrame* skipFrame{nullptr};

    bool needDistance(
        const CollisionObject* object1,
        const CollisionObject* object2) const override
    {
      return object1->getShapeFrame() != skipFrame
             && object2->getShapeFrame() != skipFrame;
    }
  };

  ShapeFrameDistanceFilter filter;
  filter.skipFrame = sphereFrame1.get();

  DistanceOption option(true, 0.0, &filter);
  DistanceResult result;

  const double distance = group1->distance(group2.get(), option, &result);
  EXPECT_NEAR(distance, 2.0, kDistanceTol);
  EXPECT_NEAR(result.minDistance, 2.0, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_EQ(result.shapeFrame1, boxFrame.get());
  EXPECT_EQ(result.shapeFrame2, sphereFrame2.get());
  EXPECT_TRUE(result.nearestPoint1.isApprox(
      Eigen::Vector3d(4.5, 0.0, 0.0), kDistanceTol));
  EXPECT_TRUE(result.nearestPoint2.isApprox(
      Eigen::Vector3d(2.5, 0.0, 0.0), kDistanceTol));
}

//==============================================================================
TEST(DartDistance, GroupGroupDistanceSkipsSharedObjects)
{
  auto detector = DARTCollisionDetector::create();

  auto sharedFrame = SimpleFrame::createShared(Frame::World());
  auto sphereFrame1 = SimpleFrame::createShared(Frame::World());
  auto sphereFrame2 = SimpleFrame::createShared(Frame::World());

  sharedFrame->setShape(std::make_shared<SphereShape>(0.5));
  sphereFrame1->setShape(std::make_shared<SphereShape>(0.5));
  sphereFrame2->setShape(std::make_shared<SphereShape>(0.5));

  sharedFrame->setTranslation(Eigen::Vector3d::Zero());
  sphereFrame1->setTranslation(Eigen::Vector3d(3.0, 0.0, 0.0));
  sphereFrame2->setTranslation(Eigen::Vector3d(5.0, 0.0, 0.0));

  auto group1 = detector->createCollisionGroup(
      sharedFrame.get(), sphereFrame1.get());
  auto group2 = detector->createCollisionGroup(
      sharedFrame.get(), sphereFrame2.get());

  DistanceOption option(true, 0.0, nullptr);
  DistanceResult result;

  const double distance = group1->distance(group2.get(), option, &result);
  EXPECT_NEAR(distance, 1.0, kDistanceTol);
  EXPECT_NEAR(result.minDistance, 1.0, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_EQ(result.shapeFrame1, sphereFrame1.get());
  EXPECT_EQ(result.shapeFrame2, sphereFrame2.get());
  EXPECT_TRUE(result.nearestPoint1.isApprox(
      Eigen::Vector3d(3.5, 0.0, 0.0), kDistanceTol));
  EXPECT_TRUE(result.nearestPoint2.isApprox(
      Eigen::Vector3d(4.5, 0.0, 0.0), kDistanceTol));
}

//==============================================================================
TEST(DartDistance, GroupDistanceWithFilter)
{
  auto detector = DARTCollisionDetector::create();

  auto sphereFrame1 = SimpleFrame::createShared(Frame::World());
  auto sphereFrame2 = SimpleFrame::createShared(Frame::World());
  auto sphereFrame3 = SimpleFrame::createShared(Frame::World());

  sphereFrame1->setShape(std::make_shared<SphereShape>(0.5));
  sphereFrame2->setShape(std::make_shared<SphereShape>(0.5));
  sphereFrame3->setShape(std::make_shared<SphereShape>(0.5));

  sphereFrame1->setTranslation(Eigen::Vector3d::Zero());
  sphereFrame2->setTranslation(Eigen::Vector3d(2.0, 0.0, 0.0));
  sphereFrame3->setTranslation(Eigen::Vector3d(5.0, 0.0, 0.0));

  auto group = detector->createCollisionGroup(
      sphereFrame1.get(), sphereFrame2.get(), sphereFrame3.get());

  struct ShapeFrameDistanceFilter final : DistanceFilter
  {
    const dynamics::ShapeFrame* skipFrame{nullptr};

    bool needDistance(
        const CollisionObject* object1,
        const CollisionObject* object2) const override
    {
      return object1->getShapeFrame() != skipFrame
             && object2->getShapeFrame() != skipFrame;
    }
  };

  ShapeFrameDistanceFilter filter;
  filter.skipFrame = sphereFrame1.get();

  DistanceOption option(true, 0.0, &filter);
  DistanceResult result;

  const double distance = group->distance(option, &result);
  EXPECT_NEAR(distance, 2.0, kDistanceTol);
  EXPECT_NEAR(result.minDistance, 2.0, kDistanceTol);
  EXPECT_TRUE(result.found());
  EXPECT_EQ(result.shapeFrame1, sphereFrame2.get());
  EXPECT_EQ(result.shapeFrame2, sphereFrame3.get());
  EXPECT_TRUE(result.nearestPoint1.isApprox(
      Eigen::Vector3d(2.5, 0.0, 0.0), kDistanceTol));
  EXPECT_TRUE(result.nearestPoint2.isApprox(
      Eigen::Vector3d(4.5, 0.0, 0.0), kDistanceTol));
}
