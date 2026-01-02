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

//==============================================================================
void testBasicInterface(
    const std::shared_ptr<CollisionDetector>& cd, double tol = 1e-12)
{
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());

  ShapePtr shape1(new EllipsoidShape(Eigen::Vector3d(1.0, 1.0, 1.0)));
  ShapePtr shape2(new EllipsoidShape(Eigen::Vector3d(0.5, 0.5, 0.5)));
  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);

  auto group1 = cd->createCollisionGroup(simpleFrame1.get());
  auto group2 = cd->createCollisionGroup(simpleFrame2.get());
  auto group12 = cd->createCollisionGroup(group1.get(), group2.get());

  EXPECT_EQ(group1->getNumShapeFrames(), 1u);
  EXPECT_EQ(group2->getNumShapeFrames(), 1u);
  EXPECT_EQ(group12->getNumShapeFrames(), 2u);

  collision::DistanceOption option;
  option.enableNearestPoints = true;
  EXPECT_TRUE(option.enableNearestPoints == true);
  EXPECT_TRUE(option.distanceLowerBound == 0.0);
  EXPECT_TRUE(option.distanceFilter == nullptr);

  collision::DistanceResult result;
  EXPECT_TRUE(result.found() == false);
  const auto expectNearestPair = [&](const collision::DistanceResult& out,
                                     const Eigen::Vector3d& pointA,
                                     const Eigen::Vector3d& pointB) {
    const bool forward = out.nearestPoint1.isApprox(pointA, tol)
                         && out.nearestPoint2.isApprox(pointB, tol);
    const bool swapped = out.nearestPoint1.isApprox(pointB, tol)
                         && out.nearestPoint2.isApprox(pointA, tol);
    EXPECT_TRUE(forward || swapped);
  };

  result.clear();
  simpleFrame1->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.0));
  simpleFrame2->setTranslation(Eigen::Vector3d(0.75, 0.0, 0.0));
  group1->distance(group2.get(), option, &result);
  EXPECT_DOUBLE_EQ(result.minDistance, 0.0);
  EXPECT_TRUE(
      result.shapeFrame1 == simpleFrame1.get()
      || result.shapeFrame1 == simpleFrame2.get());
  EXPECT_TRUE(
      result.shapeFrame2 == simpleFrame1.get()
      || result.shapeFrame2 == simpleFrame2.get());
  expectNearestPair(
      result,
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.25, 0.0, 0.0));
  EXPECT_TRUE(result.found() == true);

  result.clear();
  simpleFrame1->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.0));
  simpleFrame2->setTranslation(Eigen::Vector3d(0.75, 0.0, 0.0));
  group12->distance(option, &result);
  EXPECT_DOUBLE_EQ(result.minDistance, 0.0);
  EXPECT_TRUE(
      result.shapeFrame1 == simpleFrame1.get()
      || result.shapeFrame1 == simpleFrame2.get());
  EXPECT_TRUE(
      result.shapeFrame2 == simpleFrame1.get()
      || result.shapeFrame2 == simpleFrame2.get());
  expectNearestPair(
      result,
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.25, 0.0, 0.0));
  EXPECT_TRUE(result.found() == true);

  result.clear();
  simpleFrame1->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.0));
  simpleFrame2->setTranslation(Eigen::Vector3d(0.75, 0.0, 0.0));
  cd->distance(group1.get(), group2.get(), option, &result);
  EXPECT_DOUBLE_EQ(result.minDistance, 0.0);
  EXPECT_TRUE(
      result.shapeFrame1 == simpleFrame1.get()
      || result.shapeFrame1 == simpleFrame2.get());
  EXPECT_TRUE(
      result.shapeFrame2 == simpleFrame1.get()
      || result.shapeFrame2 == simpleFrame2.get());
  expectNearestPair(
      result,
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.25, 0.0, 0.0));
  EXPECT_TRUE(result.found() == true);
}

//==============================================================================
TEST(Distance, testBasicInterface)
{
  auto dart = DARTCollisionDetector::create();
  testBasicInterface(dart);
}

//==============================================================================
void testOptions(
    const std::shared_ptr<CollisionDetector>& cd, double tol = 1e-12)
{
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());

  ShapePtr shape1(new EllipsoidShape(Eigen::Vector3d(1.0, 1.0, 1.0)));
  ShapePtr shape2(new EllipsoidShape(Eigen::Vector3d(0.5, 0.5, 0.5)));
  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);

  auto group1 = cd->createCollisionGroup(simpleFrame1.get());
  auto group2 = cd->createCollisionGroup(simpleFrame2.get());
  auto group12 = cd->createCollisionGroup(group1.get(), group2.get());

  EXPECT_EQ(group1->getNumShapeFrames(), 1u);
  EXPECT_EQ(group2->getNumShapeFrames(), 1u);
  EXPECT_EQ(group12->getNumShapeFrames(), 2u);

  collision::DistanceOption option;
  collision::DistanceResult result;
  const auto expectNearestPair = [&](const collision::DistanceResult& out,
                                     const Eigen::Vector3d& pointA,
                                     const Eigen::Vector3d& pointB) {
    const bool forward = out.nearestPoint1.isApprox(pointA, tol)
                         && out.nearestPoint2.isApprox(pointB, tol);
    const bool swapped = out.nearestPoint1.isApprox(pointB, tol)
                         && out.nearestPoint2.isApprox(pointA, tol);
    EXPECT_TRUE(forward || swapped);
  };

  EXPECT_TRUE(option.distanceFilter == nullptr);
  EXPECT_TRUE(option.enableNearestPoints == false);

  EXPECT_TRUE(result.found() == false);

  result.clear();
  simpleFrame1->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.0));
  simpleFrame2->setTranslation(Eigen::Vector3d(0.75, 0.0, 0.0));
  group12->distance(option, &result);
  EXPECT_DOUBLE_EQ(result.minDistance, 0.0);
  EXPECT_TRUE(
      result.shapeFrame1 == simpleFrame1.get()
      || result.shapeFrame1 == simpleFrame2.get());
  EXPECT_TRUE(
      result.shapeFrame2 == simpleFrame1.get()
      || result.shapeFrame2 == simpleFrame2.get());
  EXPECT_EQ(result.nearestPoint1, Eigen::Vector3d::Zero());
  EXPECT_EQ(result.nearestPoint2, Eigen::Vector3d::Zero());
  EXPECT_TRUE(result.found() == true);

  option.enableNearestPoints = true;
  result.clear();
  simpleFrame1->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.0));
  simpleFrame2->setTranslation(Eigen::Vector3d(0.75, 0.0, 0.0));
  group12->distance(option, &result);
  EXPECT_DOUBLE_EQ(result.minDistance, 0.0);
  EXPECT_TRUE(
      result.shapeFrame1 == simpleFrame1.get()
      || result.shapeFrame1 == simpleFrame2.get());
  EXPECT_TRUE(
      result.shapeFrame2 == simpleFrame1.get()
      || result.shapeFrame2 == simpleFrame2.get());
  expectNearestPair(
      result,
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.25, 0.0, 0.0));
  EXPECT_TRUE(result.found() == true);

  option.enableNearestPoints = true;
  result.clear();
  simpleFrame1->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.0));
  simpleFrame2->setTranslation(Eigen::Vector3d(1.0, 0.0, 0.0));
  group12->distance(option, &result);
  EXPECT_DOUBLE_EQ(result.minDistance, 0.0);
  EXPECT_TRUE(
      result.shapeFrame1 == simpleFrame1.get()
      || result.shapeFrame1 == simpleFrame2.get());
  EXPECT_TRUE(
      result.shapeFrame2 == simpleFrame1.get()
      || result.shapeFrame2 == simpleFrame2.get());
  expectNearestPair(
      result,
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.5, 0.0, 0.0));
  EXPECT_TRUE(result.found() == true);
}

//==============================================================================
TEST(Distance, Options)
{
  auto dart = DARTCollisionDetector::create();
  testOptions(dart);
}

//==============================================================================
void testSphereSphere(
    const std::shared_ptr<CollisionDetector>& cd, double tol = 1e-12)
{
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());

  ShapePtr shape1(new EllipsoidShape(Eigen::Vector3d(1.0, 1.0, 1.0)));
  ShapePtr shape2(new EllipsoidShape(Eigen::Vector3d(0.5, 0.5, 0.5)));
  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);

  auto group1 = cd->createCollisionGroup(simpleFrame1.get());
  auto group2 = cd->createCollisionGroup(simpleFrame2.get());
  auto group12 = cd->createCollisionGroup(group1.get(), group2.get());

  EXPECT_EQ(group1->getNumShapeFrames(), 1u);
  EXPECT_EQ(group2->getNumShapeFrames(), 1u);
  EXPECT_EQ(group12->getNumShapeFrames(), 2u);

  collision::DistanceOption option;
  collision::DistanceResult result;

  result.clear();
  simpleFrame1->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.0));
  simpleFrame2->setTranslation(Eigen::Vector3d(0.75, 0.0, 0.0));
  group12->distance(option, &result);
  EXPECT_DOUBLE_EQ(result.minDistance, 0.0);
  EXPECT_TRUE(
      result.shapeFrame1 == simpleFrame1.get()
      || result.shapeFrame1 == simpleFrame2.get());
  EXPECT_TRUE(
      result.shapeFrame2 == simpleFrame1.get()
      || result.shapeFrame2 == simpleFrame2.get());
  EXPECT_TRUE(result.nearestPoint1.isApprox(Eigen::Vector3d::Zero(), tol));
  EXPECT_TRUE(result.nearestPoint2.isApprox(Eigen::Vector3d::Zero(), tol));
}

//==============================================================================
TEST(Distance, SphereSphere)
{
  auto dart = DARTCollisionDetector::create();
  testSphereSphere(dart);
}

//==============================================================================
TEST(Distance, UsesMinimumAcrossPairs)
{
  auto dart = DARTCollisionDetector::create();

  auto frame1 = SimpleFrame::createShared(Frame::World());
  auto frame2 = SimpleFrame::createShared(Frame::World());
  auto frame3 = SimpleFrame::createShared(Frame::World());
  auto frame4 = SimpleFrame::createShared(Frame::World());

  auto sphere = std::make_shared<SphereShape>(0.1);
  frame1->setShape(sphere);
  frame2->setShape(sphere);
  frame3->setShape(sphere);
  frame4->setShape(sphere);

  frame1->setTranslation(Eigen::Vector3d::Zero());
  frame2->setTranslation(Eigen::Vector3d(0.6, 0.0, 0.0));
  frame3->setTranslation(Eigen::Vector3d(5.0, 0.0, 0.0));
  frame4->setTranslation(Eigen::Vector3d(6.0, 0.0, 0.0));

  auto group = dart->createCollisionGroup(
      frame1.get(), frame2.get(), frame3.get(), frame4.get());

  DistanceResult result;
  const double distance = group->distance(DistanceOption(), &result);

  const double expectedDistance = 0.4;
  EXPECT_DOUBLE_EQ(expectedDistance, distance);
  EXPECT_DOUBLE_EQ(expectedDistance, result.minDistance);
  EXPECT_TRUE(
      (result.shapeFrame1 == frame1.get() && result.shapeFrame2 == frame2.get())
      || (result.shapeFrame1 == frame2.get()
          && result.shapeFrame2 == frame1.get()));
}
