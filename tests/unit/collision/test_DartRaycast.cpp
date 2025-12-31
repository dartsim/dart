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

#include "../../helpers/GTestUtils.hpp"

using namespace dart;
using namespace collision;
using namespace dynamics;
using namespace dart::test;

namespace {

constexpr double kFractionTolerance = 1e-5;
constexpr double kPi = 3.141592653589793;

} // namespace

//==============================================================================
TEST(DartRaycast, BasicInterface)
{
  auto detector = DARTCollisionDetector::create();

  auto frame = SimpleFrame::createShared(Frame::World());
  frame->setShape(std::make_shared<SphereShape>(1.0));

  auto group = detector->createCollisionGroup(frame.get());

  RaycastOption option;
  option.mEnableAllHits = false;

  RaycastResult result;
  EXPECT_FALSE(result.hasHit());

  RayHit rayHit;

  result.clear();
  frame->setTranslation(Eigen::Vector3d(5.0, 0.0, 0.0));
  detector->raycast(
      group.get(),
      Eigen::Vector3d(-2.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, 0.0, 0.0),
      option,
      &result);
  EXPECT_FALSE(result.hasHit());

  result.clear();
  frame->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.0));
  detector->raycast(
      group.get(),
      Eigen::Vector3d(-2.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, 0.0, 0.0),
      option,
      &result);
  EXPECT_TRUE(result.hasHit());
  ASSERT_EQ(result.mRayHits.size(), 1u);
  rayHit = result.mRayHits[0];
  EXPECT_TRUE(equals(rayHit.mPoint, Eigen::Vector3d(-1.0, 0.0, 0.0)));
  EXPECT_TRUE(equals(rayHit.mNormal, Eigen::Vector3d(-1.0, 0.0, 0.0)));
  EXPECT_NEAR(rayHit.mFraction, 0.25, kFractionTolerance);

  result.clear();
  frame->setTranslation(Eigen::Vector3d(1.0, 0.0, 0.0));
  detector->raycast(
      group.get(),
      Eigen::Vector3d(-2.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, 0.0, 0.0),
      option,
      &result);
  EXPECT_TRUE(result.hasHit());
  ASSERT_EQ(result.mRayHits.size(), 1u);
  rayHit = result.mRayHits[0];
  EXPECT_TRUE(equals(rayHit.mPoint, Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_TRUE(equals(rayHit.mNormal, Eigen::Vector3d(-1.0, 0.0, 0.0)));
  EXPECT_NEAR(rayHit.mFraction, 0.5, kFractionTolerance);
}

//==============================================================================
TEST(DartRaycast, EllipsoidHit)
{
  auto detector = DARTCollisionDetector::create();

  auto frame = SimpleFrame::createShared(Frame::World());
  frame->setShape(
      std::make_shared<EllipsoidShape>(Eigen::Vector3d(1.0, 1.0, 1.0)));

  auto group = detector->createCollisionGroup(frame.get());

  RaycastOption option;
  RaycastResult result;

  detector->raycast(
      group.get(),
      Eigen::Vector3d(-2.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, 0.0, 0.0),
      option,
      &result);
  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(result.mRayHits.size(), 1u);
  const auto& hit = result.mRayHits[0];
  EXPECT_TRUE(equals(hit.mPoint, Eigen::Vector3d(-1.0, 0.0, 0.0)));
  EXPECT_TRUE(equals(hit.mNormal, Eigen::Vector3d(-1.0, 0.0, 0.0)));
  EXPECT_NEAR(hit.mFraction, 0.25, kFractionTolerance);
}

//==============================================================================
TEST(DartRaycast, InsideHits)
{
  auto detector = DARTCollisionDetector::create();

  RaycastOption option;
  RaycastResult result;

  auto sphereFrame = SimpleFrame::createShared(Frame::World());
  sphereFrame->setShape(std::make_shared<SphereShape>(1.0));
  auto sphereGroup = detector->createCollisionGroup(sphereFrame.get());
  detector->raycast(
      sphereGroup.get(),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, 0.0, 0.0),
      option,
      &result);
  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(result.mRayHits.size(), 1u);
  auto rayHit = result.mRayHits[0];
  EXPECT_TRUE(equals(rayHit.mPoint, Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_TRUE(equals(rayHit.mNormal, Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_NEAR(rayHit.mFraction, 0.5, kFractionTolerance);

  result.clear();
  auto boxFrame = SimpleFrame::createShared(Frame::World());
  boxFrame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(2.0, 2.0, 2.0)));
  auto boxGroup = detector->createCollisionGroup(boxFrame.get());
  detector->raycast(
      boxGroup.get(),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(3.0, 0.0, 0.0),
      option,
      &result);
  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(result.mRayHits.size(), 1u);
  rayHit = result.mRayHits[0];
  EXPECT_TRUE(equals(rayHit.mPoint, Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_TRUE(equals(rayHit.mNormal, Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_NEAR(rayHit.mFraction, 1.0 / 3.0, kFractionTolerance);

  result.clear();
  auto cylinderFrame = SimpleFrame::createShared(Frame::World());
  cylinderFrame->setShape(std::make_shared<CylinderShape>(1.0, 2.0));
  auto cylinderGroup = detector->createCollisionGroup(cylinderFrame.get());
  detector->raycast(
      cylinderGroup.get(),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, 3.0),
      option,
      &result);
  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(result.mRayHits.size(), 1u);
  rayHit = result.mRayHits[0];
  EXPECT_TRUE(equals(rayHit.mPoint, Eigen::Vector3d(0.0, 0.0, 1.0)));
  EXPECT_TRUE(equals(rayHit.mNormal, Eigen::Vector3d(0.0, 0.0, 1.0)));
  EXPECT_NEAR(rayHit.mFraction, 1.0 / 3.0, kFractionTolerance);
}

//==============================================================================
TEST(DartRaycast, TangentSphereHit)
{
  auto detector = DARTCollisionDetector::create();

  auto frame = SimpleFrame::createShared(Frame::World());
  frame->setShape(std::make_shared<SphereShape>(1.0));

  auto group = detector->createCollisionGroup(frame.get());

  RaycastOption option;
  RaycastResult result;

  detector->raycast(
      group.get(),
      Eigen::Vector3d(-2.0, 1.0, 0.0),
      Eigen::Vector3d(2.0, 1.0, 0.0),
      option,
      &result);
  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(result.mRayHits.size(), 1u);
  const auto& hit = result.mRayHits[0];
  EXPECT_TRUE(equals(hit.mPoint, Eigen::Vector3d(0.0, 1.0, 0.0)));
  EXPECT_TRUE(equals(hit.mNormal, Eigen::Vector3d(0.0, 1.0, 0.0)));
  EXPECT_NEAR(hit.mFraction, 0.5, kFractionTolerance);
}

//==============================================================================
TEST(DartRaycast, TangentCylinderHit)
{
  auto detector = DARTCollisionDetector::create();

  auto frame = SimpleFrame::createShared(Frame::World());
  frame->setShape(std::make_shared<CylinderShape>(1.0, 2.0));

  auto group = detector->createCollisionGroup(frame.get());

  RaycastOption option;
  RaycastResult result;

  detector->raycast(
      group.get(),
      Eigen::Vector3d(-2.0, 1.0, 0.0),
      Eigen::Vector3d(2.0, 1.0, 0.0),
      option,
      &result);
  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(result.mRayHits.size(), 1u);
  const auto& hit = result.mRayHits[0];
  EXPECT_TRUE(equals(hit.mPoint, Eigen::Vector3d(0.0, 1.0, 0.0)));
  EXPECT_TRUE(equals(hit.mNormal, Eigen::Vector3d(0.0, 1.0, 0.0)));
  EXPECT_NEAR(hit.mFraction, 0.5, kFractionTolerance);
}

//==============================================================================
TEST(DartRaycast, Options)
{
  auto detector = DARTCollisionDetector::create();

  auto frame1 = SimpleFrame::createShared(Frame::World());
  frame1->setShape(std::make_shared<SphereShape>(1.0));

  auto frame2 = SimpleFrame::createShared(Frame::World());
  frame2->setShape(std::make_shared<SphereShape>(1.0));

  auto group = detector->createCollisionGroup(frame1.get(), frame2.get());

  RaycastOption option;
  option.mEnableAllHits = true;
  option.mSortByClosest = true;

  RaycastResult result;

  frame1->setTranslation(Eigen::Vector3d(-2.0, 0.0, 0.0));
  frame2->setTranslation(Eigen::Vector3d(2.0, 0.0, 0.0));

  detector->raycast(
      group.get(),
      Eigen::Vector3d(-5.0, 0.0, 0.0),
      Eigen::Vector3d(5.0, 0.0, 0.0),
      option,
      &result);
  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(result.mRayHits.size(), 2u);

  auto rayHit = result.mRayHits[0];
  EXPECT_TRUE(equals(rayHit.mPoint, Eigen::Vector3d(-3.0, 0.0, 0.0)));
  EXPECT_TRUE(equals(rayHit.mNormal, Eigen::Vector3d(-1.0, 0.0, 0.0)));
  EXPECT_NEAR(rayHit.mFraction, 0.2, kFractionTolerance);

  rayHit = result.mRayHits[1];
  EXPECT_TRUE(equals(rayHit.mPoint, Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_TRUE(equals(rayHit.mNormal, Eigen::Vector3d(-1.0, 0.0, 0.0)));
  EXPECT_NEAR(rayHit.mFraction, 0.6, kFractionTolerance);
}

//==============================================================================
TEST(DartRaycast, Filters)
{
  auto detector = DARTCollisionDetector::create();

  auto frame1 = SimpleFrame::createShared(Frame::World());
  auto frame2 = SimpleFrame::createShared(Frame::World());

  auto sphere = std::make_shared<SphereShape>(1.0);
  frame1->setShape(sphere);
  frame2->setShape(sphere);

  auto group = detector->createCollisionGroup(frame1.get(), frame2.get());

  RaycastOption option;
  RaycastResult result;

  frame1->setTranslation(Eigen::Vector3d(-2.0, 0.0, 0.0));
  frame2->setTranslation(Eigen::Vector3d(2.0, 0.0, 0.0));

  option.mEnableAllHits = false;
  option.mSortByClosest = false;
  option.mFilter = [&](const collision::CollisionObject* obj) {
    return obj->getShapeFrame() == frame2.get();
  };

  detector->raycast(
      group.get(),
      Eigen::Vector3d(-5.0, 0.0, 0.0),
      Eigen::Vector3d(5.0, 0.0, 0.0),
      option,
      &result);
  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(result.mRayHits.size(), 1u);
  auto rayHit = result.mRayHits[0];
  EXPECT_TRUE(equals(rayHit.mPoint, Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_TRUE(equals(rayHit.mNormal, Eigen::Vector3d(-1.0, 0.0, 0.0)));
  EXPECT_NEAR(rayHit.mFraction, 0.6, kFractionTolerance);

  option.mEnableAllHits = true;
  option.mFilter = [&](const collision::CollisionObject* obj) {
    return obj->getShapeFrame() == frame1.get();
  };
  result.clear();
  detector->raycast(
      group.get(),
      Eigen::Vector3d(-5.0, 0.0, 0.0),
      Eigen::Vector3d(5.0, 0.0, 0.0),
      option,
      &result);
  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(result.mRayHits.size(), 1u);
  rayHit = result.mRayHits[0];
  EXPECT_TRUE(equals(rayHit.mPoint, Eigen::Vector3d(-3.0, 0.0, 0.0)));
  EXPECT_TRUE(equals(rayHit.mNormal, Eigen::Vector3d(-1.0, 0.0, 0.0)));
  EXPECT_NEAR(rayHit.mFraction, 0.2, kFractionTolerance);

  option.mFilter = [&](const collision::CollisionObject*) { return false; };
  result.clear();
  detector->raycast(
      group.get(),
      Eigen::Vector3d(-5.0, 0.0, 0.0),
      Eigen::Vector3d(5.0, 0.0, 0.0),
      option,
      &result);
  EXPECT_FALSE(result.hasHit());
  EXPECT_TRUE(result.mRayHits.empty());
}

//==============================================================================
TEST(DartRaycast, BoxHits)
{
  auto detector = DARTCollisionDetector::create();

  auto frame = SimpleFrame::createShared(Frame::World());
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(2.0, 2.0, 2.0)));

  auto group = detector->createCollisionGroup(frame.get());

  RaycastOption option;
  RaycastResult result;

  frame->setTranslation(Eigen::Vector3d::Zero());
  detector->raycast(
      group.get(),
      Eigen::Vector3d(-3.0, 0.0, 0.0),
      Eigen::Vector3d(3.0, 0.0, 0.0),
      option,
      &result);
  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(result.mRayHits.size(), 1u);
  const auto& hit = result.mRayHits[0];
  EXPECT_TRUE(equals(hit.mPoint, Eigen::Vector3d(-1.0, 0.0, 0.0)));
  EXPECT_TRUE(equals(hit.mNormal, Eigen::Vector3d(-1.0, 0.0, 0.0)));
  EXPECT_NEAR(hit.mFraction, 1.0 / 3.0, kFractionTolerance);
}

//==============================================================================
TEST(DartRaycast, CylinderHits)
{
  auto detector = DARTCollisionDetector::create();

  auto frame = SimpleFrame::createShared(Frame::World());
  frame->setShape(std::make_shared<CylinderShape>(1.0, 2.0));

  auto group = detector->createCollisionGroup(frame.get());

  RaycastOption option;
  RaycastResult result;

  frame->setTranslation(Eigen::Vector3d::Zero());
  detector->raycast(
      group.get(),
      Eigen::Vector3d(-3.0, 0.0, 0.0),
      Eigen::Vector3d(3.0, 0.0, 0.0),
      option,
      &result);
  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(result.mRayHits.size(), 1u);
  auto hit = result.mRayHits[0];
  EXPECT_TRUE(equals(hit.mPoint, Eigen::Vector3d(-1.0, 0.0, 0.0)));
  EXPECT_TRUE(equals(hit.mNormal, Eigen::Vector3d(-1.0, 0.0, 0.0)));
  EXPECT_NEAR(hit.mFraction, 1.0 / 3.0, kFractionTolerance);

  result.clear();
  detector->raycast(
      group.get(),
      Eigen::Vector3d(0.0, 0.0, 3.0),
      Eigen::Vector3d(0.0, 0.0, -3.0),
      option,
      &result);
  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(result.mRayHits.size(), 1u);
  hit = result.mRayHits[0];
  EXPECT_TRUE(equals(hit.mPoint, Eigen::Vector3d(0.0, 0.0, 1.0)));
  EXPECT_TRUE(equals(hit.mNormal, Eigen::Vector3d(0.0, 0.0, 1.0)));
  EXPECT_NEAR(hit.mFraction, 1.0 / 3.0, kFractionTolerance);
}

//==============================================================================
TEST(DartRaycast, PlaneHits)
{
  auto detector = DARTCollisionDetector::create();

  auto frame = SimpleFrame::createShared(Frame::World());
  frame->setShape(std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));

  auto group = detector->createCollisionGroup(frame.get());

  RaycastOption option;
  RaycastResult result;

  detector->raycast(
      group.get(),
      Eigen::Vector3d(0.0, 0.0, 2.0),
      Eigen::Vector3d(0.0, 0.0, -2.0),
      option,
      &result);
  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(result.mRayHits.size(), 1u);
  const auto& hit = result.mRayHits[0];
  EXPECT_TRUE(equals(hit.mPoint, Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_TRUE(equals(hit.mNormal, Eigen::Vector3d(0.0, 0.0, 1.0)));
  EXPECT_NEAR(hit.mFraction, 0.5, kFractionTolerance);
}

//==============================================================================
TEST(DartRaycast, PlaneOffsetHit)
{
  auto detector = DARTCollisionDetector::create();

  auto frame = SimpleFrame::createShared(Frame::World());
  frame->setShape(std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 1.0));

  auto group = detector->createCollisionGroup(frame.get());

  RaycastOption option;
  RaycastResult result;

  detector->raycast(
      group.get(),
      Eigen::Vector3d(0.0, 0.0, 2.0),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      option,
      &result);
  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(result.mRayHits.size(), 1u);
  const auto& hit = result.mRayHits[0];
  EXPECT_TRUE(equals(hit.mPoint, Eigen::Vector3d(0.0, 0.0, 1.0)));
  EXPECT_TRUE(equals(hit.mNormal, Eigen::Vector3d(0.0, 0.0, 1.0)));
  EXPECT_NEAR(hit.mFraction, 0.5, kFractionTolerance);
}

//==============================================================================
TEST(DartRaycast, RotatedPlaneHit)
{
  auto detector = DARTCollisionDetector::create();

  auto frame = SimpleFrame::createShared(Frame::World());
  frame->setShape(std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));
  frame->setRotation(
      Eigen::AngleAxisd(0.5 * kPi, Eigen::Vector3d::UnitY()).toRotationMatrix());

  auto group = detector->createCollisionGroup(frame.get());

  RaycastOption option;
  RaycastResult result;

  detector->raycast(
      group.get(),
      Eigen::Vector3d(2.0, 0.0, 0.0),
      Eigen::Vector3d(-2.0, 0.0, 0.0),
      option,
      &result);
  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(result.mRayHits.size(), 1u);
  const auto& hit = result.mRayHits[0];
  EXPECT_TRUE(equals(hit.mPoint, Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_TRUE(equals(hit.mNormal, Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_NEAR(hit.mFraction, 0.5, kFractionTolerance);
}

//==============================================================================
TEST(DartRaycast, RotatedBoxHit)
{
  auto detector = DARTCollisionDetector::create();

  auto frame = SimpleFrame::createShared(Frame::World());
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(2.0, 2.0, 2.0)));
  frame->setRotation(
      Eigen::AngleAxisd(0.25 * kPi, Eigen::Vector3d::UnitZ()).toRotationMatrix());

  auto group = detector->createCollisionGroup(frame.get());

  RaycastOption option;
  RaycastResult result;

  const double root2 = std::sqrt(2.0);
  const Eigen::Vector3d normal(root2 * 0.5, root2 * 0.5, 0.0);
  const Eigen::Vector3d from = normal * 3.0;
  const Eigen::Vector3d to = -normal;

  detector->raycast(group.get(), from, to, option, &result);
  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(result.mRayHits.size(), 1u);
  const auto& hit = result.mRayHits[0];
  EXPECT_TRUE(equals(hit.mPoint, normal));
  EXPECT_TRUE(equals(hit.mNormal, normal));
  EXPECT_NEAR(hit.mFraction, 0.5, kFractionTolerance);
}

//==============================================================================
TEST(DartRaycast, RotatedCylinderHit)
{
  auto detector = DARTCollisionDetector::create();

  auto frame = SimpleFrame::createShared(Frame::World());
  frame->setShape(std::make_shared<CylinderShape>(1.0, 2.0));
  frame->setRotation(
      Eigen::AngleAxisd(0.5 * kPi, Eigen::Vector3d::UnitY()).toRotationMatrix());

  auto group = detector->createCollisionGroup(frame.get());

  RaycastOption option;
  RaycastResult result;

  detector->raycast(
      group.get(),
      Eigen::Vector3d(-3.0, 0.0, 0.0),
      Eigen::Vector3d(3.0, 0.0, 0.0),
      option,
      &result);
  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(result.mRayHits.size(), 1u);
  const auto& hit = result.mRayHits[0];
  EXPECT_TRUE(equals(hit.mPoint, Eigen::Vector3d(-1.0, 0.0, 0.0)));
  EXPECT_TRUE(equals(hit.mNormal, Eigen::Vector3d(-1.0, 0.0, 0.0)));
  EXPECT_NEAR(hit.mFraction, 1.0 / 3.0, kFractionTolerance);
}

//==============================================================================
TEST(DartRaycast, ParallelPlaneMiss)
{
  auto detector = DARTCollisionDetector::create();

  auto frame = SimpleFrame::createShared(Frame::World());
  frame->setShape(std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));

  auto group = detector->createCollisionGroup(frame.get());

  RaycastOption option;
  RaycastResult result;

  detector->raycast(
      group.get(),
      Eigen::Vector3d(0.0, 0.0, 1.0),
      Eigen::Vector3d(1.0, 0.0, 1.0),
      option,
      &result);
  EXPECT_FALSE(result.hasHit());
  EXPECT_TRUE(result.mRayHits.empty());
}
