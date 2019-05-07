/*
 * Copyright (c) 2011-2019, The DART development contributors
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
#include "dart/collision/fcl/fcl.hpp"
#include "dart/dart.hpp"
#if HAVE_BULLET
#include "dart/collision/bullet/bullet.hpp"
#endif
#include "TestHelpers.hpp"

using namespace dart;

//==============================================================================
void testBasicInterface(const std::shared_ptr<CollisionDetector>& cd)
{
  if (cd->getType() != collision::BulletCollisionDetector::getStaticType())
  {
    dtwarn << "Aborting test: distance check is not supported by "
           << cd->getType() << ".\n";
    return;
  }

  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());

  auto shape1 = std::make_shared<SphereShape>(1.0);
  simpleFrame1->setShape(shape1);

  auto group1 = cd->createCollisionGroup(simpleFrame1.get());

  EXPECT_EQ(group1->getNumShapeFrames(), 1u);

  collision::RaycastOption option;
  option.mEnableAllHits = false;

  collision::RaycastResult result;
  EXPECT_FALSE(result.hasHit());

  RayHit rayHit;

  result.clear();
  simpleFrame1->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.0));
  cd->raycast(
      group1.get(),
      Eigen::Vector3d(-2, 0, 0),
      Eigen::Vector3d(2, 0, 0),
      option,
      &result);
  EXPECT_TRUE(result.hasHit());
  EXPECT_EQ(result.mRayHits.size(), 1u);
  rayHit = result.mRayHits[0];
  EXPECT_TRUE(equals(rayHit.mPoint, Eigen::Vector3d(-1, 0, 0)));
  EXPECT_TRUE(equals(rayHit.mNormal, Eigen::Vector3d(-1, 0, 0)));
  EXPECT_DOUBLE_EQ(rayHit.mFraction, 0.25);

  result.clear();
  simpleFrame1->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.0));
  simpleFrame1->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.0));
  cd->raycast(
      group1.get(),
      Eigen::Vector3d(2, 0, 0),
      Eigen::Vector3d(-2, 0, 0),
      option,
      &result);
  EXPECT_TRUE(result.hasHit());
  EXPECT_EQ(result.mRayHits.size(), 1u);
  rayHit = result.mRayHits[0];
  EXPECT_TRUE(equals(rayHit.mPoint, Eigen::Vector3d(1, 0, 0)));
  EXPECT_TRUE(equals(rayHit.mNormal, Eigen::Vector3d(1, 0, 0)));
  EXPECT_DOUBLE_EQ(rayHit.mFraction, 0.25);

  result.clear();
  simpleFrame1->setTranslation(Eigen::Vector3d(1.0, 0.0, 0.0));
  cd->raycast(
      group1.get(),
      Eigen::Vector3d(-2, 0, 0),
      Eigen::Vector3d(2, 0, 0),
      option,
      &result);
  EXPECT_TRUE(result.hasHit());
  EXPECT_EQ(result.mRayHits.size(), 1u);
  rayHit = result.mRayHits[0];
  EXPECT_TRUE(equals(rayHit.mPoint, Eigen::Vector3d(0, 0, 0)));
  EXPECT_TRUE(equals(rayHit.mNormal, Eigen::Vector3d(-1, 0, 0)));
  EXPECT_DOUBLE_EQ(rayHit.mFraction, 0.5);
}

//==============================================================================
TEST(Distance, testBasicInterface)
{
  auto fcl = FCLCollisionDetector::create();
  testBasicInterface(fcl);

#if HAVE_BULLET
  auto bullet = BulletCollisionDetector::create();
  testBasicInterface(bullet);
#endif

  auto dart = DARTCollisionDetector::create();
  testBasicInterface(dart);
}

//==============================================================================
void testOptions(const std::shared_ptr<CollisionDetector>& cd)
{
  if (cd->getType() != collision::BulletCollisionDetector::getStaticType())
  {
    dtwarn << "Aborting test: distance check is not supported by "
           << cd->getType() << ".\n";
    return;
  }

  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto shape1 = std::make_shared<SphereShape>(1.0);
  simpleFrame1->setShape(shape1);

  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());
  auto shape2 = std::make_shared<SphereShape>(1.0);
  simpleFrame2->setShape(shape1);

  auto group = cd->createCollisionGroup(simpleFrame1.get(), simpleFrame2.get());

  collision::RaycastOption option;
  option.mEnableAllHits = false;

  collision::RaycastResult result;
  EXPECT_FALSE(result.hasHit());

  RayHit rayHit;

  result.clear();
  option.mEnableAllHits = false;
  option.mSortByClosest = false;
  simpleFrame1->setTranslation(Eigen::Vector3d(-2, 0, 0));
  simpleFrame2->setTranslation(Eigen::Vector3d(2, 0, 0));
  cd->raycast(
      group.get(),
      Eigen::Vector3d(-5, 0, 0),
      Eigen::Vector3d(5, 0, 0),
      option,
      &result);
  EXPECT_TRUE(result.hasHit());
  EXPECT_EQ(result.mRayHits.size(), 1u);
  rayHit = result.mRayHits[0];
  EXPECT_TRUE(equals(rayHit.mPoint, Eigen::Vector3d(-3, 0, 0)));
  EXPECT_TRUE(equals(rayHit.mNormal, Eigen::Vector3d(-1, 0, 0)));
  EXPECT_NEAR(rayHit.mFraction, 0.2, 1e-5);

  result.clear();
  option.mEnableAllHits = true;
  option.mSortByClosest = true;
  simpleFrame1->setTranslation(Eigen::Vector3d(-2, 0, 0));
  simpleFrame2->setTranslation(Eigen::Vector3d(2, 0, 0));
  cd->raycast(
      group.get(),
      Eigen::Vector3d(-5, 0, 0),
      Eigen::Vector3d(5, 0, 0),
      option,
      &result);
  EXPECT_TRUE(result.hasHit());
  EXPECT_EQ(result.mRayHits.size(), 2u);
  rayHit = result.mRayHits[0];
  EXPECT_TRUE(equals(rayHit.mPoint, Eigen::Vector3d(-3, 0, 0)));
  EXPECT_TRUE(equals(rayHit.mNormal, Eigen::Vector3d(-1, 0, 0)));
  EXPECT_NEAR(rayHit.mFraction, 0.2, 1e-5);
  rayHit = result.mRayHits[1];
  EXPECT_TRUE(equals(rayHit.mPoint, Eigen::Vector3d(1, 0, 0)));
  EXPECT_TRUE(equals(rayHit.mNormal, Eigen::Vector3d(-1, 0, 0)));
  EXPECT_NEAR(rayHit.mFraction, 0.6, 1e-5);
}

//==============================================================================
TEST(Distance, testOptions)
{
  auto fcl = FCLCollisionDetector::create();
  testOptions(fcl);

#if HAVE_BULLET
  auto bullet = BulletCollisionDetector::create();
  testOptions(bullet);
#endif

  auto dart = DARTCollisionDetector::create();
  testOptions(dart);
}
