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

/// \file test_Issue2470.cpp
/// \brief Regression test for https://github.com/dartsim/dart/issues/2470
///
/// Verifies sphere-sphere collision with DARTCollisionDetector does not cause
/// memory corruption when accessing Contact.point after collision detection.

#include <dart/collision/dart/DARTCollisionDetector.hpp>

#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/SphereShape.hpp>

#include <gtest/gtest.h>

using namespace dart;

//==============================================================================
TEST(Issue2470, SphereSphereCollisionContactAccess)
{
  auto cd = collision::DARTCollisionDetector::create();

  auto simpleFrame1
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  auto simpleFrame2
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());

  auto shape1 = std::make_shared<dynamics::SphereShape>(1.0);
  auto shape2 = std::make_shared<dynamics::SphereShape>(0.5);

  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);

  auto group = cd->createCollisionGroup(simpleFrame1.get(), simpleFrame2.get());

  collision::CollisionOption option;
  option.enableContact = true;
  collision::CollisionResult result;

  const double tol = 1e-12;
  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d(1.5 - tol, 0.0, 0.0));

  result.clear();
  bool collided = group->collide(option, &result);

  EXPECT_TRUE(collided);
  EXPECT_GT(result.getNumContacts(), 0u);

  if (result.getNumContacts() > 0) {
    const auto& contact = result.getContact(0);

    // Issue #2470: isApprox() triggered SIGSEGV on contact.point
    Eigen::Vector3d expectedPoint = Eigen::Vector3d::UnitX();
    bool pointMatch = contact.point.isApprox(expectedPoint, 2.0 * tol);
    (void)pointMatch;

    EXPECT_NEAR(contact.point.x(), 1.0, 0.1);
    EXPECT_NEAR(contact.point.y(), 0.0, 0.1);
    EXPECT_NEAR(contact.point.z(), 0.0, 0.1);

    EXPECT_FALSE(contact.normal.hasNaN());
    EXPECT_NEAR(contact.normal.norm(), 1.0, 1e-6);
  }
}

//==============================================================================
TEST(Issue2470, SphereSphereOverlappingCollision)
{
  auto cd = collision::DARTCollisionDetector::create();

  auto simpleFrame1
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  auto simpleFrame2
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());

  auto shape1 = std::make_shared<dynamics::SphereShape>(1.0);
  auto shape2 = std::make_shared<dynamics::SphereShape>(0.5);

  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);

  auto group = cd->createCollisionGroup(simpleFrame1.get(), simpleFrame2.get());

  collision::CollisionOption option;
  option.enableContact = true;
  collision::CollisionResult result;

  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d(1.0, 0.0, 0.0));

  result.clear();
  bool collided = group->collide(option, &result);

  EXPECT_TRUE(collided);
  EXPECT_GT(result.getNumContacts(), 0u);

  if (result.getNumContacts() > 0) {
    const auto& contact = result.getContact(0);

    EXPECT_FALSE(contact.point.hasNaN());
    EXPECT_FALSE(contact.normal.hasNaN());
    EXPECT_NEAR(contact.normal.norm(), 1.0, 1e-6);
    EXPECT_GT(contact.penetrationDepth, 0.0);
  }
}

//==============================================================================
TEST(Issue2470, SphereSphereNoCollision)
{
  auto cd = collision::DARTCollisionDetector::create();

  auto simpleFrame1
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  auto simpleFrame2
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());

  auto shape1 = std::make_shared<dynamics::SphereShape>(1.0);
  auto shape2 = std::make_shared<dynamics::SphereShape>(0.5);

  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);

  auto group = cd->createCollisionGroup(simpleFrame1.get(), simpleFrame2.get());

  collision::CollisionOption option;
  option.enableContact = true;
  collision::CollisionResult result;

  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d(2.0, 0.0, 0.0));

  result.clear();
  bool collided = group->collide(option, &result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.getNumContacts(), 0u);
}

//==============================================================================
TEST(Issue2470, EqualSphereCollisionMultipleAxes)
{
  auto cd = collision::DARTCollisionDetector::create();

  auto simpleFrame1
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  auto simpleFrame2
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());

  const double radius = 1.0;
  auto shape1 = std::make_shared<dynamics::SphereShape>(radius);
  auto shape2 = std::make_shared<dynamics::SphereShape>(radius);

  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);

  auto group = cd->createCollisionGroup(simpleFrame1.get(), simpleFrame2.get());

  collision::CollisionOption option;
  option.enableContact = true;
  collision::CollisionResult result;

  std::vector<Eigen::Vector3d> directions
      = {Eigen::Vector3d::UnitX(),
         Eigen::Vector3d::UnitY(),
         Eigen::Vector3d::UnitZ(),
         Eigen::Vector3d(1, 1, 0).normalized(),
         Eigen::Vector3d(1, 1, 1).normalized()};

  for (const auto& dir : directions) {
    const double tol = 1e-12;
    simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
    simpleFrame2->setTranslation(dir * (2.0 * radius - tol));

    result.clear();
    bool collided = group->collide(option, &result);

    EXPECT_TRUE(collided) << "Direction: " << dir.transpose();
    EXPECT_GT(result.getNumContacts(), 0u) << "Direction: " << dir.transpose();

    if (result.getNumContacts() > 0) {
      const auto& contact = result.getContact(0);

      EXPECT_FALSE(contact.point.hasNaN()) << "Direction: " << dir.transpose();
      EXPECT_FALSE(contact.normal.hasNaN()) << "Direction: " << dir.transpose();
    }
  }
}
