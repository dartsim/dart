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
TEST(DartCollisionPrimitives, CylinderCylinder)
{
  auto detector = DARTCollisionDetector::create();

  auto frame1 = SimpleFrame::createShared(Frame::World());
  auto frame2 = SimpleFrame::createShared(Frame::World());

  frame1->setShape(std::make_shared<CylinderShape>(1.0, 2.0));
  frame2->setShape(std::make_shared<CylinderShape>(1.0, 2.0));

  auto group = detector->createCollisionGroup(frame1.get(), frame2.get());

  CollisionOption option;
  option.enableContact = true;

  CollisionResult result;

  frame1->setTranslation(Eigen::Vector3d::Zero());
  frame2->setTranslation(Eigen::Vector3d(3.0, 0.0, 0.0));
  result.clear();
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 0u);

  frame2->setTranslation(Eigen::Vector3d(0.0, 0.0, 3.0));
  result.clear();
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 0u);

  frame2->setTranslation(Eigen::Vector3d(1.5, 0.0, 0.0));
  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_GE(result.getNumContacts(), 1u);
}

//==============================================================================
TEST(DartCollisionPrimitives, BoxCylinder)
{
  auto detector = DARTCollisionDetector::create();

  auto boxFrame = SimpleFrame::createShared(Frame::World());
  auto cylinderFrame = SimpleFrame::createShared(Frame::World());

  boxFrame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(2.0, 2.0, 2.0)));
  cylinderFrame->setShape(std::make_shared<CylinderShape>(0.5, 2.0));

  auto group = detector->createCollisionGroup(
      boxFrame.get(), cylinderFrame.get());

  CollisionOption option;
  option.enableContact = true;

  CollisionResult result;

  boxFrame->setTranslation(Eigen::Vector3d::Zero());
  cylinderFrame->setTranslation(Eigen::Vector3d(2.0, 0.0, 0.0));
  result.clear();
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 0u);

  cylinderFrame->setTranslation(Eigen::Vector3d(1.25, 0.0, 0.0));
  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_GE(result.getNumContacts(), 1u);

  cylinderFrame->setTranslation(Eigen::Vector3d::Zero());
  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_GE(result.getNumContacts(), 1u);
}
